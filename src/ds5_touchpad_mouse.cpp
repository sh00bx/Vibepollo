/**
 * @file src/ds5_touchpad_mouse.cpp
 * @brief DualSense touchpad as a desktop mouse (see header).
 */
// standard includes
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <mutex>

// lib includes
#include <moonlight-common-c/src/Limelight.h>

// local includes
#include "config.h"
#include "ds5_touchpad_mouse.h"
#include "input.h"
#include "logging.h"
#include "platform/common.h"
#include "process.h"
#ifdef _WIN32
  #include "platform/windows/playnite_integration.h"
#endif

using namespace std::chrono_literals;

namespace tpmouse {

  namespace {

    // DS5 touchpad sensor space (hid-playstation).
    constexpr int PAD_W = 1920;
    constexpr int PAD_H = 1080;

    // A tap is a short, still contact.
    constexpr auto TAP_MAX_DURATION = 280ms;
    constexpr int TAP_MAX_TRAVEL = 14;  // pad units, euclidean

    // Two-finger scroll: pad units of vertical travel per wheel notch.
    constexpr int SCROLL_UNITS_PER_NOTCH = 36;

    // Moonlight mouse button ids as understood by platf::button_mouse().
    constexpr int MOUSE_LEFT = 1;
    constexpr int MOUSE_RIGHT = 3;

    struct contact_t {
      bool down {false};
      uint8_t id {0};  // 7-bit contact counter from the pad / pointer slot tag
      int x {0}, y {0};
      int start_x {0}, start_y {0};
      std::chrono::steady_clock::time_point t_down {};
      bool moved {false};
    };

    struct state_t {
      std::mutex mtx;
      contact_t c[2];
      int fingers_seen {0};   // max concurrent contacts during the current gesture
      bool click_down {false};
      int click_button {MOUSE_LEFT};
      bool gesture_clicked {false};  // physical click happened during this contact
      float acc_x {0.f}, acc_y {0.f};
      int scroll_acc {0};
      // Pointer velocity tracking (libinput-style, see move_pointer_locked)
      std::chrono::steady_clock::time_point last_move_time {};
      bool have_move_time {false};
      double velocity_ema {0.0};  // device units per µs, smoothed
      // SDL path: map moonlight pointerId -> slot
      uint32_t ptr_id[2] {0, 0};
      bool ptr_used[2] {false, false};
    };

    state_t st;

    // The gate is evaluated at report rate; cache the verdict briefly so the
    // config mutex and process lock are not taken 250 times a second.
    std::atomic<int> gate_cache {-1};
    std::atomic<int64_t> gate_stamp_ms {0};

    int64_t now_ms() {
      return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
    }

    bool evaluate_gate() {
      std::string mode;
      {
        std::lock_guard lk(config::ds5b_mutex);
        mode = config::ds5b.touchpad_mouse;
      }
      if (mode == "off") {
        return false;
      }
      if (mode == "always") {
        return true;
      }
      // "auto": only while no game is actually running. Two signals, because
      // the launched moonlight app alone is not enough — a Desktop session
      // with a game started from inside it (the normal Playnite flow here)
      // still says "Desktop":
      // 1. Playnite's live gameStarted/gameStopped status. Authoritative for
      //    every Playnite-managed launch, no matter how the stream began.
#ifdef _WIN32
      if (platf::playnite::get_active_game_status().active) {
        return false;
      }
#endif
      // 2. The streamed app's own metadata: anything that launches something
      //    (cmd, Playnite target, detached) owns the touchpad.
      if (proc::proc.running() == 0) {
        // No app at all: no active stream is feeding us anyway; allow, so the
        // brief window during session start behaves like the desktop it shows.
        return true;
      }
      return proc::proc.running_app_launches_nothing();
    }

    int speed_percent() {
      std::lock_guard lk(config::ds5b_mutex);
      return config::ds5b.touchpad_mouse_speed;
    }

    bool natural_scroll() {
      std::lock_guard lk(config::ds5b_mutex);
      return config::ds5b.touchpad_mouse_natural_scroll;
    }

    void release_click_locked() {
      if (st.click_down) {
        platf::button_mouse(input::raw_platf_input(), st.click_button, true);
        st.click_down = false;
      }
    }

    void reset_locked() {
      release_click_locked();
      st.c[0] = contact_t {};
      st.c[1] = contact_t {};
      st.fingers_seen = 0;
      st.gesture_clicked = false;
      st.acc_x = st.acc_y = 0.f;
      st.scroll_acc = 0;
      st.velocity_ema = 0.0;
      st.have_move_time = false;
      st.ptr_used[0] = st.ptr_used[1] = false;
    }

    /* Pointer motion: a faithful port of libinput's touchpad acceleration
     * (filter-touchpad.c touchpad_accel_profile_linear), which is what makes
     * this same pad feel precise under Linux with default settings:
     *  - deceleration below a nominal 7 mm/s down to factor 0.3 (subpixel
     *    precision for small corrections),
     *  - a flat 0.9 plateau up to 130 mm/s (predictable 1:1-feel — the part
     *    an "accelerate everything" curve gets wrong),
     *  - a soft curve above, capped at 4x the threshold,
     *  - everything times TP_MAGIC_SLOWDOWN (0.2968).
     * Parameters as libinput resolves them for THIS pad: hid-playstation sets
     * no resolution and no hwdb/quirk entry exists, so libinput assumes a
     * 69x50 mm pad => res_x = 1920/69 = 27, res_y = 1080/50 = 21 units/mm
     * (integer division, as libinput does it); y is rescaled to the x axis
     * (27/21). The accel filter runs at DEFAULT_MOUSE_DPI = 1000, where
     * normalize_for_dpi() is the identity, and "mm/s" in the profile is the
     * nominal units-as-1000dpi-counts speed, not physical mm. Bluetooth pads
     * get libinput's delta smoothener: an event interval below 50 ms is
     * REPLACED by 10 ms for the velocity estimate — at a 250 Hz report rate
     * that substitution dominates the feel, so it is kept verbatim.
     * Velocity is EMA-smoothed as a stand-in for libinput's tracker set plus
     * Simpson's-rule factor integration. Fractional remainders are carried so
     * slow movement is not truncated away. */
    void move_pointer_locked(int dx, int dy) {
      constexpr double TP_MAGIC_SLOWDOWN = 0.2968;
      constexpr double BASELINE = 0.9;
      constexpr double THRESHOLD_MM_S = 130.0;   // nominal, see above
      constexpr double DECEL_LIMIT_MM_S = 7.0;
      constexpr double SMOOTH_THRESHOLD_US = 50000.0;
      constexpr double SMOOTH_VALUE_US = 10000.0;
      constexpr double XY_SCALE = 27.0 / 21.0;
      constexpr double EMA_ALPHA = 0.3;

      auto now = std::chrono::steady_clock::now();
      double dys = (double) dy * XY_SCALE;

      double dt_us = SMOOTH_VALUE_US;
      if (st.have_move_time) {
        dt_us = (double) std::chrono::duration_cast<std::chrono::microseconds>(
                  now - st.last_move_time)
                  .count() +
                1.0;
        if (dt_us < SMOOTH_THRESHOLD_US) {
          dt_us = SMOOTH_VALUE_US;
        }
      }
      st.last_move_time = now;
      st.have_move_time = true;

      double v = std::hypot((double) dx, dys) / dt_us;  // units/µs
      st.velocity_ema += EMA_ALPHA * (v - st.velocity_ema);

      // units/µs -> nominal mm/s at 1000 dpi: *1e6 (per s) * 25.4/1000
      double speed_in = st.velocity_ema * 25400.0;
      double factor;
      if (speed_in < DECEL_LIMIT_MM_S) {
        factor = std::min(BASELINE, 0.1 * speed_in + 0.3);
      } else if (speed_in < THRESHOLD_MM_S) {
        factor = BASELINE;
      } else {
        double capped = std::min(speed_in, THRESHOLD_MM_S * 4.0);
        factor = 0.0025 * (capped / THRESHOLD_MM_S) * (capped - THRESHOLD_MM_S) + BASELINE;
      }
      factor *= (double) speed_percent() / 100.0;
      factor *= TP_MAGIC_SLOWDOWN;

      st.acc_x += (float) ((double) dx * factor);
      st.acc_y += (float) (dys * factor);
      int out_x = (int) st.acc_x;
      int out_y = (int) st.acc_y;
      if (out_x != 0 || out_y != 0) {
        st.acc_x -= (float) out_x;
        st.acc_y -= (float) out_y;
        platf::move_mouse(input::raw_platf_input(), out_x, out_y);
      }
    }

    void scroll_locked(int dy) {
      st.scroll_acc += dy;
      while (std::abs(st.scroll_acc) >= SCROLL_UNITS_PER_NOTCH) {
        int dir = st.scroll_acc > 0 ? 1 : -1;
        st.scroll_acc -= dir * SCROLL_UNITS_PER_NOTCH;
        // Finger travel down (dy > 0) scrolls the view down (negative wheel),
        // matching the Windows precision-touchpad default. natural_scroll
        // inverts (content follows the fingers).
        int notch = natural_scroll() ? dir * 120 : dir * -120;
        platf::scroll(input::raw_platf_input(), notch);
      }
    }

    // One contact slot transitioned or moved; run the shared gesture logic.
    // click = physical touchpad button (HID path only; false on the SDL path).
    void process_locked(const contact_t prev[2], bool click) {
      int fingers = (st.c[0].down ? 1 : 0) + (st.c[1].down ? 1 : 0);
      st.fingers_seen = std::max(st.fingers_seen, fingers);

      // Primary contact: slot 0 when down, else slot 1.
      int pi = st.c[0].down ? 0 : 1;
      const contact_t &p = st.c[pi];
      const contact_t &pp = prev[pi];

      // Movement, only for an ongoing contact of the same instance.
      if (p.down && pp.down && p.id == pp.id) {
        int dx = p.x - pp.x;
        int dy = p.y - pp.y;
        if (dx != 0 || dy != 0) {
          if (fingers >= 2) {
            scroll_locked(dy);
          } else if (!st.click_down || st.click_button == MOUSE_LEFT) {
            // One finger: pointer motion. Also while the pad is physically
            // clicked with one finger — that is a drag.
            move_pointer_locked(dx, dy);
          }
        }
      }

      // Travel bookkeeping for tap detection.
      for (int i = 0; i < 2; i++) {
        if (st.c[i].down && !st.c[i].moved) {
          int tx = st.c[i].x - st.c[i].start_x;
          int ty = st.c[i].y - st.c[i].start_y;
          if (tx * tx + ty * ty > TAP_MAX_TRAVEL * TAP_MAX_TRAVEL) {
            st.c[i].moved = true;
          }
        }
      }

      // Physical click (the pad is one big button): edge-triggered.
      if (click && !st.click_down) {
        st.click_button = st.fingers_seen >= 2 ? MOUSE_RIGHT : MOUSE_LEFT;
        platf::button_mouse(input::raw_platf_input(), st.click_button, false);
        st.click_down = true;
        st.gesture_clicked = true;
      } else if (!click && st.click_down) {
        release_click_locked();
      }

      // Tap-to-click: fires when the last finger lifts after a short, still,
      // unclicked gesture.
      if (fingers == 0 && (pp.down || prev[1 - pi].down)) {
        const contact_t &last = pp.down ? pp : prev[1 - pi];
        auto held = std::chrono::steady_clock::now() - last.t_down;
        bool any_moved = prev[0].moved || prev[1].moved || st.c[0].moved || st.c[1].moved;
        if (!st.gesture_clicked && !any_moved && held <= TAP_MAX_DURATION) {
          int button = st.fingers_seen >= 2 ? MOUSE_RIGHT : MOUSE_LEFT;
          platf::button_mouse(input::raw_platf_input(), button, false);
          platf::button_mouse(input::raw_platf_input(), button, true);
        }
        st.fingers_seen = 0;
        st.gesture_clicked = false;
        st.scroll_acc = 0;
        st.acc_x = st.acc_y = 0.f;
        st.ptr_used[0] = st.ptr_used[1] = false;
      }
    }

    void set_contact_locked(int slot, bool down, uint8_t id, int x, int y) {
      contact_t &c = st.c[slot];
      bool fresh = down && (!c.down || c.id != id);
      if (fresh) {
        // libinput resets its velocity trackers when a touch begins.
        st.velocity_ema = 0.0;
        st.have_move_time = false;
      }
      c.down = down;
      c.id = id;
      if (fresh) {
        c.start_x = x;
        c.start_y = y;
        c.t_down = std::chrono::steady_clock::now();
        c.moved = false;
      }
      if (down) {
        c.x = x;
        c.y = y;
      }
    }

  }  // namespace

  bool active() {
    int64_t now = now_ms();
    if (gate_cache.load(std::memory_order_relaxed) < 0 ||
        now - gate_stamp_ms.load(std::memory_order_relaxed) > 500) {
      bool on = evaluate_gate();
      int prev = gate_cache.exchange(on ? 1 : 0, std::memory_order_relaxed);
      gate_stamp_ms.store(now, std::memory_order_relaxed);
      if (prev >= 0 && prev != (on ? 1 : 0)) {
        BOOST_LOG(info) << "tpmouse: touchpad-mouse " << (on ? "engaged (desktop)" : "released (game running)");
        if (!on) {
          std::lock_guard lk(st.mtx);
          reset_locked();
        }
      }
    }
    return gate_cache.load(std::memory_order_relaxed) == 1;
  }

  void feed_usb_report(const uint8_t *usb, size_t len) {
    // USB 0x01 layout: payload p = usb + 1; touch points at p[32..35] / p[36..39]
    // (bit7 of the first byte = finger up, low 7 bits = contact counter);
    // touchpad click = p[9] & 0x02. Offsets per Linux hid-playstation, the
    // same map the client's neutralizer uses.
    if (!usb || len < 41 || usb[0] != 0x01) {
      return;
    }
    if (!active()) {
      return;
    }
    const uint8_t *p = usb + 1;
    std::lock_guard lk(st.mtx);
    contact_t prev[2] = {st.c[0], st.c[1]};
    for (int i = 0; i < 2; i++) {
      const uint8_t *t = p + 32 + i * 4;
      bool down = (t[0] & 0x80) == 0;
      uint8_t id = t[0] & 0x7f;
      int x = t[1] | ((t[2] & 0x0f) << 8);
      int y = (t[2] >> 4) | (t[3] << 4);
      x = std::clamp(x, 0, PAD_W - 1);
      y = std::clamp(y, 0, PAD_H - 1);
      set_contact_locked(i, down, id, x, y);
    }
    bool click = (p[9] & 0x02) != 0;
    process_locked(prev, click);
  }

  bool feed_touch_event(uint8_t event_type, uint32_t pointer_id, float x, float y) {
    if (!active()) {
      return false;
    }
    std::lock_guard lk(st.mtx);
    contact_t prev[2] = {st.c[0], st.c[1]};

    if (event_type == LI_TOUCH_EVENT_CANCEL_ALL) {
      set_contact_locked(0, false, st.c[0].id, st.c[0].x, st.c[0].y);
      set_contact_locked(1, false, st.c[1].id, st.c[1].x, st.c[1].y);
      process_locked(prev, false);
      return true;
    }

    // Resolve the pointer to a slot; allocate on DOWN.
    int slot = -1;
    for (int i = 0; i < 2; i++) {
      if (st.ptr_used[i] && st.ptr_id[i] == pointer_id) {
        slot = i;
        break;
      }
    }
    if (slot < 0) {
      if (event_type != LI_TOUCH_EVENT_DOWN) {
        return true;  // stale MOVE/UP for a pointer we never tracked
      }
      for (int i = 0; i < 2; i++) {
        if (!st.ptr_used[i]) {
          slot = i;
          st.ptr_used[i] = true;
          st.ptr_id[i] = pointer_id;
          break;
        }
      }
      if (slot < 0) {
        return true;  // third finger; the DS5 pad tracks two
      }
    }

    int px = std::clamp((int) (x * (PAD_W - 1)), 0, PAD_W - 1);
    int py = std::clamp((int) (y * (PAD_H - 1)), 0, PAD_H - 1);
    switch (event_type) {
      case LI_TOUCH_EVENT_DOWN:
        set_contact_locked(slot, true, (uint8_t) (pointer_id & 0x7f), px, py);
        break;
      case LI_TOUCH_EVENT_MOVE:
        set_contact_locked(slot, st.c[slot].down, st.c[slot].id, px, py);
        break;
      case LI_TOUCH_EVENT_UP:
      case LI_TOUCH_EVENT_CANCEL:
        set_contact_locked(slot, false, st.c[slot].id, st.c[slot].x, st.c[slot].y);
        st.ptr_used[slot] = false;
        break;
      default:
        return true;  // HOVER etc. — not meaningful for a mouse
    }
    process_locked(prev, false);
    return true;
  }

}  // namespace tpmouse
