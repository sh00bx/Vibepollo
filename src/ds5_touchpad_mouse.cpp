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
      // Settling: contacts were disturbed by something that is not a movement,
      // so motion and scroll are held until a finger travels deliberately again.
      bool settling {false};
      int settle_x[2] {0, 0}, settle_y[2] {0, 0};
      // Pointer velocity tracking (see move_pointer_locked)
      std::chrono::steady_clock::time_point last_move_time {};
      bool have_move_time {false};
      double velocity_ema {0.0};  // device units per µs, smoothed
      // The pad's own free-running clock, when the report carries one. Raw
      // ticks; the kernel divides by 3 for microseconds.
      bool dev_clock_valid {false};
      uint32_t dev_now_raw {0};
      uint32_t dev_prev_raw {0};
      bool have_dev_prev {false};
      // SDL path: map moonlight pointerId -> slot
      uint32_t ptr_id[2] {0, 0};
      bool ptr_used[2] {false, false};
      // The feeder currently driving the gesture state (0 = none). Everything
      // here is per-gesture, so interleaved reports from a second pad would
      // corrupt it -- the non-owner is ignored instead.
      uintptr_t owner {0};
    };

    state_t st;

    // The gate is evaluated at report rate; cache the verdict briefly so the
    // config mutex and process lock are not taken 250 times a second.
    std::atomic<int> gate_cache {-1};
    std::atomic<int64_t> gate_stamp_ms {0};

    // Tuning values, mirrored out of the config at the same cadence: the
    // report path reads them per moving report while holding st.mtx, and must
    // not take the config mutex there either.
    std::atomic<int> speed_cache {100};
    std::atomic<bool> natural_scroll_cache {false};

    // Client preference (CTMB_MSG_TPMOUSE); -1 = none, fall back to config.
    std::atomic<int> client_mode {-1};

    int64_t now_ms() {
      return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
    }

    bool evaluate_gate() {
      // The TV client's setting wins over the host config: the TV UI is where
      // the user actually flips this, and it re-asserts every session.
      int cm = client_mode.load(std::memory_order_relaxed);
      if (cm == 0) {
        return false;
      }
      if (cm == 2) {
        return true;
      }
      if (cm < 0) {
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
      //    (cmd, Playnite target, detached) owns the touchpad. Read the app
      //    id passively: proc.running() drives the session state machine
      //    (it consumes deferred-launch flags and may run terminate(), with
      //    its blocking undo commands, in the caller's thread) and belongs
      //    to the control thread's poll, not to a gate evaluated on the
      //    bridge and stream-input threads.
      if (proc::proc.current_app_id() <= 0) {
        // No app at all: no active stream is feeding us anyway; allow, so the
        // brief window during session start behaves like the desktop it shows.
        return true;
      }
      return proc::proc.running_app_launches_nothing();
    }

    int speed_percent() {
      return speed_cache.load(std::memory_order_relaxed);
    }

    bool natural_scroll() {
      return natural_scroll_cache.load(std::memory_order_relaxed);
    }

    void release_click_locked() {
      if (st.click_down) {
        platf::button_mouse(input::raw_platf_input(), st.click_button, true);
        st.click_down = false;
      }
    }

    /* A finger never lands, lifts or presses cleanly -- it rolls, and at any
     * usable pointer speed that roll is tens of pixels. So every disturbance
     * that is not itself a movement parks motion here instead of emitting it:
     * a contact appearing or disappearing, and both edges of the physical
     * click (the contacts rock as the pad goes down, and rock back on
     * release). Settling ends when some finger travels past the same slop that
     * already tells a tap from a drag, and whatever gesture suits the fingers
     * still down then resumes on its own. Movement made while settling is
     * dropped, never replayed -- the pointer must not jump to catch up on
     * motion the user could not see. Buttons are deliberately not gated by
     * this; a click still fires the moment the pad reports it. */
    void begin_settle_locked() {
      st.settling = true;
      for (int i = 0; i < 2; i++) {
        st.settle_x[i] = st.c[i].x;
        st.settle_y[i] = st.c[i].y;
      }
      // Nothing accumulated so far may survive into the resumed gesture.
      st.scroll_acc = 0;
      st.acc_x = st.acc_y = 0.f;
      st.velocity_ema = 0.0;
      st.have_move_time = false;
      st.have_dev_prev = false;
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
      st.have_dev_prev = false;
      st.settling = false;
      st.ptr_used[0] = st.ptr_used[1] = false;
      st.owner = 0;
    }

    // One source owns the gesture state at a time. A different source takes
    // over only when it actually touches (engaged) and the current owner is
    // fully idle -- its resting reports must not tear down an ongoing gesture.
    bool acquire_source_locked(uintptr_t source, bool engaged) {
      if (st.owner == source) {
        return true;
      }
      if (st.owner != 0 && (st.c[0].down || st.c[1].down || st.click_down)) {
        return false;
      }
      if (!engaged) {
        return false;
      }
      reset_locked();
      st.owner = source;
      return true;
    }

    /* Pointer motion: libinput's touchpad acceleration (filter-touchpad.c
     * touchpad_accel_profile_linear), with two deviations that a measurement on
     * the real pad forced -- both documented at the constants below:
     *  - deceleration below DECEL_LIMIT_MM_S down to factor 0.3 (subpixel
     *    precision for small corrections; see DEVIATION 2 for the limit),
     *  - a flat 0.9 plateau up to 130 mm/s (predictable 1:1-feel — the part
     *    an "accelerate everything" curve gets wrong),
     *  - a soft curve above, capped at 4x the threshold,
     *  - everything times TP_MAGIC_SLOWDOWN (0.2968).
     * Parameters as libinput resolves them for THIS pad: hid-playstation sets
     * no resolution and no hwdb/quirk entry exists, so libinput assumes a
     * 69x50 mm pad => res_x = 1920/69 = 27, res_y = 1080/50 = 21 units/mm
     * (integer division, as libinput does it); y is rescaled to the x axis
     * (27/21). The accel filter runs at DEFAULT_MOUSE_DPI = 1000, where
     * normalize_for_dpi() is the identity — but libinput ALSO scales touchpad
     * deltas by (1000/25.4)/res_x ≈ 1.458 (tp_normalize_delta) before the
     * filter and on its output, which this port omits: velocities here are
     * libinput's divided by 1.458, and "mm/s" below is that raw-pad-unit
     * scale, not physical mm. Divide libinput constants by 1.458 before
     * comparing; the thresholds below are tuned in THIS unit on measured
     * data, so they need no conversion. Bluetooth pads
     * get libinput's delta smoothener: an event interval below 50 ms is
     * REPLACED by 10 ms for the velocity estimate — at a 250 Hz report rate
     * that substitution dominates the feel, so it is kept verbatim (SDL path
     * only; see DEVIATION 1).
     * Velocity is EMA-smoothed as a stand-in for libinput's tracker set plus
     * Simpson's-rule factor integration; the blend is time-weighted so its
     * time constant does not ride the pad's 40x cadence spread, and so a
     * pause (a large honest dt with a near-zero instantaneous speed) drains
     * the estimate almost completely — the first careful correction after a
     * flick must decelerate, not inherit the flick's speed. Fractional
     * remainders are carried so slow movement is not truncated away.
     *
     * DEVIATION 1 -- the interval comes from the pad, not from arrival.
     * libinput replaces any interval below 50 ms with a flat 10 ms because a
     * Bluetooth pad's delivery is bursty and the arrival gaps say nothing about
     * the device. Measured here (25 s of real use, 6261 reports read straight
     * off the TV's hidraw node): nothing was lost on the way, and the arrival
     * spacing tracks the pad's own emission clock almost exactly -- so there is
     * no burst artifact to compensate, and the substitution only destroys
     * information. The pad emits irregularly on its own, in multiples of about
     * 502 us: real intervals ran median 4015 us, p95 20575, max 42155. Pinning
     * that 40x spread to a constant turns "speed" into "distance per report",
     * and the curve then sat in its flat middle band for 89% of all moving
     * reports -- an acceleration profile that never accelerated and a
     * deceleration that almost never decelerated. The DS5 report carries a
     * free-running clock (sensor_timestamp), so use it; it is immune to
     * whatever the network does downstream. Reports without one (the SDL touch
     * path) keep libinput's substitution.
     * The clock reference advances only on reports that actually moved: the
     * coordinates are absolute, so a moving report's delta already integrates
     * everything since the last step, and pairing it with the full inter-step
     * interval is what makes the speed true. A per-report reference made
     * quantized slow motion (a 1-unit step every ~15-20 ms, reported every
     * ~4 ms) read 5-40x too fast -- the deceleration floor was unreachable a
     * second time -- and made pauses invisible to the estimator. This is also
     * the kernel's semantic: input events only fire on change.
     *
     * DEVIATION 2 -- the deceleration threshold is this pad's, not libinput's.
     * libinput's 7 mm/s assumes a pad that reports its resolution; this one
     * does not (hid-playstation never calls input_abs_set_res for the touch
     * device), so our "mm/s" is a stack of assumptions rather than a physical
     * speed. On the measured distribution 7 sits below the 5th percentile --
     * unreachable, which is why careful aiming got no help. The threshold is
     * placed at the 20th percentile of real use instead. The upper bound is
     * left at libinput's 130, because the same measurement puts it at the 75th
     * percentile, which is where it belongs. */
    void move_pointer_locked(int dx, int dy) {
      constexpr double TP_MAGIC_SLOWDOWN = 0.2968;
      constexpr double BASELINE = 0.9;
      constexpr double THRESHOLD_MM_S = 130.0;   // nominal, ~p75 of measured use
      constexpr double DECEL_LIMIT_MM_S = 30.0;  // ~p20 of measured use (libinput: 7)
      constexpr double DECEL_FLOOR = 0.3;        // gain at a standstill
      // Ramp from DECEL_FLOOR at rest to BASELINE exactly at the limit, so the
      // deceleration meets the plateau without a step wherever the limit sits.
      constexpr double DECEL_SLOPE = (BASELINE - DECEL_FLOOR) / DECEL_LIMIT_MM_S;
      constexpr double SMOOTH_THRESHOLD_US = 50000.0;  // SDL path only
      constexpr double SMOOTH_VALUE_US = 10000.0;      // SDL path only
      // Sanity band for the pad clock, now spanning steps rather than
      // reports: faster than the pad can emit is a corrupt timestamp, and
      // beyond a second the "interval" is a wrap-ambiguous pause whose speed
      // the arrival clock estimates just as well.
      constexpr double DEV_DT_MIN_US = 400.0;      // faster than the pad can report
      constexpr double DEV_DT_MAX_US = 1000000.0;  // longer is a pause, not a step
      constexpr double XY_SCALE = 27.0 / 21.0;
      // Time constant of the velocity EMA; 1-exp(-4015/11000) = 0.30, the
      // per-event alpha the curve was tuned with at the pad's median cadence.
      constexpr double EMA_TAU_US = 11000.0;

      auto now = std::chrono::steady_clock::now();
      double dys = (double) dy * XY_SCALE;

      double dt_us = SMOOTH_VALUE_US;
      bool dev_dt_ok = false;
      if (st.dev_clock_valid && st.have_dev_prev) {
        // Unsigned arithmetic carries the 32-bit wrap (~24 min) on its own.
        uint32_t ticks = st.dev_now_raw - st.dev_prev_raw;
        double us = (double) ticks / 3.0;  // kernel: microseconds = raw / 3
        // Out of band means the clock is not telling us anything -- a pad that
        // never advances it would otherwise pin dt to the floor and make the
        // pointer race. Fall back to arrival time instead of clamping into
        // range: a wrong-but-plausible interval is worse than the old estimate.
        if (us >= DEV_DT_MIN_US && us <= DEV_DT_MAX_US) {
          dt_us = us;
          dev_dt_ok = true;
        }
      }
      if (!dev_dt_ok && st.have_move_time) {
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
      if (st.dev_clock_valid) {
        // Only motion advances the reference (see the header comment): the
        // next step's dt then spans the stationary reports in between.
        st.dev_prev_raw = st.dev_now_raw;
        st.have_dev_prev = true;
      }

      double v = std::hypot((double) dx, dys) / dt_us;  // units/µs
      double alpha = 1.0 - std::exp(-dt_us / EMA_TAU_US);
      st.velocity_ema += alpha * (v - st.velocity_ema);

      // units/µs -> nominal mm/s at 1000 dpi: *1e6 (per s) * 25.4/1000
      double speed_in = st.velocity_ema * 25400.0;
      double factor;
      if (speed_in < DECEL_LIMIT_MM_S) {
        factor = std::min(BASELINE, DECEL_SLOPE * speed_in + DECEL_FLOOR);
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
      int prev_fingers = (prev[0].down ? 1 : 0) + (prev[1].down ? 1 : 0);
      st.fingers_seen = std::max(st.fingers_seen, fingers);

      // st.click_down still holds the previous report's state here -- the edge
      // itself is acted on further down.
      bool contacts_changed = fingers != prev_fingers;
      for (int i = 0; i < 2 && !contacts_changed; i++) {
        // A lift and a fresh landing inside one report keeps the count but is
        // still a new finger.
        contacts_changed = st.c[i].down && prev[i].down && st.c[i].id != prev[i].id;
      }
      if (contacts_changed || click != st.click_down) {
        begin_settle_locked();
      } else if (st.settling) {
        for (int i = 0; i < 2; i++) {
          if (!st.c[i].down) {
            continue;
          }
          int tx = st.c[i].x - st.settle_x[i];
          int ty = st.c[i].y - st.settle_y[i];
          if (tx * tx + ty * ty > TAP_MAX_TRAVEL * TAP_MAX_TRAVEL) {
            st.settling = false;
            break;
          }
        }
      }

      // Primary contact: slot 0 when down, else slot 1.
      int pi = st.c[0].down ? 0 : 1;
      const contact_t &p = st.c[pi];
      const contact_t &pp = prev[pi];

      // Movement, only for an ongoing contact of the same instance, and only
      // once the contacts have settled.
      if (!st.settling && p.down && pp.down && p.id == pp.id) {
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
      {
        std::lock_guard lk(config::ds5b_mutex);
        speed_cache.store(config::ds5b.touchpad_mouse_speed, std::memory_order_relaxed);
        natural_scroll_cache.store(config::ds5b.touchpad_mouse_natural_scroll, std::memory_order_relaxed);
      }
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

  void set_client_mode(int mode) {
    int prev = client_mode.exchange(mode, std::memory_order_relaxed);
    if (prev != mode) {
      BOOST_LOG(info) << "tpmouse: client preference "
                      << (mode == 0 ? "off" : mode == 2 ? "always" : mode == 1 ? "auto" : "cleared");
      // Re-evaluate on the next feed instead of waiting out the cache. Expire
      // the stamp rather than the cached verdict: active()'s off-transition
      // reset needs the previous verdict (prev >= 0) to see the edge, and
      // wiping it would leave a held click stuck across a mode change.
      gate_stamp_ms.store(0, std::memory_order_relaxed);
    }
  }

  void reset() {
    std::lock_guard lk(st.mtx);
    reset_locked();
  }

  bool feed_usb_report(uintptr_t source, const uint8_t *usb, size_t len) {
    // USB 0x01 layout: payload p = usb + 1; touch points at p[32..35] / p[36..39]
    // (bit7 of the first byte = finger up, low 7 bits = contact counter);
    // touchpad click = p[9] & 0x02. Offsets per Linux hid-playstation, the
    // same map the client's neutralizer uses.
    if (!usb || len < 41 || usb[0] != 0x01) {
      return false;
    }
    if (!active()) {
      return false;
    }
    const uint8_t *p = usb + 1;
    const bool engaged = (p[32] & 0x80) == 0 || (p[36] & 0x80) == 0 || (p[9] & 0x02) != 0;
    // Free-running pad clock at common[27..30] (hid-playstation.c,
    // struct dualsense_input_report::sensor_timestamp). Little endian.
    const uint32_t dev_raw = (uint32_t) p[27] | ((uint32_t) p[28] << 8) |
                             ((uint32_t) p[29] << 16) | ((uint32_t) p[30] << 24);
    std::lock_guard lk(st.mtx);
    if (!acquire_source_locked(source, engaged)) {
      return false;
    }
    st.dev_clock_valid = true;
    st.dev_now_raw = dev_raw;
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
    // The clock reference is advanced inside move_pointer_locked, by motion
    // only -- a stationary report must lengthen the next step's interval,
    // not reset it.
    return true;
  }

  bool feed_touch_event(uintptr_t source, uint8_t event_type, uint32_t pointer_id, float x, float y) {
    if (!active()) {
      return false;
    }
    std::lock_guard lk(st.mtx);
    if (!acquire_source_locked(source, event_type == LI_TOUCH_EVENT_DOWN)) {
      // Another pad owns the gesture state; this one stays a pad touchpad.
      return false;
    }
    // Moonlight touch events carry no pad clock; this path keeps libinput's
    // arrival-time estimate.
    st.dev_clock_valid = false;
    st.have_dev_prev = false;
    contact_t prev[2] = {st.c[0], st.c[1]};

    if (event_type == LI_TOUCH_EVENT_CANCEL_ALL) {
      set_contact_locked(0, false, st.c[0].id, st.c[0].x, st.c[0].y);
      set_contact_locked(1, false, st.c[1].id, st.c[1].x, st.c[1].y);
      process_locked(prev, false);
      // Pass it on as well: the emulated pad may hold contacts from before a
      // gate flip, and cancel-everything means as much there as here.
      return false;
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
        // A pointer we never tracked: its DOWN went to the emulated pad
        // before the gate flipped on. Pass the rest of that gesture through
        // too -- the pad must see the terminating edge, or it holds the
        // contact (and its ViGEm pointer slot) forever.
        return false;
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
        // Third finger; the DS5 pad tracks two. Let the whole extra gesture
        // stay a pad touch rather than consuming its DOWN and orphaning the
        // UP.
        return false;
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
