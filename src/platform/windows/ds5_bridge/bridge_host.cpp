/**
 * @file src/platform/windows/ds5_bridge/bridge_host.cpp
 * @brief Host-side CTM-Bridge server + per-controller data-plane session.
 *
 * License-clean interop implementation (see bridge_host.h). No CTM source used.
 */
// clang-format off
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
// clang-format on

#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <deque>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

#include <boost/asio/ip/address.hpp>
#include <enet/enet.h>

#include "src/ds5_touchpad_mouse.h"
#include "src/logging.h"
#include "src/platform/common.h"
#include "src/platform/windows/ds5_bridge/bridge_host.h"
#include "src/platform/windows/ds5_bridge/ctmb_protocol.h"
#include "src/platform/windows/ds5_bridge/ds5_haptics.h"
#include "src/platform/windows/ds5_bridge/ds5_reports.h"
#include "src/platform/windows/ds5_bridge/vhci_attach.h"

using namespace std::literals;

namespace platf::ds5_bridge {

  namespace {
    uint64_t now_us() {
      using namespace std::chrono;
      return (uint64_t) duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
    }

    // Bridge links that completed a HELLO handshake, across all sessions. The
    // touchpad-mouse client preference is client-scoped, not per-pad: it may
    // only fall back to the host config once the LAST live link is gone (each
    // link re-asserts it right after its handshake).
    std::atomic<int> g_hello_links {0};

    void ensure_enet() {
      static std::once_flag once;
      std::call_once(once, [] {
        if (enet_initialize() != 0) {
          BOOST_LOG(error) << "ds5-bridge: enet_initialize failed"sv;
        }
      });
    }
  }  // namespace

  // ===========================================================================
  //  bridge_session — one plugged controller: data-plane transport + usbip slot
  // ===========================================================================
  class bridge_session {
  public:
    bridge_session(usbip_ds5_device *usbip, std::string ctmb_busid, int dport, bool haptics,
                   bool audio_batched, int audio_cushion,
                   const std::atomic<uint32_t> *lightbar_rgb):
        usbip_(usbip), ctmb_busid_(std::move(ctmb_busid)), dport_(dport),
        audio_batched_(audio_batched), audio_cushion_(audio_cushion),
        lightbar_rgb_(lightbar_rgb) { haptics_want_.store(haptics); }

    /// Control thread, on every BRIDGE_START for this session (incl. adopts).
    /// Takes effect at the next HELLO (see on_hello).
    void set_haptics(bool on) { haptics_want_.store(on, std::memory_order_relaxed); }

    ~bridge_session() { stop(); }

    void start() {
      stop_.store(false);
      thread_ = std::thread(&bridge_session::run, this);
    }

    void stop() {
      if (stop_.exchange(true)) {
        if (thread_.joinable()) thread_.join();
        return;
      }
      if (thread_.joinable()) thread_.join();
    }

    /// The data port this session owns — the controller's stable identity across
    /// reconnects. Set once at construction, never mutated, so read lock-free.
    int dport() const { return dport_; }

    /// Re-label a live session to the busid the TV issued on this reconnect, so a
    /// later BRIDGE_STOP resolves. Called from the control thread; guarded because
    /// the session thread reads the label for logging (see label()).
    void relabel(const std::string &busid) {
      std::lock_guard<std::mutex> lk(label_mtx_);
      ctmb_busid_ = busid;
    }

    /// Current busid label (control thread resolves BRIDGE_STOP by scanning
    /// these; also used in log lines on the session thread).
    std::string label() {
      std::lock_guard<std::mutex> lk(label_mtx_);
      return ctmb_busid_;
    }

  private:

    // ---- outbound message send (transport-agnostic) -----------------------
    void send_msg(uint16_t type, uint32_t flags, uint32_t request_id,
                  const uint8_t *payload, uint32_t len) {
      std::vector<uint8_t> msg(sizeof(ctmb_header_t) + len);
      auto *h = reinterpret_cast<ctmb_header_t *>(msg.data());
      h->magic = CTMB_MAGIC;
      h->version = CTMB_VERSION;
      h->type = type;
      h->flags = flags;
      h->sequence = seq_++;
      h->timestamp_us = now_us();
      h->request_id = request_id;
      h->payload_len = len;
      if (len) std::memcpy(msg.data() + sizeof(ctmb_header_t), payload, len);

      const bool paced = (flags & CTMB_FLAG_PACED) != 0;
      if (transport_ == ENET && peer_) {
        // Paced audio rides UNRELIABLE on its own channel: with a continuous
        // 100/s stream on the reliable channel there is ALWAYS unacked data in
        // flight, so any >=5 s WiFi stall is a guaranteed peer timeout, and the
        // retransmit queue head-of-line-blocks input/control traffic (the CTM
        // reference splits exactly this way; the TV ingest is channel-agnostic
        // and tolerates gaps). A lost frame is a ~10 ms blip.
        uint32_t pktflags = paced ? 0 : ENET_PACKET_FLAG_RELIABLE;
        uint8_t channel = (paced && peer_->channelCount > 1) ? 1 : 0;
        ENetPacket *pkt = enet_packet_create(msg.data(), msg.size(), pktflags);
        if (pkt && enet_peer_send(peer_, channel, pkt) == 0) {
          enet_host_flush(host_);
        } else if (pkt) {
          enet_packet_destroy(pkt);
        }
      } else if (transport_ == TCP && tcp_client_ != INVALID_SOCKET) {
        int off = 0, total = (int) msg.size();
        while (off < total) {
          int n = ::send(tcp_client_, (const char *) msg.data() + off, total - off, 0);
          if (n > 0) { off += n; continue; }
          int e = (n == 0) ? WSAECONNRESET : WSAGetLastError();
          if (e == WSAEWOULDBLOCK || e == WSAENOBUFS) {
            // Full send buffer (WiFi stall) on the nonblocking socket is NOT a
            // dead link — the recv path detects real drops. An untouched paced
            // frame is disposable; anything partially sent (or non-paced) must
            // complete or the byte stream desyncs, so wait for writability.
            if (paced && off == 0) return;
            WSAPOLLFD p {};
            p.fd = tcp_client_;
            p.events = POLLOUT;
            if (WSAPoll(&p, 1, 1000) <= 0) { link_down_ = true; break; }
            continue;
          }
          link_down_ = true;
          break;
        }
      }
    }

    // ---- inbound message dispatch -----------------------------------------
    void on_message(const ctmb_header_t *h, const uint8_t *payload) {
      // Any non-HELLO inbound frame proves the client's handshake() consumed
      // HOST_CONFIG (it hard-fails and drops the link otherwise) — from here the
      // unreliable ch1 0x36 stream can no longer overtake a lost-and-
      // retransmitted HOST_CONFIG and kill the fresh session.
      if (h->type != CTMB_MSG_HELLO) {
        client_ready_.store(true, std::memory_order_relaxed);
      }
      switch (h->type) {
        case CTMB_MSG_HELLO:
          on_hello(h, payload);
          break;
        case CTMB_MSG_INPUT_REPORT:
          if (slot_) {
            uint8_t usb[INPUT_REPORT_LEN];
            if (bt_input_to_usb(payload, h->payload_len, usb)) {
              // Desktop touchpad-mouse tap. Cheap no-op while a game is
              // streamed. While the mouse consumes this pad's touch, lift the
              // contacts and the touchpad click off the copy the virtual pad
              // sees -- a host-side pad reader (Steam Input and friends) must
              // not receive the same finger the desktop pointer acts on.
              if (tpmouse::feed_usb_report((uintptr_t) this, usb, sizeof(usb))) {
                usb[1 + 32] |= 0x80;  // touch point 0: finger up
                usb[1 + 36] |= 0x80;  // touch point 1: finger up
                usb[1 + 9] &= ~0x02;  // touchpad click
              }
              usbip_->set_input(slot_, usb);
            }
          }
          break;
        case CTMB_MSG_FEATURE_REPORT: {
          uint8_t rid = (uint8_t) h->request_id;
          if ((h->flags & CTMB_FLAG_OK) && h->payload_len > 0) {
            std::lock_guard<std::mutex> lk(feat_mtx_);
            auto &slot = feat_cache_[rid];
            slot.fill(0);
            std::memcpy(slot.data(), payload, std::min<uint32_t>(h->payload_len, 64));
            feat_have_[rid] = true;
          }
          break;
        }
        case CTMB_MSG_PACE_FEEDBACK:
          on_pace_feedback(h, payload);
          break;
        case CTMB_MSG_TPMOUSE:
          if (h->payload_len >= sizeof(ctmb_tpmouse_t)) {
            ctmb_tpmouse_t tp;
            std::memcpy(&tp, payload, sizeof(tp));
            if (tp.mode <= 2) {
              tpmouse::set_client_mode((int) tp.mode);
            } else {
              // The client's word overrides the host config, so a value we
              // do not understand must change nothing -- coercing it to a
              // real mode would enable the feature against an explicit
              // host-side "off".
              BOOST_LOG(warning) << "ds5-bridge: ignoring unknown tpmouse mode "sv << (int) tp.mode;
            }
          }
          break;
        default:
          break;  // LOG/ERROR/ENUM/etc. — not used by the DS5 path
      }
    }

    void on_hello(const ctmb_header_t *h, const uint8_t *payload) {
      if (h->payload_len < sizeof(ctmb_device_caps_t)) return;
      ctmb_device_caps_t caps;
      std::memcpy(&caps, payload, sizeof(caps));
      caps.serial[sizeof(caps.serial) - 1] = '\0';
      std::string serial = caps.serial[0] ? caps.serial : label();
      const bool client_0x39 = (caps.flags & CTMB_DEVCAP_DS5_AUDIO_0X39) != 0;

      // First HELLO on this session: register the virtual DualSense (its output +
      // feature callbacks run on a usbip server thread) and attach the vhci once.
      // A HELLO on a reconnect keeps the same slot + device (the game never sees a
      // hot-unplug); only the handshake below is replayed.
      if (!slot_) {
        slot_ = usbip_->add_slot(
          serial,
          [this](const uint8_t *eff) { on_game_output(eff); },
          [this](uint8_t rid, uint8_t *out) -> int {
            std::lock_guard<std::mutex> lk(feat_mtx_);
            if (feat_have_[rid]) {
              std::memcpy(out, feat_cache_[rid].data(), 64);
              return 64;
            }
            return 0;
          });
        // Phase 2: capture the game's iso-OUT PCM and drive a paced 0x36 haptic
        // stream. The builder is cheap (DSP tables + one Opus encoder, no device
        // handles), so it and the pacer are set up unconditionally and gated at
        // runtime by haptics_on_ — that lets a config flip land on the next HELLO
        // of an adopted session without racing the usbip iso callback. Wire the
        // hook before the attach below (which is when the game starts writing).
        hap_ = std::make_unique<ds5_haptic_builder>();
        // Report form is fixed for the life of the builder: the pad tracks an
        // audio packet counter whose step encodes how many frames a report
        // carries, so flipping mid-stream would hand it a discontinuity.
        // Batched 0x39 is caps-gated: a pre-0x39 client (CTMB_VERSION never
        // bumped) would silently drop the reports, so without the caps bit the
        // session falls back to unbatched 0x36 regardless of config.
        const bool batched = audio_batched_ && client_0x39;
        hap_->set_batched(batched);
        hap_->set_cushion_frames(audio_cushion_);   // after set_batched: the floor depends on it
        if (batched) {
          BOOST_LOG(info) << "ds5-bridge: audio downlink = batched 0x39 ("sv
                          << DS5_0X39_LEN << " B, two frames + two coil blocks per report)"sv;
        } else if (audio_batched_) {
          BOOST_LOG(info) << "ds5-bridge: audio downlink = unbatched 0x36 (config wants batched, but client caps 0x"sv
                          << std::hex << caps.flags << std::dec << " lack CTMB_DEVCAP_DS5_AUDIO_0X39)"sv;
        } else {
          BOOST_LOG(info) << "ds5-bridge: audio downlink = unbatched 0x36 (per config)"sv;
        }
        BOOST_LOG(info) << "ds5-bridge: speaker cushion = "sv << hap_->cushion_frames()
                        << " frames (~"sv << (hap_->cushion_frames() * 1067 / 100) << " ms), requested "sv
                        << audio_cushion_ << ", usable margin "sv
                        << ((hap_->cushion_frames() - hap_->spk_frames_per_tick()) * 1067 / 100) << " ms"sv;
        slot_->on_iso_out = [this](const uint8_t *pcm, size_t len) {
          if (hap_ && haptics_on_.load(std::memory_order_relaxed)) hap_->feed_pcm(pcm, len);
        };
        pacer_stop_.store(false);
        pacer_thread_ = std::thread(&bridge_session::pacer_run, this);
        // Attach off the session thread: `usbip attach` spawns a CLI + two port
        // snapshots (a couple of seconds); the loop must keep servicing ENet in
        // the meantime or the client link would go unpinged.
        std::string busid = slot_->busid;
        attach_thread_ = std::thread([this, busid] { vhci_port_.store(vhci_attach(busid)); });
        BOOST_LOG(info) << "ds5-bridge: session "sv << label() << " up (serial="sv
                        << serial << ", usbip busid="sv << busid << ")"sv;
      }

      // Re-HELLO of an adopted session: the builder's report form is fixed for
      // its lifetime (the pacer also caches report_len/pace at thread start), so
      // a client that re-HELLOs WITHOUT the 0x39 caps bit into a batched session
      // cannot be downgraded live — it will silently drop every 0x39 report.
      // Make that state loudly diagnosable instead of dark: audio stays dead
      // until the session is torn down (app/stream restart), not forever.
      if (slot_ && hap_ && hap_->batched() && !client_0x39) {
        BOOST_LOG(warning) << "ds5-bridge: re-HELLO from a client without CTMB_DEVCAP_DS5_AUDIO_0X39 (caps 0x"sv
                           << std::hex << caps.flags << std::dec
                           << ") into a batched-0x39 session — speaker/haptics will be SILENT until this session is recreated"sv;
      }

      // Latch the configured haptics state for this connect (see haptics_want_).
      {
        const bool want = haptics_want_.load(std::memory_order_relaxed);
        if (want != haptics_on_.load(std::memory_order_relaxed)) {
          haptics_on_.store(want, std::memory_order_relaxed);
          BOOST_LOG(info) << "ds5-bridge: session "sv << label() << " haptics "sv
                          << (want ? "on"sv : "off"sv) << " (applied on HELLO)"sv;
        }
      }

      // The fresh handshake starts unconfirmed: hold the unreliable 0x36 stream
      // until the first post-HELLO inbound frame proves the client got
      // HOST_CONFIG (see client_ready_).
      client_ready_.store(false, std::memory_order_relaxed);
      // A re-HELLO can mean a re-paired BT link whose firmware re-latched the
      // lightbar-setup gate — restart the re-assert window (see lb_in_release_window).
      lb_connect_ms_.store(now_ms(), std::memory_order_relaxed);

      // Always (re)answer the handshake: the client waits for HOST_CONFIG on every
      // (re)connect (needs_host_config), so this must be sent each HELLO.
      ctmb_host_config_t cfg {};
      // TV drain pace for its paced queue: advertise the TRUE emission cadence
      // (10667 us unbatched 0x36, 21334 us batched 0x39). A pre-08-03 client
      // clamps anything above 8 ms down to 8 ms; a newer client paces the
      // daemon-free hidraw drain at 3/4 of this advert — for the legacy 10667
      // that is exactly the old 8000 us, so both directions stay compatible
      // without a lockstep deploy. hap_ exists by the time any HELLO is
      // answered (created above on session setup); the fallback only covers a
      // re-HELLO racing teardown.
      cfg.bt_pace_us = hap_ ? (uint32_t) hap_->pace_base_us() : 10667;
      // Advertise the rate-servo capability: a client that sees this forwards
      // the daemon's inject-queue telemetry as CTMB_MSG_PACE_FEEDBACK. Old
      // clients ignore reserved bytes and simply never send it.
      cfg.reserved[0] = CTMB_HOSTCFG_PACE_FEEDBACK;
      cfg.input_report_len = caps.input_report_len;
      cfg.output_report_len = BT_OUTPUT_LEN;
      cfg.feature_report_len = caps.feature_report_len;
      // Advertise BOTH audio report forms as paced. Which one this session emits
      // is fixed at session creation (audio_batched_, caps-gated above), but advertising both is
      // free (the list holds 16) and keeps the advertisement correct no matter
      // when the HELLO arrives relative to the builder's construction.
      cfg.paced_report_count = 2;
      cfg.paced_report_ids[0] = 0x36;
      cfg.paced_report_ids[1] = 0x39;
      send_msg(CTMB_MSG_HOST_CONFIG, CTMB_FLAG_OK, 0,
               reinterpret_cast<const uint8_t *>(&cfg), sizeof(cfg));

      // Prefetch calibration / MAC / firmware so the game's EP0 GET_REPORT is
      // answered from the real controller. This read also unlocks the pad's
      // extended (0x31) BT input mode, so refresh it on every (re)connect.
      for (uint8_t rid : {0x05, 0x09, 0x20}) {
        uint8_t req[64] = {0};
        req[0] = rid;
        send_msg(CTMB_MSG_FEATURE_GET, CTMB_FLAG_OK, rid, req, sizeof(req));
      }

      // Only now may output reports flow: the TV's handshake() hard-fails the
      // session on ANY frame that is not HOST_CONFIG ("host config unexpected
      // type"). Flushing a stale outbox ahead of this reply put every reconnect
      // right back into a drop loop (the Phase 2b ~6 s drop/reconnect cycle, and
      // the "2 drops then stable" connects before it).
      hello_seen_.store(true);
      if (!link_live_.exchange(true)) {
        g_hello_links.fetch_add(1);
      }

      // BT firmware gate: over Bluetooth the DualSense ignores lightbar color
      // writes until a one-shot lightbar-setup release (valid_flag2 bit1 +
      // lightbar_setup=2 — what hid-playstation/SDL send on BT connect). Games
      // talk to the virtual USB pad and never send it (USB needs none), so
      // release it here each (re)connect or in-game lightbar colors stay dark.
      {
        uint8_t common[USB_OUTPUT_COMMON_LEN] = {0};
        common[38] = 0x02;  // valid_flag2: LIGHTBAR_SETUP_CONTROL_ENABLE
        common[41] = 0x02;  // lightbar_setup: LIGHT_OUT
        // Carry the synthetic color along so the bar lights on pad connect,
        // not only once a game starts writing output reports.
        const uint32_t synth = lightbar_rgb_ ? lightbar_rgb_->load(std::memory_order_relaxed) : LIGHTBAR_OFF;
        if (synth != LIGHTBAR_OFF) {
          common[1] |= 0x04;  // valid_flag1: LIGHTBAR_CONTROL_ENABLE
          common[44] = (uint8_t) (synth >> 16);
          common[45] = (uint8_t) (synth >> 8);
          common[46] = (uint8_t) synth;
          lb_last_paint_ms_.store(now_ms(), std::memory_order_relaxed);
        }
        uint8_t bt[BT_OUTPUT_LEN];
        usb_output_to_bt(common, out_seq_.fetch_add(1, std::memory_order_relaxed), bt);
        std::lock_guard<std::mutex> lk(out_mtx_);
        outbox_.emplace_back(bt, bt + BT_OUTPUT_LEN);
      }
    }

    // Session/run thread: inject-queue telemetry from the TV daemon (~4/s).
    // Integrating servo on the pacer period: true backlog (fifo_count — frames
    // parked behind a FULL credit window) or fresh drops stretch the period so
    // the queue bleeds; clean samples decay it back toward the true drain
    // cadence. Time constants: full stretch in ~1 s of pegged backlog, full
    // relax in ~12 s of clean feedback — fast enough to shed a gap-storm
    // backlog, slow enough not to chase the in-flight jitter.
    void on_pace_feedback(const ctmb_header_t *h, const uint8_t *payload) {
      if (h->payload_len < sizeof(ctmb_pace_feedback_t)) return;
      ctmb_pace_feedback_t fb {};
      std::memcpy(&fb, payload, sizeof(fb));
      uint32_t drops = fb_seen_ ? (fb.drop_total - fb_last_drop_) : 0;
      fb_last_drop_ = fb.drop_total;
      fb_seen_ = true;
      // Backlog = parked FIFO frames PLUS credit-window excess. The window
      // (outstanding) is its own ratchet: with arrival == drain it parks
      // wherever the connect burst left it (observed 5-7/12 sustained = 50-75ms
      // of latency the fifo signal never sees). Frames above the target ride
      // the same shed path, just with a gentler gain; the true in-flight floor
      // (~1-2 at this rate) stops the shed naturally — once q reaches the
      // target the excess term is 0 and adj decays back to the drain rate.
      constexpr int Q_TARGET = 3;
      int q_excess = std::max(0, (int) fb.outstanding - Q_TARGET);
      int adj = pace_adj_us_.load(std::memory_order_relaxed);
      if (fb.fifo_count > 0 || q_excess > 0 || drops > 0) {
        int up = fb.fifo_count * 8 + q_excess * 4 + (int) std::min<uint32_t>(drops, 5u) * 20;
        adj = std::min(adj + std::min(up, 40), DS5_PACE_ADJ_MAX_US);
      } else if (adj > 0) {
        adj = std::max(adj - 3, 0);
      }
      pace_adj_us_.store(adj, std::memory_order_relaxed);
      fb_last_ms_.store(
        std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count(),
        std::memory_order_relaxed);
      // Servo telemetry, ~every 10 s (40 samples at 4/s).
      dbg_fb_drops_ += drops;
      if (fb.fifo_count > dbg_fb_fifo_max_) dbg_fb_fifo_max_ = fb.fifo_count;
      if (++dbg_fb_n_ >= 40) {
        BOOST_LOG(info) << "ds5-pace: adj=" << adj << "us ("
                        << ((hap_ ? hap_->pace_base_us() : DS5_PACE_BASE_US) + adj)
                        << "us/tick) fifo_max="
                        << dbg_fb_fifo_max_ << " q=" << (int) fb.outstanding
                        << "/" << fb.maxq << " drops+=" << dbg_fb_drops_
                        << " inj=" << fb.inj_total;
        dbg_fb_n_ = 0;
        dbg_fb_drops_ = 0;
        dbg_fb_fifo_max_ = 0;
      }
    }

    // usbip server thread: the game wrote a 47-byte USB output effects block.
    void on_game_output(const uint8_t *eff) {
      if (!hello_seen_.load(std::memory_order_relaxed)) return;  // pre-handshake: drop (state refreshes)
      uint8_t common[USB_OUTPUT_COMMON_LEN];
      std::memcpy(common, eff, USB_OUTPUT_COMMON_LEN);
      // ---- Rumble-vs-haptics flag override -----------------------------------
      // valid_flag0 bit0 = RUMBLE_EMULATION (motor bytes valid), bit1 =
      // USE_RUMBLE_NOT_HAPTICS (drive the coils from the motor emulation and
      // IGNORE the 0x12 audio-coil block).
      //
      // Some titles assert bit1 permanently while delivering their entire
      // vibration as HD-haptic PCM on the audio endpoint's ch2/3 and never
      // writing a motor value at all (AC4 Resynced measured: 55/55 outputs
      // motors=00/00, bit1 in 21 of them, coil RMS up to 0.72). Taken at face
      // value the flag tells the pad to ignore exactly the data the game is
      // sending, and the pad falls silent: the 0x13 speaker block still plays,
      // the 0x12 coil block does not. The precedent is in our own notes —
      // Cyberpunk's "classic rumble" is likewise 0 and its vibration IS the HD
      // haptic; haptic routing is per-title and must never be generalised.
      //
      // So override the flag only where it contradicts the game's own data:
      // this title has fed real coil energy AND this report carries no motor
      // value. A report that does carry one passes through untouched with the
      // flag intact, so a title that genuinely wants motor rumble is unaffected
      // — and because the gate is per-report, the override corrects itself the
      // moment the game starts driving the motors.
      const uint8_t raw_f0 = common[0];  // pre-override, for the diagnostic
      const bool motors_idle = (common[2] | common[3]) == 0;
      const bool coil_title = hap_ && haptics_on_.load(std::memory_order_relaxed) &&
                              hap_->coil_ever_active();
      const bool rumble_flag_override = coil_title && motors_idle && (common[0] & 0x03) != 0;
      if (rumble_flag_override) {
        // Clear both bits: with zero motors, RUMBLE_EMULATION only re-asserts a
        // zero rumble, and leaving it set keeps the emulation owning the coils.
        common[0] &= (uint8_t) ~0x03;
      }
      // The HELLO-time lightbar-setup release can arrive before the pad has
      // switched to extended BT mode (the 0x05/0x09/0x20 feature reads do the
      // unlock, asynchronously). Fold the release into the game's own reports
      // too — SDL ships it combined with color/rumble the same way.
      if (lb_in_release_window()) {
        common[38] |= 0x02;  // valid_flag2: LIGHTBAR_SETUP_CONTROL_ENABLE
        common[41] = 0x02;   // lightbar_setup: LIGHT_OUT
      }
      // Synthetic lightbar: libScePad titles write the lightbar once at pad
      // init — to black — and never again (on the PS5 the OS supplies the
      // player color), so the bar stays dark on PC. Ownership follows the last
      // lightbar write: a real color hands the bar to the game, black hands it
      // back to the synth — libScePad writes black both at pad init and as its
      // exit reset, so without the hand-back the bar goes dark for the rest of
      // the connect once a lightbar-aware game quits.
      bool owned = lb_game_owned_.load(std::memory_order_relaxed);
      if (common[1] & 0x04) {
        owned = (common[44] | common[45] | common[46]) != 0;
      }
      const uint32_t synth = lightbar_rgb_ ? lightbar_rgb_->load(std::memory_order_relaxed) : LIGHTBAR_OFF;
      if (!owned && synth != LIGHTBAR_OFF) {
        common[1] |= 0x04;  // valid_flag1: LIGHTBAR_CONTROL_ENABLE
        common[44] = (uint8_t) (synth >> 16);
        common[45] = (uint8_t) (synth >> 8);
        common[46] = (uint8_t) synth;
        lb_last_paint_ms_.store(now_ms(), std::memory_order_relaxed);
      }
      // Diagnostics: first outputs per connect, plus lightbar color changes at
      // most once a second (games animating the bar write a new RGB on every
      // report — unthrottled that was ~60 log lines/s on the rumble thread).
      const bool lb_write = (common[1] & 0x04) != 0;
      const bool rgb_changed = lb_write && (common[44] != dbg_rgb_[0] || common[45] != dbg_rgb_[1] || common[46] != dbg_rgb_[2]);
      if (lb_write) { dbg_rgb_[0] = common[44]; dbg_rgb_[1] = common[45]; dbg_rgb_[2] = common[46]; }
      // Everything except the lightbar colour: flags, motors and the two trigger
      // effect modes (common[10] / common[21] head the 11-byte FFB blocks).
      // The RGB rule alone samples this at whatever phase the bar animation
      // happens to run at, which is how "motors=00/00" looked like a fact when
      // it was 55 samples of an unrelated trigger — a motor or trigger write
      // between two colour changes was invisible. Log the non-colour fields on
      // their OWN change, throttled only enough to survive a title that
      // animates trigger effects per report.
      const uint64_t dbg_sig = ((uint64_t) raw_f0) | ((uint64_t) common[1] << 8) |
                               ((uint64_t) common[38] << 16) | ((uint64_t) common[2] << 24) |
                               ((uint64_t) common[3] << 32) | ((uint64_t) common[10] << 40) |
                               ((uint64_t) common[21] << 48) | ((uint64_t) common[43] << 56);
      const bool sig_changed = dbg_sig != dbg_sig_;
      dbg_sig_ = dbg_sig;
      const int64_t dbg_now = now_ms();
      if (dbg_out_n_.load(std::memory_order_relaxed) < 10 ||
          (sig_changed && dbg_now - dbg_sig_log_ms_ >= 250) ||
          (rgb_changed && dbg_now - dbg_rgb_log_ms_ >= 1000)) {
        dbg_out_n_.fetch_add(1, std::memory_order_relaxed);
        if (sig_changed) dbg_sig_log_ms_ = dbg_now;
        if (rgb_changed) dbg_rgb_log_ms_ = dbg_now;
        char msg[192];
        std::snprintf(msg, sizeof(msg),
                      "ds5-out: f0=%02x f1=%02x f2=%02x motors=%02x/%02x rt=%02x lt=%02x "
                      "setup=%02x pled=%02x rgb=%02x%02x%02x%s",
                      raw_f0, common[1], common[38], common[2], common[3],
                      common[10], common[21],
                      common[41], common[43], common[44], common[45], common[46],
                      rumble_flag_override ? " [haptics-override]" : "");
        BOOST_LOG(info) << msg;
      }
      uint8_t bt[BT_OUTPUT_LEN];
      usb_output_to_bt(common, out_seq_.fetch_add(1, std::memory_order_relaxed), bt);
      // Publish ownership and enqueue as one unit under out_mtx_: the keep-alive
      // re-checks ownership under the same lock before pushing, so a synth paint
      // can never land in the outbox after the game's first color write.
      std::lock_guard<std::mutex> lk(out_mtx_);
      lb_game_owned_.store(owned, std::memory_order_relaxed);
      outbox_.emplace_back(bt, bt + BT_OUTPUT_LEN);
    }

    static int64_t now_ms() {
      return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    // Whether outgoing reports should still carry the BT lightbar-setup release.
    //
    // This used to be a one-shot latch, set the moment the release was FOLDED
    // into a frame -- i.e. on the optimistic assumption that it arrived and
    // took effect. It cannot be observed: the pad reports no lightbar state, so
    // there is nothing to confirm against. When the assumption was wrong the
    // bar stayed under firmware control for the entire connect, because every
    // later frame was colour-only and the gate silently dropped it.
    //
    // The reproducible case is a pad that was charging over USB: it connects
    // with the firmware charge indication owning the bar, the game's very first
    // output report spends the one-shot release before the pad has finished
    // switching to extended BT mode, and the bar stays orange until unplug.
    //
    // So re-assert for a bounded window after each (re)connect instead. The
    // release is idempotent and rides along in reports that are sent anyway, so
    // repeating it costs nothing; bounding it keeps a game that legitimately
    // drives the bar from fighting a permanent re-assert.
    static constexpr int64_t LB_RELEASE_WINDOW_MS = 60000;

    bool lb_in_release_window() const {
      const int64_t t0 = lb_connect_ms_.load(std::memory_order_relaxed);
      return t0 != 0 && (now_ms() - t0) < LB_RELEASE_WINDOW_MS;
    }

    // Session thread: repaint the synthetic lightbar color every few seconds
    // while connected and no game owns the bar (see on_game_output).
    void maybe_repaint_lightbar() {
      if (!hello_seen_.load(std::memory_order_relaxed)) return;
      if (lb_game_owned_.load(std::memory_order_relaxed)) return;
      const uint32_t synth = lightbar_rgb_ ? lightbar_rgb_->load(std::memory_order_relaxed) : LIGHTBAR_OFF;
      if (synth == LIGHTBAR_OFF) return;
      const int64_t now = now_ms();
      if (now - lb_last_paint_ms_.load(std::memory_order_relaxed) < 5000) return;
      lb_last_paint_ms_.store(now, std::memory_order_relaxed);
      uint8_t common[USB_OUTPUT_COMMON_LEN] = {0};
      // Keep folding the BT lightbar-setup release for the whole re-assert
      // window: a re-paired BT link re-latches the firmware gate, and without
      // the release every color-only repaint is silently ignored — the very
      // scenario this keep-alive exists for.
      if (lb_in_release_window()) {
        common[38] = 0x02;  // valid_flag2: LIGHTBAR_SETUP_CONTROL_ENABLE
        common[41] = 0x02;  // lightbar_setup: LIGHT_OUT
      }
      common[1] = 0x04;  // valid_flag1: LIGHTBAR_CONTROL_ENABLE
      common[44] = (uint8_t) (synth >> 16);
      common[45] = (uint8_t) (synth >> 8);
      common[46] = (uint8_t) synth;
      uint8_t bt[BT_OUTPUT_LEN];
      usb_output_to_bt(common, out_seq_.fetch_add(1, std::memory_order_relaxed), bt);
      // Ownership re-check and push are one unit under out_mtx_ (see
      // on_game_output): without this, a game's first color write racing the
      // 5 s tick could be overpainted by a stale synth frame for the rest of
      // the session.
      std::lock_guard<std::mutex> lk(out_mtx_);
      if (lb_game_owned_.load(std::memory_order_relaxed)) return;
      outbox_.emplace_back(bt, bt + BT_OUTPUT_LEN);
    }

    void drain_outbox() {
      if (!hello_seen_.load(std::memory_order_relaxed)) return;
      std::deque<std::vector<uint8_t>> pending;
      { std::lock_guard<std::mutex> lk(out_mtx_); pending.swap(outbox_); }
      for (auto &bt : pending) {
        // Audio/haptic reports are paced (HOST_CONFIG advertises them); the
        // TV drains its PACED queue onto the raw-ACL injector at min(bt_pace_us,
        // 8 ms) (~125/s cap), so on-air cadence is arrival-limited to our 100/s.
        // BOTH forms must be listed: with ds5_native_audio_batched the report id
        // is 0x39, and tagging only 0x36 would send the whole batched stream
        // down the TV's unpaced path (the rate servo would have no queue to act
        // on). The rare standalone 0x32 SetState stays unpaced on purpose.
        uint32_t flags = CTMB_FLAG_OK;
        if (!bt.empty() && (bt[0] == 0x36 || bt[0] == 0x39)) flags |= CTMB_FLAG_PACED;
        send_msg(CTMB_MSG_OUTPUT_REPORT, flags, 0, bt.data(), (uint32_t) bt.size());
      }
    }

    // 10 ms grid producing paced 0x36 haptic reports from the game's iso-OUT
    // PCM. Enqueues to the outbox (drained on the run/session thread) so the
    // ENet host is only ever serviced from one thread.
    void pacer_run() {
      // Time-critical so the 10 ms grid is not descheduled by the capture/encoder
      // threads under heavy game load — an irregular grid makes the coil actuation
      // choppy (ds5_av_play.c boosts the same loop for the same reason).
      SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
      using namespace std::chrono;
      auto next = steady_clock::now();
      // RATE-SERVO'D period: base = the DS5's true 93.75/s drain cadence,
      // stretched by pace_adj_us_ (TV inject-queue feedback, on_pace_feedback)
      // so the TV queue bleeds after BT NOCP gap bursts instead of ratcheting
      // up (parked latency + drop bursts — see the clocking block in
      // ds5_haptics.h). With no live feedback (old TV app, or none for >2 s)
      // it falls back to the static -0.33% margin. Emitting faster than drain
      // (100/s was tried) overruns the TV's paced queue outright: ~320 ms
      // parked plus a drop (audible Opus discontinuity) every ~160 ms. The
      // speaker resample ratio follows the live period (set_pace_us), so one
      // 480-sample frame per tick balances at any servo setting.
      // HIGH_RESOLUTION waitable timer: Win11 ignores timer-resolution requests
      // from windowless processes, so sleep_until quantizes to the 15.625 ms
      // system tick. The absolute grid keeps the AVERAGE rate regardless
      // (expired deadlines pass through), but only at the cost of ~1.6-report
      // bursts per wake; the high-res timer removes the bursting.
      auto hpt = platf::create_high_precision_timer();
      uint8_t rep[DS5_AUDIO_REPORT_MAX];
      // Form-dependent geometry, read once: set_batched() is called before this
      // thread starts and never changes afterwards.
      const int rep_len = hap_ ? hap_->report_len() : DS5_0X36_LEN;
      const int pace_base_us = hap_ ? hap_->pace_base_us() : DS5_PACE_BASE_US;
      const int pace_scale = pace_base_us / DS5_PACE_BASE_US;  // 1 (0x36) or 2 (0x39)
      // Batched mode only: the audio SetState no longer rides every report, so
      // re-assert it ~1/s as a standalone 0x32. Cheap (142 B), self-healing if
      // one is lost, and it keeps the "never fight the game's own writes"
      // property of the inline block (same audio-only payload).
      uint8_t setstate[DS5_0X32_LEN];
      const int setstate_every = pace_base_us > 0 ? (1000000 / pace_base_us) : 47;
      // Primed to fire on the FIRST report: unlike the 0x36 form, which carries
      // the SetState inline on every tick, the batched form has to assert the
      // audio Allow bits separately — starting the counter at 0 would leave the
      // pad without them for the first ~1 s of every session (it drops the audio
      // it has not been told to route).
      int setstate_ticks = setstate_every;
      while (!pacer_stop_.load() && !stop_.load()) {
        int64_t now_ms = duration_cast<milliseconds>(
          steady_clock::now().time_since_epoch()).count();
        bool fb_live = (now_ms - fb_last_ms_.load(std::memory_order_relaxed)) < 2000;
        int adj = fb_live ? pace_adj_us_.load(std::memory_order_relaxed)
                          : DS5_PACE_FALLBACK_ADJ_US;
        // The servo's adj is a RELATIVE rate offset expressed in microseconds of
        // the 0x36 period; a batched report covers two of those, so scale it or
        // the same feedback would only stretch the wire rate half as much.
        const int adj_scaled = adj * pace_scale;
        const auto period = microseconds(pace_base_us + adj_scaled);
        if (hap_) hap_->set_pace_us(pace_base_us + adj_scaled);
        next += period;
        auto now = steady_clock::now();
        if (now - next > milliseconds(100)) next = now;  // genuine stall: no catch-up burst
        else if (next > now && hpt && *hpt) hpt->sleep_for(next - now);
        if (!hap_ || !haptics_on_.load(std::memory_order_relaxed)) continue;
        // client_ready_ (not hello_seen_): 0x36 rides UNRELIABLE ch1, so it can
        // overtake a lost HOST_CONFIG (reliable ch0) and hard-fail the TV's
        // handshake. Wait for the first post-HELLO inbound frame instead; the
        // dropped frames are disposable.
        if (hap_->build_audio(rep) && client_ready_.load(std::memory_order_relaxed)) {
          std::lock_guard<std::mutex> lk(out_mtx_);
          if (outbox_.size() < 256) outbox_.emplace_back(rep, rep + rep_len);
          if (hap_->batched() && ++setstate_ticks >= setstate_every) {
            setstate_ticks = 0;
            hap_->build_setstate_0x32(setstate);
            if (outbox_.size() < 256) outbox_.emplace_back(setstate, setstate + DS5_0X32_LEN);
          }
        }
      }
    }

    // ---- transport setup + main loop --------------------------------------
    void run() {
      ensure_enet();

      ENetAddress eaddr {};
      enet_address_set_host(&eaddr, "0.0.0.0");
      enet_address_set_port(&eaddr, (uint16_t) dport_);
      host_ = enet_host_create(AF_INET, &eaddr, 2, 2, 0, 0);
      if (!host_) {
        BOOST_LOG(warning) << "ds5-bridge: enet_host_create on port "sv << dport_
                           << " failed; ENet path unavailable"sv;
      }

      SOCKET ls = open_tcp_listener();
      u_long nb = 1;
      if (ls != INVALID_SOCKET) ioctlsocket(ls, FIONBIO, &nb);

      while (!stop_.load()) {
        // 1) Select a transport (ENet CONNECT wins if it arrives; else TCP accept).
        if (transport_ == NONE) {
          if (host_) {
            ENetEvent ev;
            if (enet_host_service(host_, &ev, 2) > 0 && ev.type == ENET_EVENT_TYPE_CONNECT) {
              peer_ = ev.peer;
              transport_ = ENET;
              // AC_VO / DSCP-EF on the UDP socket (same helper Sunshine's A/V
              // sockets use): the 93.75/s audio stream must not queue behind
              // bulk WiFi traffic — every late frame is a TV-side PLC gap.
              // (This ENet fork stores peer addresses as sockaddr_storage.)
              boost::asio::ip::address peer_addr;
              uint16_t peer_port = 0;
              auto *sa = (const sockaddr *) &peer_->address.address;
              if (sa->sa_family == AF_INET) {
                auto *s4 = (const sockaddr_in *) sa;
                boost::asio::ip::address_v4::bytes_type b {};
                std::memcpy(b.data(), &s4->sin_addr, b.size());
                peer_addr = boost::asio::ip::address_v4(b);
                peer_port = ntohs(s4->sin_port);
              } else if (sa->sa_family == AF_INET6) {
                auto *s6 = (const sockaddr_in6 *) sa;
                boost::asio::ip::address_v6::bytes_type b {};
                std::memcpy(b.data(), &s6->sin6_addr, b.size());
                peer_addr = boost::asio::ip::address_v6(b);
                peer_port = ntohs(s6->sin6_port);
              }
              if (peer_port) {
                qos_ = platf::enable_socket_qos((uintptr_t) host_->socket, peer_addr,
                                                peer_port, platf::qos_data_type_e::audio, true);
              }
              BOOST_LOG(info) << "ds5-bridge: session "sv << label() << " transport=ENet"sv
                              << (qos_ ? " (AC_VO)"sv : ""sv);
            }
          }
          if (transport_ == NONE && ls != INVALID_SOCKET) {
            SOCKET c = ::accept(ls, nullptr, nullptr);
            if (c != INVALID_SOCKET) {
              int one = 1;
              setsockopt(c, IPPROTO_TCP, TCP_NODELAY, (const char *) &one, sizeof(one));
              ioctlsocket(c, FIONBIO, &nb);
              tcp_client_ = c;
              transport_ = TCP;
              BOOST_LOG(info) << "ds5-bridge: session "sv << label() << " transport=TCP"sv;
            }
          }
          if (transport_ == NONE) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
          }
        }

        // 2) Pump inbound messages.
        if (transport_ == ENET) {
          pump_enet();
        } else {
          pump_tcp();
        }

        // 3) Flush outbound (paced BT output reports produced by the game).
        drain_outbox();

        // 3b) Lightbar keep-alive: after a lightbar-aware game exits there are
        // no further game outputs to ride on, and a BT re-pair can drop the
        // latched color. Repaint the synthetic color at a slow cadence while
        // no game owns the bar.
        maybe_repaint_lightbar();

        // 4) A dropped link recycles the transport (the client reconnects to the
        // same port and re-HELLOs) — the virtual device + vhci stay attached.
        if (link_down_) reset_transport();
      }

      teardown(ls);
    }

    void reset_transport() {
      if (tcp_client_ != INVALID_SOCKET) { ::closesocket(tcp_client_); tcp_client_ = INVALID_SOCKET; }
      qos_.reset();
      peer_ = nullptr;
      rxbuf_.clear();
      transport_ = NONE;
      link_down_ = false;
      // Nothing queued while disconnected may greet the fresh link: the TV's
      // handshake fails hard on any pre-HOST_CONFIG frame. hello_seen_ regates
      // the producers; the outbox drops what already accumulated.
      hello_seen_.store(false);
      client_ready_.store(false, std::memory_order_relaxed);
      { std::lock_guard<std::mutex> lk(out_mtx_); outbox_.clear(); }
      // Fresh link may be a fresh pad connect: re-arm the lightbar-setup
      // release, lightbar ownership and the per-connect output diagnostics.
      // Zero (not now_ms()) so the window opens at the next HELLO -- there is
      // no link to carry the release while disconnected.
      lb_connect_ms_.store(0, std::memory_order_relaxed);
      lb_game_owned_.store(false, std::memory_order_relaxed);
      dbg_out_n_.store(0, std::memory_order_relaxed);
      // Servo: the next session may be a fresh daemon run (drop_total restarts)
      // — rebase the delta and fall back to the static margin until feedback
      // flows again. The learned adj is kept; it decays on clean samples.
      fb_seen_ = false;
      fb_last_ms_.store(0, std::memory_order_relaxed);
      // The touchpad-mouse only advances on reports; with the link gone a
      // held synthesized button would stay down forever. Scoped to this pad:
      // a second pad's live gesture must survive. The client preference falls
      // only with the LAST live link -- it belongs to the client, not to one
      // pad's transport, and every link re-asserts it after its handshake.
      tpmouse::reset((uintptr_t) this);
      if (link_live_.exchange(false) && g_hello_links.fetch_sub(1) == 1) {
        tpmouse::set_client_mode(-1);
      }
      BOOST_LOG(info) << "ds5-bridge: session "sv << label() << " link dropped; awaiting reconnect"sv;
    }

    void pump_enet() {
      ENetEvent ev;
      int rc = enet_host_service(host_, &ev, 2);
      while (rc > 0) {
        if (ev.type == ENET_EVENT_TYPE_RECEIVE) {
          decode_stream(ev.packet->data, ev.packet->dataLength);
          enet_packet_destroy(ev.packet);
        } else if (ev.type == ENET_EVENT_TYPE_DISCONNECT) {
          link_down_ = true;
          break;
        }
        rc = enet_host_service(host_, &ev, 0);
      }
    }

    void pump_tcp() {
      uint8_t buf[2048];
      int n = ::recv(tcp_client_, (char *) buf, sizeof(buf), 0);
      if (n > 0) {
        rxbuf_.insert(rxbuf_.end(), buf, buf + n);
        drain_rxbuf();
      } else if (n == 0) {
        link_down_ = true;
      } else {
        int e = WSAGetLastError();
        if (e != WSAEWOULDBLOCK) link_down_ = true;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }

    // For ENet, one CTMB message per packet — but be tolerant and frame it too.
    void decode_stream(const uint8_t *data, size_t len) {
      rxbuf_.insert(rxbuf_.end(), data, data + len);
      drain_rxbuf();
    }

    void drain_rxbuf() {
      size_t off = 0;
      while (rxbuf_.size() - off >= sizeof(ctmb_header_t)) {
        ctmb_header_t h;
        std::memcpy(&h, rxbuf_.data() + off, sizeof(h));
        if (h.magic != CTMB_MAGIC || h.payload_len > CTMB_MAX_PAYLOAD) {
          // Desync — drop everything and resync on the next packet boundary.
          rxbuf_.clear();
          return;
        }
        if (rxbuf_.size() - off < sizeof(ctmb_header_t) + h.payload_len) break;
        const uint8_t *pl = rxbuf_.data() + off + sizeof(ctmb_header_t);
        on_message(&h, pl);
        off += sizeof(ctmb_header_t) + h.payload_len;
      }
      if (off > 0) rxbuf_.erase(rxbuf_.begin(), rxbuf_.begin() + off);
    }

    SOCKET open_tcp_listener() {
      SOCKET s = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
      if (s == INVALID_SOCKET) return INVALID_SOCKET;
      int yes = 1;
      setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (const char *) &yes, sizeof(yes));
      sockaddr_in a {};
      a.sin_family = AF_INET;
      a.sin_addr.s_addr = INADDR_ANY;
      a.sin_port = htons((uint16_t) dport_);
      if (::bind(s, (sockaddr *) &a, sizeof(a)) != 0 || ::listen(s, 1) != 0) {
        ::closesocket(s);
        return INVALID_SOCKET;
      }
      u_long nb = 1;
      ioctlsocket(s, FIONBIO, &nb);
      return s;
    }

    void teardown(SOCKET ls) {
      // Same as reset_transport(): no more reports will arrive to release a
      // held button, and this link's share of the client preference ends.
      tpmouse::reset((uintptr_t) this);
      if (link_live_.exchange(false) && g_hello_links.fetch_sub(1) == 1) {
        tpmouse::set_client_mode(-1);
      }
      pacer_stop_.store(true);
      if (pacer_thread_.joinable()) pacer_thread_.join();
      if (attach_thread_.joinable()) attach_thread_.join();
      if (slot_) {
        vhci_detach(vhci_port_.load());
        usbip_->remove_slot(slot_);
        slot_.reset();
      }
      if (tcp_client_ != INVALID_SOCKET) { ::closesocket(tcp_client_); tcp_client_ = INVALID_SOCKET; }
      if (ls != INVALID_SOCKET) ::closesocket(ls);
      qos_.reset();
      if (host_) { enet_host_destroy(host_); host_ = nullptr; }
      peer_ = nullptr;
    }

    enum transport_e { NONE, ENET, TCP };

    usbip_ds5_device *usbip_;
    std::mutex label_mtx_;
    std::string ctmb_busid_;  // guarded by label_mtx_ (relabel from control thread)
    int dport_;
    // Desired HD-haptics state (config), pushed by the control thread on every
    // BRIDGE_START (incl. adopts); latched into haptics_on_ at the next HELLO so
    // the documented "takes effect on the next connect" A/B semantics hold even
    // though adopted sessions never re-run the constructor.
    std::atomic<bool> haptics_want_ {false};
    // Report form for this session's whole lifetime (see set_batched).
    const bool audio_batched_ {false};
    const int audio_cushion_ {4};
    std::atomic<bool> haptics_on_ {false};

    std::unique_ptr<ds5_haptic_builder> hap_;   // Phase 2 0x36 builder (gated by haptics_on_)
    std::thread pacer_thread_;
    std::atomic<bool> pacer_stop_ {false};

    std::atomic<bool> stop_ {false};
    bool link_down_ {false};
    // True from HOST_CONFIG sent (handshake answered) until the next link drop.
    // Output-report producers and the drain are gated on it: the TV handshake
    // hard-fails on any frame that precedes HOST_CONFIG. Atomic — set/cleared on
    // the session thread, read by the pacer and usbip threads.
    std::atomic<bool> hello_seen_ {false};
    std::atomic<bool> link_live_ {false};  // this session holds one g_hello_links count

    // Rate servo (on_pace_feedback writes on the run thread, pacer reads).
    std::atomic<int> pace_adj_us_ {0};       // period stretch over the base cadence
    std::atomic<int64_t> fb_last_ms_ {0};    // steady-clock ms of the last feedback
    uint32_t fb_last_drop_ = 0;              // run thread only
    bool fb_seen_ = false;                   // run thread only
    uint32_t dbg_fb_n_ = 0, dbg_fb_drops_ = 0, dbg_fb_fifo_max_ = 0;
    std::thread thread_;

    transport_e transport_ {NONE};
    ENetHost *host_ {nullptr};
    ENetPeer *peer_ {nullptr};
    std::unique_ptr<platf::deinit_t> qos_;  // AC_VO flow on the ENet socket (per link)
    SOCKET tcp_client_ {INVALID_SOCKET};
    std::vector<uint8_t> rxbuf_;

    std::shared_ptr<slot_t> slot_;
    std::thread attach_thread_;
    std::atomic<int> vhci_port_ {-1};
    uint32_t seq_ {0};
    // BT report sequence nibble; bumped from both the usbip server thread
    // (on_game_output) and the session thread (HELLO paint, lightbar
    // keep-alive), hence atomic.
    std::atomic<uint8_t> out_seq_ {0};
    // Lightbar state crosses the usbip server thread (on_game_output), the
    // session thread (HELLO paint, keep-alive, reset_transport) and the control
    // thread — all flags atomic. dbg_rgb_/dbg_rgb_log_ms_ are usbip-thread-only.
    // Start of the current connect's lightbar-setup re-assert window (0 = none).
    std::atomic<int64_t> lb_connect_ms_ {0};
    std::atomic<bool> lb_game_owned_ {false};
    std::atomic<int64_t> lb_last_paint_ms_ {0};
    const std::atomic<uint32_t> *lightbar_rgb_ {nullptr};
    std::atomic<uint32_t> dbg_out_n_ {0};
    uint8_t dbg_rgb_[3] {};
    int64_t dbg_rgb_log_ms_ {0};
    // Non-colour output fields (flags/motors/triggers/player LEDs), packed, so a
    // change in any of them logs on its own instead of only when the lightbar
    // animation happens to tick. Touched on the usbip output thread only.
    uint64_t dbg_sig_ {~0ull};
    int64_t dbg_sig_log_ms_ {0};
    // Set on the first post-HELLO inbound frame; gates the unreliable 0x36
    // stream so it cannot outrun a lost HOST_CONFIG (see on_message).
    std::atomic<bool> client_ready_ {false};

    std::mutex out_mtx_;
    std::deque<std::vector<uint8_t>> outbox_;

    std::mutex feat_mtx_;
    std::array<std::array<uint8_t, 64>, 256> feat_cache_ {};
    std::array<bool, 256> feat_have_ {};
  };

  // ===========================================================================
  //  bridge_host
  // ===========================================================================
  bridge_host::bridge_host() = default;
  bridge_host::~bridge_host() { stop(); }

  bool bridge_host::start(int port) {
    if (running_.load()) return true;
    port_ = port > 0 ? port : 48054;

    WSADATA wsa;
    static std::once_flag wsa_once;
    std::call_once(wsa_once, [&] { WSAStartup(MAKEWORD(2, 2), &wsa); });

    if (!usbip_.start()) {
      BOOST_LOG(error) << "ds5-bridge: usbip device failed to start; bridge disabled"sv;
      return false;
    }

    // Control-plane TCP listener (BRIDGE_START/STOP) + UDP discovery responder.
    SOCKET ts = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    SOCKET us = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    int yes = 1;
    setsockopt(ts, SOL_SOCKET, SO_REUSEADDR, (const char *) &yes, sizeof(yes));
    setsockopt(us, SOL_SOCKET, SO_REUSEADDR, (const char *) &yes, sizeof(yes));
    sockaddr_in a {};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = INADDR_ANY;
    a.sin_port = htons((uint16_t) port_);
    if (ts == INVALID_SOCKET || us == INVALID_SOCKET ||
        ::bind(ts, (sockaddr *) &a, sizeof(a)) != 0 || ::listen(ts, 4) != 0 ||
        ::bind(us, (sockaddr *) &a, sizeof(a)) != 0) {
      BOOST_LOG(error) << "ds5-bridge: control plane bind on port "sv << port_
                       << " failed (in use?); bridge disabled"sv;
      if (ts != INVALID_SOCKET) ::closesocket(ts);
      if (us != INVALID_SOCKET) ::closesocket(us);
      usbip_.stop();
      return false;
    }
    tcp_listen_ = (uintptr_t) ts;
    udp_sock_ = (uintptr_t) us;
    stop_.store(false);
    running_.store(true);
    control_thread_ = std::thread(&bridge_host::control_loop, this);
    BOOST_LOG(info) << "ds5-bridge: native provider up (control plane :"sv << port_
                    << ", usbip 127.0.0.1:3240)"sv;
    return true;
  }

  void bridge_host::control_loop() {
    SOCKET ts = (SOCKET) tcp_listen_;
    SOCKET us = (SOCKET) udp_sock_;
    while (!stop_.load()) {
      fd_set rd;
      FD_ZERO(&rd);
      FD_SET(ts, &rd);
      FD_SET(us, &rd);
      timeval tv { 0, 200000 };  // 200 ms so stop_ is checked promptly
      int r = ::select(0, &rd, nullptr, nullptr, &tv);
      if (r <= 0) continue;

      if (FD_ISSET(us, &rd)) {
        char buf[256];
        sockaddr_in from {};
        int flen = sizeof(from);
        int n = ::recvfrom(us, buf, sizeof(buf) - 1, 0, (sockaddr *) &from, &flen);
        if (n > 0) {
          buf[n] = '\0';
          if (std::strncmp(buf, "CTM_DISCOVER_V1", 15) == 0) {
            std::string reply = "CTM_AGENT_V1 port=" + std::to_string(port_);
            ::sendto(us, reply.c_str(), (int) reply.size(), 0, (sockaddr *) &from, flen);
          }
        }
      }

      if (FD_ISSET(ts, &rd)) {
        SOCKET c = ::accept(ts, nullptr, nullptr);
        if (c == INVALID_SOCKET) continue;
        // One command per connection (matches the client's connect/send/recv/close).
        std::string line;
        char ch;
        int guard = 0;
        while (guard++ < 512) {
          int k = ::recv(c, &ch, 1, 0);
          if (k <= 0) break;
          if (ch == '\n') break;
          line.push_back(ch);
        }
        std::string resp = handle_command(line);
        resp.push_back('\n');
        ::send(c, resp.c_str(), (int) resp.size(), 0);
        ::closesocket(c);
      }
    }
  }

  std::string bridge_host::handle_command(const std::string &line) {
    std::istringstream iss(line);
    std::string cmd;
    iss >> cmd;
    if (cmd == "BRIDGE_START") {
      std::string kind, busid;
      int dport = 0;
      iss >> kind >> dport >> busid;
      if (kind != "ds5") {
        return "ERR unsupported kind (native provider is DS5-only in this build)";
      }
      if (dport <= 0 || busid.empty()) {
        return "ERR bad args";
      }
      std::lock_guard<std::mutex> lk(sessions_mtx_);
      // The data port is a controller's stable identity across reconnects: the TV
      // reuses it but issues a fresh busid each time (ctm-ds5-1 -> ctm-ds5-2 ...,
      // restarting from 1 whenever the TV app relaunches). The map is keyed by
      // dport for exactly that reason — a reused busid must never collide with
      // (let alone destroy) another pad's live session. If a session already owns
      // this dport, adopt it — its ENet host, usbip slot and vhci attach stay up,
      // so the game never sees a re-plug — and just relabel it to the incoming
      // busid so a later BRIDGE_STOP resolves. Spinning up a second session would
      // only race for the same port: enet_host_create fails and the loser lingers
      // as a zombie TCP-fallback session the reconnecting peer never reaches.
      auto it = sessions_.find(dport);
      if (it != sessions_.end()) {
        it->second->relabel(busid);
        it->second->set_haptics(haptics_.load());
        BOOST_LOG(info) << "ds5-bridge: BRIDGE_START ds5 port="sv << dport << " busid="sv
                        << busid << " (reconnect; adopted live session on this port)"sv;
        return "OK";
      }
      auto sess = std::make_unique<bridge_session>(&usbip_, busid, dport, haptics_.load(),
                                                  audio_batched_.load(), audio_cushion_.load(),
                                                  &lightbar_rgb_);
      sess->start();
      sessions_[dport] = std::move(sess);
      BOOST_LOG(info) << "ds5-bridge: BRIDGE_START ds5 port="sv << dport << " busid="sv << busid;
      return "OK";
    }
    if (cmd == "BRIDGE_STOP") {
      std::string busid;
      iss >> busid;
      stop_session(busid);
      return "OK";
    }
    return "ERR unknown command";
  }

  void bridge_host::stop_session(const std::string &busid) {
    std::unique_ptr<bridge_session> victim;
    {
      std::lock_guard<std::mutex> lk(sessions_mtx_);
      for (auto it = sessions_.begin(); it != sessions_.end(); ++it) {
        if (it->second->label() != busid) continue;
        victim = std::move(it->second);
        sessions_.erase(it);
        break;
      }
      if (!victim) return;  // stale busid (e.g. from before a TV app relaunch)
    }
    victim->stop();  // join outside the lock
    BOOST_LOG(info) << "ds5-bridge: BRIDGE_STOP busid="sv << busid;
  }

  void bridge_host::stop() {
    if (!running_.exchange(false)) return;
    stop_.store(true);
    if (tcp_listen_ != ~uintptr_t(0)) { ::closesocket((SOCKET) tcp_listen_); tcp_listen_ = ~uintptr_t(0); }
    if (udp_sock_ != ~uintptr_t(0)) { ::closesocket((SOCKET) udp_sock_); udp_sock_ = ~uintptr_t(0); }
    if (control_thread_.joinable()) control_thread_.join();
    {
      std::lock_guard<std::mutex> lk(sessions_mtx_);
      for (auto &kv : sessions_) kv.second->stop();
      sessions_.clear();
    }
    usbip_.stop();
    BOOST_LOG(info) << "ds5-bridge: native provider stopped"sv;
  }

}  // namespace platf::ds5_bridge
