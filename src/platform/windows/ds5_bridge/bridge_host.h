/**
 * @file src/platform/windows/ds5_bridge/bridge_host.h
 * @brief Host-side CTM-Bridge server: the license-clean in-process replacement
 *        for the external ctm-usbip.exe agent.
 *
 * Speaks the CTMB protocol Aurora's GPL HID-passthrough client already talks:
 *   - control plane on <port> (default 48054): UDP discovery
 *     (CTM_DISCOVER_V1 -> CTM_AGENT_V1 port=<port>) + a TCP line protocol
 *     (BRIDGE_START <kind> <dport> <busid> / BRIDGE_STOP <busid>);
 *   - one data-plane session per plugged controller on its own <dport>
 *     (ENet/UDP preferred, TCP fallback), carrying HELLO / HOST_CONFIG /
 *     INPUT_REPORT / OUTPUT_REPORT / FEATURE_* messages.
 *
 * Each session bridges the TV's real DualSense to a virtual USB DualSense
 * presented to the game by usbip_ds5_device (loopback usbip-win2).
 *
 * Phase 1 supports DS5 only (kind "ds5"); other kinds are rejected so the CTM
 * agent can still handle them if configured. HD-haptic audio (0x36) is Phase 2.
 */
#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "src/platform/windows/ds5_bridge/usbip_ds5_device.h"

namespace platf::ds5_bridge {

  /// Sentinel for "no synthetic lightbar color" (top byte set — real colors
  /// are plain 0x00RRGGBB).
  inline constexpr uint32_t LIGHTBAR_OFF = 0xFF000000u;

  class bridge_session;

  class bridge_host {
  public:
    // Both special members are defined out-of-line (in the .cpp, where
    // bridge_session is complete) so no TU that merely holds a bridge_host has
    // to instantiate std::unique_ptr<bridge_session>'s destructor.
    bridge_host();
    ~bridge_host();

    bridge_host(const bridge_host &) = delete;
    bridge_host &operator=(const bridge_host &) = delete;

    /// Start the usbip device + the control plane on @p port. Idempotent.
    bool start(int port);
    /// Stop all sessions, the control plane and the usbip device.
    void stop();
    bool is_running() const { return running_.load(); }

    /// Phase 2 HD-haptics toggle. Read when a session is created (a fresh
    /// controller connect), so flipping it takes effect on the next connect.
    void set_haptics(bool on) { haptics_.store(on); }

    /// Batched audio downlink (0x39 instead of 0x36). Like set_haptics, it is
    /// read when a session is created, so a config flip lands on the next
    /// controller connect rather than mid-stream (the pad's audio packet
    /// counter steps by frames-per-report, so the form must not change under it).
    void set_audio_batched(bool on) { audio_batched_.store(on); }
    void set_audio_cushion(int n) { audio_cushion_.store(n); }
    /// AudioControl echo/noise-cancel bits in the audio SetState (next connect).
    void set_audio_cancel_bits(bool on) { audio_cancel_bits_.store(on); }
    /// Negative rate-servo branch that makes up dropped audio (next connect).
    void set_pace_refill(bool on, int cap_ms) {
      pace_refill_.store(on);
      pace_refill_cap_ms_.store(cap_ms);
    }
    /// Rumble-vs-haptics override hand-back form (next connect).
    void set_haptics_handback(bool on) { haptics_handback_.store(on); }

    /// Synthetic lightbar color (0x00RRGGBB) or LIGHTBAR_OFF. Sessions read it
    /// live on every game output, so config changes apply immediately.
    void set_lightbar(uint32_t rgb) { lightbar_rgb_.store(rgb); }

  private:
    void control_loop();
    std::string handle_command(const std::string &line);
    void stop_session(const std::string &busid);

    std::atomic<bool> running_ {false};
    std::atomic<bool> stop_ {false};
    std::atomic<bool> haptics_ {false};
    std::atomic<bool> audio_batched_ {false};
    std::atomic<int> audio_cushion_ {4};
    std::atomic<bool> audio_cancel_bits_ {false};
    std::atomic<bool> pace_refill_ {false};
    std::atomic<int> pace_refill_cap_ms_ {64};
    std::atomic<bool> haptics_handback_ {false};
    std::atomic<uint32_t> lightbar_rgb_ {LIGHTBAR_OFF};
    int port_ {48054};

    usbip_ds5_device usbip_;

    uintptr_t tcp_listen_ {~uintptr_t(0)};
    uintptr_t udp_sock_ {~uintptr_t(0)};
    std::thread control_thread_;

    std::mutex sessions_mtx_;
    // Keyed by data port — the controller's stable identity across reconnects.
    // The TV's busid is only a label (it restarts from ctm-ds5-1 whenever the
    // TV app relaunches, so keying by busid let a reconnecting pad collide with
    // — and destroy — another pad's live session). BRIDGE_STOP resolves busids
    // by scanning the labels.
    std::map<int, std::unique_ptr<bridge_session>> sessions_;
  };

}  // namespace platf::ds5_bridge
