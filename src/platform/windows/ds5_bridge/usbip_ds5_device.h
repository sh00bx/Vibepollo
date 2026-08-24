/**
 * @file src/platform/windows/ds5_bridge/usbip_ds5_device.h
 * @brief Embedded multi-slot USB/IP server presenting one or more virtual
 *        Sony DualSense devices to Windows.
 *
 * The Microsoft-signed usbip-win2 vhci client attaches to this server on
 * 127.0.0.1:3240 (loads under Secure Boot ON, empirically verified 2026-07-14),
 * so each controller enumerates as a genuine USB\VID_054C&PID_0CE6 device that
 * strict libScePad titles accept for full output (adaptive triggers, rumble,
 * lightbar).
 *
 * This is a license-clean evolution of our own host half (apollo commit
 * 8f6b61e0 + the M1 composite-descriptor work), generalised from one hard-coded
 * device to per-controller slots keyed by USB/IP busid, each with a stable
 * serial (so a session never leaks a ghost device the way a new-serial-per-
 * session agent does).
 */
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace platf::ds5_bridge {

  // 64-byte game-facing USB input report ([0]=0x01, [1..63]=DS5 common block).
  constexpr int INPUT_REPORT_LEN = 64;
  // 47-byte DS5 output effects payload (the 0x02 body, report-id stripped).
  constexpr int OUTPUT_PAYLOAD_LEN = 47;

  /**
   * @brief Everything that distinguishes one virtual pad model from another at
   *        the USB level. The session state machine is model-agnostic; a slot
   *        carries a pointer to its model (DS5 by default, DS4 for kind "ds4").
   */
  struct usb_model_t {
    const char *name;                 // for logs
    uint16_t vid, pid;
    const uint8_t *device_desc;       // 18 bytes
    const uint8_t *config_desc;
    int config_len;
    int hid_desc_off;                 // offset of the 9-byte HID class descriptor in config
    const uint8_t *hid_report_desc;
    int hid_report_len;
    uint8_t usb_output_report_id;     // 0x02 (DS5) / 0x05 (DS4)
    int usb_output_payload_len;       // 47 / 31
    int iso_bytes_per_frame;          // 8 (4ch s16) / 4 (2ch s16)
    int iso_sample_rate;              // 48000 / 32000
    // Static fallback for EP0 GET_REPORT(feature) when the slot has no live
    // provider hit; returns bytes written into out (up to 64), 0 for none.
    int (*feature_fallback)(uint8_t report_id, uint8_t *out);
  };

  const usb_model_t *ds5_usb_model();
  const usb_model_t *ds4_usb_model();

  /**
   * @brief One virtual DualSense, addressed by a USB/IP busid (e.g. "1-1").
   *
   * A slot carries the live input report (delivered on the next interrupt-IN),
   * an output callback (the game's 47-byte effects payload), and an optional
   * feature-report provider (so EP0 GET_REPORT can be answered from the real
   * controller's captured descriptors instead of static bytes).
   */
  struct slot_t {
    std::string busid;
    std::string serial;  // stable iSerialNumber (typically the pad's BT MAC)
    // USB personality of this slot. Fixed at add_slot; never changes while the
    // vhci session lives.
    const usb_model_t *model = nullptr;

    // Called on a server thread with the model's effects payload (bytes after
    // the output report id) whenever the game writes an output report.
    using output_cb = std::function<void(const uint8_t *payload)>;
    output_cb on_output;

    // Optional: fill @p out (up to 64 bytes) with a live feature report for the
    // given report id; return the length written, or 0 to fall back to the
    // captured/static default. Called on a server thread; must not block long.
    using feature_cb = std::function<int(uint8_t report_id, uint8_t *out)>;
    feature_cb on_feature;

    // Optional (Phase 2 — HD haptics): called on a server thread with the raw
    // iso-OUT PCM the game renders to the DS5's audio endpoint (EP 0x01,
    // 4ch/16-bit/48 kHz interleaved: ch0/1 speaker, ch2/3 voice-coil haptics).
    // The session turns it into a paced DS5 0x36 report. Unset in Phase 1, where
    // the iso URB is simply completed and the PCM discarded. Must not block long.
    using iso_out_cb = std::function<void(const uint8_t *pcm, size_t len)>;
    iso_out_cb on_iso_out;

    std::mutex input_mtx;
    uint8_t input_report[INPUT_REPORT_LEN] {};
    std::atomic<bool> attached {false};
    // The live vhci import's client socket (set by serve_session while attached,
    // ~0 otherwise). remove_slot() closes it to force the session thread — which
    // invokes on_output/on_iso_out — to exit before the owner destroys those
    // callbacks' captured state. uintptr_t so the header stays winsock-free.
    std::atomic<uintptr_t> session_sock {~uintptr_t(0)};

    slot_t() {
      input_report[0] = 0x01;
      input_report[1] = input_report[2] = input_report[3] = input_report[4] = 0x80;
    }
  };

  /**
   * @brief The single loopback USB/IP listener. Slots are added/removed as
   *        controllers plug and unplug; the vhci selects a slot by busid at
   *        import time.
   */
  class usbip_ds5_device {
  public:
    usbip_ds5_device() = default;
    ~usbip_ds5_device();

    usbip_ds5_device(const usbip_ds5_device &) = delete;
    usbip_ds5_device &operator=(const usbip_ds5_device &) = delete;

    /// Bind+listen on 127.0.0.1:3240 and spawn the accept thread. Idempotent.
    /// Returns false if Winsock or bind/listen fails (e.g. port already in use).
    bool start();

    /// Stop the server, close all sockets, join threads. Safe to call repeatedly.
    void stop();

    bool is_running() const { return running_.load(); }

    /// Register a controller slot. Returns a shared handle used for set_input /
    /// remove_slot. The next free busid ("1-1", "1-2", ...) is assigned if
    /// @p busid is empty. @p model defaults to the DualSense.
    std::shared_ptr<slot_t> add_slot(const std::string &serial,
                                     slot_t::output_cb on_output,
                                     slot_t::feature_cb on_feature,
                                     std::string busid = {},
                                     const usb_model_t *model = nullptr);

    /// Remove a slot (closes its live vhci session, if any).
    void remove_slot(const std::shared_ptr<slot_t> &slot);

    /// Update a slot's live 64-byte input report.
    void set_input(const std::shared_ptr<slot_t> &slot, const uint8_t report[INPUT_REPORT_LEN]);

    /// Busid a slot was assigned (for the `usbip attach -b <busid>` helper).
    static std::string busid_of(const std::shared_ptr<slot_t> &slot) { return slot ? slot->busid : std::string {}; }

  private:
    void accept_loop();
    void serve_session(uintptr_t client_sock);
    std::shared_ptr<slot_t> find_slot(const char *busid);
    std::string next_busid_locked() const;

    std::atomic<bool> running_ {false};
    std::atomic<bool> stop_ {false};

    uintptr_t listen_sock_ {~uintptr_t(0)};
    std::thread accept_thread_;

    std::mutex client_socks_mtx_;
    std::vector<uintptr_t> client_socks_;  // live vhci session sockets

    std::mutex slots_mtx_;
    std::vector<std::shared_ptr<slot_t>> slots_;
  };

}  // namespace platf::ds5_bridge
