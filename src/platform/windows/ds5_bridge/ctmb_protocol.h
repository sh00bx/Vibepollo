/**
 * @file src/platform/windows/ds5_bridge/ctmb_protocol.h
 * @brief CTM-Bridge wire protocol (host side).
 *
 * License-clean interop reimplementation of the CTMB protocol that Aurora's
 * GPL-3 HID-passthrough client speaks (moonlight-tv fork,
 * src/app/hid_passthrough/ctm_bridge_protocol.h). The struct layouts and
 * constants are an interoperability specification, not copyrightable
 * expression; this header is written from scratch to let a native Vibepollo
 * provider stand in for the (unlicensed, out-of-tree) ctm-usbip.exe agent.
 *
 * The layouts below MUST stay byte-compatible with the Aurora client (little
 * endian, packed). All multi-byte header/payload fields are transmitted
 * little-endian, matching the client's #pragma pack(push,1) structs on an LE
 * (x86 TV / x86 host) target.
 */
#pragma once

#include <cstdint>

namespace platf::ds5_bridge {

  constexpr uint32_t CTMB_MAGIC = 0x54424d43u;  // "CTMB"
  constexpr uint16_t CTMB_VERSION = 1u;

  constexpr uint32_t CTMB_FLAG_OK = 0x00000001u;
  constexpr uint32_t CTMB_FLAG_PACED = 0x00000002u;
  constexpr uint32_t CTMB_MAX_PAYLOAD = 65536u;

  enum ctmb_message_type : uint16_t {
    CTMB_MSG_HELLO = 1,
    CTMB_MSG_HOST_CONFIG = 2,
    CTMB_MSG_INPUT_REPORT = 3,
    CTMB_MSG_OUTPUT_REPORT = 4,
    CTMB_MSG_FEATURE_GET = 5,
    CTMB_MSG_FEATURE_REPORT = 6,
    CTMB_MSG_LOG = 7,
    CTMB_MSG_ERROR = 8,
    CTMB_MSG_FEATURE_SET = 9,
    CTMB_MSG_ENUM = 10,
    // TV -> host inject-queue telemetry (our protocol extension, not CTM's).
    // The client sends it ONLY when HOST_CONFIG advertised
    // CTMB_HOSTCFG_PACE_FEEDBACK, so a legacy/CTM host never sees the type.
    CTMB_MSG_PACE_FEEDBACK = 11,
    // TV -> host: user preference for the DS5 touchpad-mouse synthesis
    // (ds5_touchpad_mouse). Sent once after the HOST_CONFIG handshake.
    CTMB_MSG_TPMOUSE = 12,
    // TV -> host: one DS5 microphone Opus packet (ctmb_ds5_mic_t + bytes).
    // Sent ONLY when HOST_CONFIG advertised CTMB_HOSTCFG_DS5_MIC, i.e. when
    // ds5_native_mic is enabled on this host (port plan W3-02).
    CTMB_MSG_DS5_MIC = 13,
  };

  // ctmb_host_config_t.reserved[0] capability bits (0 on a CTM host).
  constexpr uint8_t CTMB_HOSTCFG_PACE_FEEDBACK = 0x01;
  // Host decodes CTMB_MSG_DS5_MIC into the virtual pad's capture endpoint.
  constexpr uint8_t CTMB_HOSTCFG_DS5_MIC = 0x02;

  // ctmb_device_caps_t.flags capability bits (0x0001 is the client's existing
  // baseline flag): client accepts batched 0x39 DS5 audio output reports.
  constexpr uint16_t CTMB_DEVCAP_DS5_AUDIO_0X39 = 0x0002;

#pragma pack(push, 1)
  struct ctmb_header_t {
    uint32_t magic;
    uint16_t version;
    uint16_t type;
    uint32_t flags;
    uint32_t sequence;
    uint64_t timestamp_us;
    uint32_t request_id;
    uint32_t payload_len;
  };
  static_assert(sizeof(ctmb_header_t) == 32, "ctmb_header_t must be 32 bytes");

  // HELLO payload: caps, then descriptor-info, then the report descriptor bytes.
  struct ctmb_device_caps_t {
    uint16_t vendor_id;
    uint16_t product_id;
    uint16_t version;
    uint16_t bus;
    uint16_t input_report_len;
    uint16_t output_report_len;
    uint16_t feature_report_len;
    uint16_t flags;
    char path[64];
    char serial[64];
    char product[64];
    char manufacturer[64];
  };
  static_assert(sizeof(ctmb_device_caps_t) == 16 + 256, "caps layout drift");

  struct ctmb_hid_descriptor_info_t {
    uint32_t report_descriptor_len;
    uint8_t reserved[28];
  };

  struct ctmb_host_config_t {
    uint32_t bt_pace_us;
    uint16_t input_report_len;
    uint16_t output_report_len;
    uint16_t feature_report_len;
    uint8_t paced_report_count;
    uint8_t paced_report_ids[16];
    uint8_t reserved[31];
  };

  // CTMB_MSG_TPMOUSE payload. mode: 0=off, 1=auto, 2=always.
#pragma pack(push, 1)

  struct ctmb_tpmouse_t {
    uint8_t mode;
    uint8_t reserved[7];
  };

#pragma pack(pop)

  // CTMB_MSG_PACE_FEEDBACK payload: a snapshot of the TV daemon's raw-ACL
  // inject queue for this pad's link (published by ds5_txd, forwarded by the
  // client ~4/s). Drives the host pacer's rate servo: fifo_count > 0 means the
  // NOCP credit window is full AND frames are parked behind it (true backlog);
  // drop_total advancing means the parked backlog overflowed fifo_cap.
  struct ctmb_pace_feedback_t {
    uint8_t outstanding;  // in-flight TX (NOCP credit window occupancy)
    uint8_t fifo_count;   // parked behind the window (elastic FIFO depth)
    uint16_t maxq;        // credit window cap
    uint16_t fifo_cap;    // elastic FIFO cap
    uint16_t reserved0;
    uint32_t inj_total;   // daemon lifetime counters (monotonic)
    uint32_t drop_total;
    uint8_t reserved[16];
  };
  static_assert(sizeof(ctmb_pace_feedback_t) == 32, "pace feedback layout drift");

  // CTMB_MSG_DS5_MIC payload header, followed by frame_len bytes of Opus.
  // seq: per-session counter the TV app keeps for the mic stream (starts at 0,
  // wraps at 16 bits) -- NOT the pad's 4-bit report sequence, which is shared
  // with the pad-state reports and useless as a mic sequence. format: 1 = the
  // DualSense BT mic packet as captured (Opus, 48 kHz, stereo, 10 ms = 480
  // samples, TOC 0xd4, 71 bytes); other values are dropped by the host.
  constexpr uint8_t CTMB_DS5_MIC_FORMAT_OPUS_48K_10MS = 1;
  struct ctmb_ds5_mic_t {
    uint16_t seq;
    uint8_t format;
    uint8_t frame_len;
    uint32_t reserved;
  };
  static_assert(sizeof(ctmb_ds5_mic_t) == 8, "ds5 mic layout drift");
#pragma pack(pop)

}  // namespace platf::ds5_bridge
