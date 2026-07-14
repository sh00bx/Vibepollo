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
  };

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
#pragma pack(pop)

}  // namespace platf::ds5_bridge
