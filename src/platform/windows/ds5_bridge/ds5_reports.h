/**
 * @file src/platform/windows/ds5_bridge/ds5_reports.h
 * @brief DualSense report translation between the USB form the virtual device
 *        exchanges with the game and the Bluetooth form the Aurora TV client
 *        injects into the real controller.
 *
 * All layout facts come from public sources (Linux hid-playstation.c, the
 * DualSense wiki / DJm00n ControllersInfo) and were re-verified against this
 * hardware on 2026-08-28 (600 live reports off /dev/hidraw3 while streaming).
 * The DualSense input/output "common block" is identical across the USB and BT
 * transports, so translation is very nearly a reframe -- with one exception:
 * the common block carries an 8-byte authentication tag the pad computes over
 * its own report, and that tag cannot survive being reframed. Everything else
 * is copied through; see bt_input_to_usb.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace platf::ds5_bridge {

  // 64-byte USB input report the virtual device delivers to the game:
  //   [0x01][63-byte common input block].
  constexpr int USB_INPUT_LEN = 64;
  // The last 8 bytes of the common block are an authentication tag the pad
  // computes over its OWN report (VIIPER 0aaae619 names it an AES-CMAC). It is
  // the only field in the block we must NOT forward -- see bt_input_to_usb.
  constexpr int USB_INPUT_TAG_OFFSET = 56;
  constexpr int USB_INPUT_TAG_LEN = 8;
  // 47-byte USB output "effects" payload (the 0x02 report body, id stripped)
  // handed to us by the virtual device when the game writes an output report.
  constexpr int USB_OUTPUT_COMMON_LEN = 47;

  // BT input report id (extended DualSense input) and its common-block offset.
  //   [0x31][seq_tag][63-byte common input block][... sensors/touch ...][crc32]
  constexpr uint8_t BT_INPUT_REPORT_ID = 0x31;
  constexpr int BT_INPUT_COMMON_OFFSET = 2;

  // BT output report:
  //   [0x31][seq_tag][0x10][47-byte common output block][24 reserved][crc32 LE]
  constexpr uint8_t BT_OUTPUT_REPORT_ID = 0x31;
  constexpr uint8_t BT_OUTPUT_TAG = 0x10;
  constexpr int BT_OUTPUT_LEN = 78;

  // DualSense Bluetooth reports are validated with a CRC32 whose input is
  // prefixed by a transaction-type seed byte: 0xA2 for HID Data/Output.
  constexpr uint8_t PS_OUTPUT_CRC_SEED = 0xA2;

  /// Raw reflected CRC-32 update (poly 0xEDB88820), no pre/post inversion — so
  /// it can be chained (seed byte, then report bytes) the way the DualSense
  /// firmware computes its BT checksum. Callers seed with 0xFFFFFFFF and invert
  /// the final result themselves.
  inline uint32_t ds5_crc32_update(uint32_t crc, const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; ++i) {
      crc ^= data[i];
      for (int b = 0; b < 8; ++b) {
        crc = (crc >> 1) ^ (0xEDB88320u & (~(crc & 1u) + 1u));
      }
    }
    return crc;
  }

  /**
   * @brief Convert a Bluetooth 0x31 input report from the TV into the 64-byte
   *        USB 0x01 report the virtual device feeds the game. Returns false if
   *        the source is too short or not a 0x31 report.
   */
  inline bool bt_input_to_usb(const uint8_t *bt, size_t bt_len, uint8_t out[USB_INPUT_LEN]) {
    if (!bt || bt_len < (size_t) (BT_INPUT_COMMON_OFFSET + USB_INPUT_LEN - 1) ||
        bt[0] != BT_INPUT_REPORT_ID) {
      return false;
    }
    out[0] = 0x01;
    std::memcpy(&out[1], &bt[BT_INPUT_COMMON_OFFSET], USB_INPUT_LEN - 1);
    // Drop the pad's authentication tag instead of forwarding it. It is
    // computed over the PHYSICAL report; we hand the game a different one --
    // the report id is reframed 0x31 -> 0x01, and while the desktop
    // touchpad-mouse owns this pad bridge_host.cpp additionally lifts the touch
    // contacts and the touchpad click out of this very copy. A forwarded tag
    // therefore cannot match what the game receives, and passing a stale tag
    // asserts an authenticity we cannot back; zero is the honest "absent".
    //
    // Measured on this hardware 2026-08-28, 600 live reports during a stream
    // plus 400 idle: bytes 56..63 take all 256 byte values, never repeat as a
    // tail across 1000 reports and are never zero -- a live per-report tag,
    // not padding. Bytes 54 and 55 sit in the same reserved run but are hard
    // zero here and are left untouched deliberately.
    std::memset(&out[USB_INPUT_TAG_OFFSET], 0, USB_INPUT_TAG_LEN);
    return true;
  }

  /**
   * @brief Build a full, CRC-signed Bluetooth 0x31 output report from the
   *        47-byte USB output common block the game wrote. @p seq is the caller's
   *        monotonic counter (only the low 4 bits are used in the BT seq tag).
   */
  inline void usb_output_to_bt(const uint8_t *common, uint8_t seq, uint8_t out[BT_OUTPUT_LEN]) {
    std::memset(out, 0, BT_OUTPUT_LEN);
    out[0] = BT_OUTPUT_REPORT_ID;
    out[1] = (uint8_t) (seq << 4);
    out[2] = BT_OUTPUT_TAG;
    std::memcpy(&out[3], common, USB_OUTPUT_COMMON_LEN);
    // CRC32 over the 0xA2 seed byte followed by bytes [0 .. BT_OUTPUT_LEN-4).
    const uint8_t seed = PS_OUTPUT_CRC_SEED;
    uint32_t crc = ds5_crc32_update(0xFFFFFFFFu, &seed, 1);
    crc = ds5_crc32_update(crc, out, BT_OUTPUT_LEN - 4);
    crc = ~crc;
    out[74] = (uint8_t) (crc & 0xFF);
    out[75] = (uint8_t) ((crc >> 8) & 0xFF);
    out[76] = (uint8_t) ((crc >> 16) & 0xFF);
    out[77] = (uint8_t) ((crc >> 24) & 0xFF);
  }

}  // namespace platf::ds5_bridge
