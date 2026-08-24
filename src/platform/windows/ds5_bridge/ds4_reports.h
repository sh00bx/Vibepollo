/**
 * @file src/platform/windows/ds5_bridge/ds4_reports.h
 * @brief DualShock 4 report translation between the USB form the virtual
 *        device exchanges with the game and the Bluetooth form the TV client
 *        writes into the real controller.
 *
 * Layout facts come from public sources (Linux hid-playstation.c, the Game
 * Controller Collective wiki, DS4Windows' DualShock4BluetoothAudioProtocol)
 * and from CTM-Bridge's GPL-3 CTM-USBIP reference (docs/ds4_knowledge.md,
 * maps/ds4_usb_over_ds4_bt.map — the probed "Layout B" audio protocol).
 * Like the DualSense, the DS4's input/output common block is identical across
 * USB and BT — only the framing and the trailing BT CRC32 differ.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "src/platform/windows/ds5_bridge/ds5_reports.h"  // ds5_crc32_update / PS_OUTPUT_CRC_SEED

namespace platf::ds5_bridge {

  // 64-byte USB input report the virtual device delivers to the game:
  //   [0x01][63-byte common input block].
  constexpr int DS4_USB_INPUT_LEN = 64;
  // 31-byte USB output common block (the 0x05 body, report-id stripped):
  //   [0]=valid_flag0 [1]=valid_flag1 [2]=reserved [3]=motor_right(weak)
  //   [4]=motor_left(strong) [5..7]=lightbar RGB [8]=blink_on [9]=blink_off
  //   [18..19]=headphone L/R volume [21]=speaker volume (TV-stamped).
  constexpr int DS4_USB_OUTPUT_COMMON_LEN = 31;
  constexpr uint8_t DS4_USB_OUTPUT_REPORT_ID = 0x05;

  // BT input report (extended mode, unlocked by reading feature 0x05):
  //   [0x11][hdr][hdr][63-byte common input block ...][crc32]
  constexpr uint8_t DS4_BT_INPUT_REPORT_ID = 0x11;
  constexpr int DS4_BT_INPUT_COMMON_OFFSET = 3;

  // BT output "effects" report:
  //   [0x11][0xC0][0x00][31-byte common block][pad][crc32 LE]  (78 bytes)
  constexpr uint8_t DS4_BT_OUTPUT_REPORT_ID = 0x11;
  constexpr int DS4_BT_OUTPUT_LEN = 78;
  constexpr int DS4_BT_OUTPUT_COMMON_OFFSET = 3;

  // Status byte carrying the jack state, at USB common offset 29 (= BT raw
  // offset 32): bits 0x0F battery, 0x10 cable, 0x20 mic, 0x40 headphones.
  constexpr int DS4_STATUS_COMMON_OFFSET = 29;
  constexpr uint8_t DS4_STATUS_HEADPHONES = 0x40;

  // Pure-audio BT report (Layout B, DS4Windows/VIIPER production protocol):
  //   [0x17][0x40][0xA0][ctr LE16][route][436 B SBC][pad][crc32 LE]  (462 bytes)
  // 0x17 carries FOUR 109-byte SBC frames (joint stereo, bitpool 48, 16 blocks,
  // 8 subbands, 32 kHz) = 512 PCM frames = 16 ms per report -> 62.5 reports/s,
  // which fits under the webOS hidraw one-outstanding ceiling (~62/s) that the
  // 8 ms 0x14 form would blow through.
  constexpr uint8_t DS4_BT_AUDIO_REPORT_ID = 0x17;
  constexpr int DS4_0X17_LEN = 462;
  constexpr int DS4_SBC_FRAME_BYTES = 109;
  constexpr int DS4_SBC_FRAMES_PER_REPORT = 4;
  constexpr int DS4_SBC_PCM_PER_FRAME = 128;  // PCM frames per channel per SBC frame
  constexpr int DS4_AUDIO_PCM_PER_REPORT = DS4_SBC_FRAMES_PER_REPORT * DS4_SBC_PCM_PER_FRAME;  // 512
  constexpr int DS4_AUDIO_SBC_BYTES = DS4_SBC_FRAMES_PER_REPORT * DS4_SBC_FRAME_BYTES;         // 436
  constexpr int DS4_AUDIO_HDR_LEN = 6;      // id, hdr, mode, ctr16, route
  constexpr int DS4_PACE_BASE_US = 16000;   // one 512-frame report per 16 ms @32 kHz

  // Route byte values (probed bitmask model, CTM map 2026-07-25): headphones
  // present -> 0xFF (stereo headphones); absent -> 0xDF (split: SBC ch0 ->
  // speaker, ch1 -> headphone-L). The TV-side patch forces 0xFF/0xDF for the
  // user's Headphones/Split modes and passes AUTO through untouched.
  constexpr uint8_t DS4_ROUTE_HEADPHONES = 0xFF;
  constexpr uint8_t DS4_ROUTE_SPLIT = 0xDF;

  /// Sign a DS4 BT output-class report in place: CRC32 over the 0xA2 seed byte
  /// followed by everything but the 4-byte tail, little-endian at the tail.
  inline void ds4_bt_sign(uint8_t *rep, int len) {
    const uint8_t seed = PS_OUTPUT_CRC_SEED;
    uint32_t crc = ds5_crc32_update(0xFFFFFFFFu, &seed, 1);
    crc = ds5_crc32_update(crc, rep, (size_t) (len - 4));
    crc = ~crc;
    rep[len - 4] = (uint8_t) (crc & 0xFF);
    rep[len - 3] = (uint8_t) ((crc >> 8) & 0xFF);
    rep[len - 2] = (uint8_t) ((crc >> 16) & 0xFF);
    rep[len - 1] = (uint8_t) ((crc >> 24) & 0xFF);
  }

  /**
   * @brief Convert a Bluetooth 0x11 input report from the TV into the 64-byte
   *        USB 0x01 report the virtual device feeds the game. Returns false if
   *        the source is too short or not a 0x11 report.
   */
  inline bool ds4_bt_input_to_usb(const uint8_t *bt, size_t bt_len, uint8_t out[DS4_USB_INPUT_LEN]) {
    if (!bt || bt_len < (size_t) (DS4_BT_INPUT_COMMON_OFFSET + DS4_USB_INPUT_LEN - 1) ||
        bt[0] != DS4_BT_INPUT_REPORT_ID) {
      return false;
    }
    out[0] = 0x01;
    std::memcpy(&out[1], &bt[DS4_BT_INPUT_COMMON_OFFSET], DS4_USB_INPUT_LEN - 1);
    return true;
  }

  /**
   * @brief Build a full, CRC-signed Bluetooth 0x11 output report from the
   *        31-byte USB output common block the game wrote.
   */
  inline void ds4_usb_output_to_bt(const uint8_t *common, uint8_t out[DS4_BT_OUTPUT_LEN]) {
    std::memset(out, 0, DS4_BT_OUTPUT_LEN);
    out[0] = DS4_BT_OUTPUT_REPORT_ID;
    out[1] = 0xC0;  // HID + CRC, default poll rate
    out[2] = 0x00;
    std::memcpy(&out[DS4_BT_OUTPUT_COMMON_OFFSET], common, DS4_USB_OUTPUT_COMMON_LEN);
    ds4_bt_sign(out, DS4_BT_OUTPUT_LEN);
  }

  /**
   * @brief Frame one CRC-signed 0x17 pure-audio report around @p sbc
   *        (DS4_AUDIO_SBC_BYTES bytes). @p ctr is the pad's audio frame
   *        counter; it must advance by DS4_SBC_FRAMES_PER_REPORT per report.
   */
  inline void ds4_build_audio_0x17(const uint8_t *sbc, uint16_t ctr, uint8_t route,
                                   uint8_t out[DS4_0X17_LEN]) {
    std::memset(out, 0, DS4_0X17_LEN);
    out[0] = DS4_BT_AUDIO_REPORT_ID;
    out[1] = 0x40;  // pure audio: HID bit OFF, default poll rate
    out[2] = 0xA0;  // speaker path, no mic duplex
    out[3] = (uint8_t) (ctr & 0xFF);
    out[4] = (uint8_t) (ctr >> 8);
    out[5] = route;
    std::memcpy(&out[DS4_AUDIO_HDR_LEN], sbc, DS4_AUDIO_SBC_BYTES);
    ds4_bt_sign(out, DS4_0X17_LEN);
  }

}  // namespace platf::ds5_bridge
