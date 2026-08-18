/**
 * @file src/ds5_touchpad_mouse.h
 * @brief DualSense touchpad as a desktop mouse.
 *
 * Synthesizes host mouse input from the DS5 touchpad while no game is being
 * streamed, so the pad doubles as a pointing device on the desktop. Games keep
 * the raw touchpad: the native bridge still delivers the untouched report to
 * the virtual pad, and the gate turns synthesis off the moment a real app is
 * launched. Two feed paths cover both transport modes:
 *  - feed_usb_report(): the native DS5 bridge's USB 0x01 input reports
 *    (HID-forwarded pads).
 *  - feed_touch_event(): moonlight controller-touch protocol events
 *    (SDL-mode pads, which otherwise have no touchpad on the emulated pad).
 */
#pragma once

#include <cstddef>
#include <cstdint>

namespace tpmouse {

  /**
   * @brief Feed one full USB 0x01 DualSense input report (64 bytes).
   * Cheap no-op while the gate is inactive. Called from the bridge session
   * thread at report rate.
   */
  void feed_usb_report(const uint8_t *usb, size_t len);

  /**
   * @brief Feed a normalized moonlight controller-touch event.
   * @return true when the event was consumed for mouse synthesis (the caller
   *         should then skip the emulated-pad touch passthrough).
   */
  bool feed_touch_event(uint8_t event_type, uint32_t pointer_id, float x, float y);

  /**
   * @brief Whether touchpad-mouse synthesis is currently in effect
   * (config gate + no game running). Result is cached briefly.
   */
  bool active();

}  // namespace tpmouse
