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
   *
   * The gesture state is single: one source (a distinct non-zero id per
   * feeder, e.g. the session object) drives the mouse at a time. A touching
   * source takes over once the current owner is fully idle; everything else
   * is ignored so an idle second pad cannot corrupt the owner's gesture.
   *
   * @return true when the mouse consumed this pad's touch (the caller should
   *         then lift the touch contacts and the touchpad click off the copy
   *         it hands the virtual pad, so nothing reads the finger twice).
   */
  bool feed_usb_report(uintptr_t source, const uint8_t *usb, size_t len);

  /**
   * @brief Same, for a DualShock 4's USB 0x01 report.
   *
   * A separate entry point rather than a sniffed one: both pads use report id
   * 0x01 with the same length, and the touch block, the click bit and the pad
   * clock all sit at different offsets with different units -- reading a DS4
   * report with the DualSense map yields plausible garbage, not a failure. The
   * caller knows which pad it bridged, so it says so.
   *
   * @return true when the mouse consumed this pad's touch; the caller then
   *         lifts contacts (payload 34 and 38) and the click (payload byte 6,
   *         bit 0x02) off the copy the virtual pad sees.
   */
  bool feed_ds4_usb_report(uintptr_t source, const uint8_t *usb, size_t len);

  /**
   * @brief Feed a normalized moonlight controller-touch event.
   * Source semantics as in feed_usb_report().
   * @return true when the event was consumed for mouse synthesis (the caller
   *         should then skip the emulated-pad touch passthrough).
   */
  bool feed_touch_event(uintptr_t source, uint8_t event_type, uint32_t pointer_id, float x, float y);

  /**
   * @brief Whether touchpad-mouse synthesis is currently in effect
   * (config gate + no game running). Result is cached briefly.
   */
  bool active();

  /**
   * @brief The streaming client's preference (CTMB_MSG_TPMOUSE): 0=off,
   * 1=auto, 2=always, -1 clears back to the ds5_touchpad_mouse config value.
   * The client re-asserts it every session, so it is never persisted here.
   */
  void set_client_mode(int mode);

  /**
   * @brief Drop the gesture state and release any held synthesized button,
   * if @p source currently owns the gesture (cheap no-op otherwise).
   * Must be called when a feed path dies (bridge link drop or teardown,
   * stream session end, pad removal, mouse permission revoked): the state
   * machine otherwise only advances on the next report, which may never
   * come. Only the id is compared -- the call is safe with a dying source.
   */
  void reset(uintptr_t source);

}  // namespace tpmouse
