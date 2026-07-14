/**
 * @file src/platform/windows/ds5_bridge/ds5_bridge.h
 * @brief Service-lifetime supervisor for the native in-process DS5 bridge
 *        provider (the license-clean replacement for the external
 *        ctm-usbip.exe agent).
 *
 * Mirrors ctm_bridge: a single jthread that, while config::ds5b.native_bridge is
 * enabled, keeps the in-process bridge_host running, and otherwise stops it. The
 * native provider and the CTM bridge are mutually exclusive (both bind the same
 * usbip loopback + control port); when both are enabled the native provider
 * yields to CTM with a warning, so a stale config can never silently double-bind.
 */
#pragma once

namespace ds5_bridge_provider {
  void start_watchdog();
  void stop_watchdog();
}  // namespace ds5_bridge_provider
