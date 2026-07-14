/**
 * @file src/platform/windows/ds5_bridge/vhci_attach.h
 * @brief Drive the usbip-win2 CLI to attach/detach the MS-signed vhci to our
 *        loopback USB/IP server. The vhci binary is redistributed UNCHANGED
 *        (BSD-2-Clause) — we only invoke it, never modify or link it.
 */
#pragma once

#include <string>

namespace platf::ds5_bridge {

  /// Attach 127.0.0.1:<busid> via `usbip attach`. Returns the vhci port number
  /// the device landed on (for a later detach), or -1 on failure. Uses a
  /// port-list snapshot/diff so the exact new port is known even with several
  /// controllers attached.
  int vhci_attach(const std::string &busid);

  /// Detach a previously attached vhci port (`usbip detach -p <port>`).
  void vhci_detach(int port);

}  // namespace platf::ds5_bridge
