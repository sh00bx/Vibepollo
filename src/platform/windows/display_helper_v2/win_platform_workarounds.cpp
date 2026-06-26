#include "src/platform/windows/display_helper_v2/win_platform_workarounds.h"

#include <display_device/windows/win_api_layer.h>
#include <display_device/windows/win_display_device.h>
#include <display_device/windows/win_api_utils.h>
#include <display_device/windows/settings_utils.h>

#include <shlobj.h>
#include <windows.h>

#include <thread>

namespace display_helper::v2 {
  namespace {
    void refresh_shell_after_display_change() {
      SHChangeNotify(SHCNE_ASSOCCHANGED, SHCNF_IDLIST | SHCNF_FLUSHNOWAIT, nullptr, nullptr);
      SystemParametersInfoW(SPI_SETICONS, 0, nullptr, SPIF_SENDCHANGE);

      auto broadcast = [](UINT msg, WPARAM wParam, LPARAM lParam) {
        DWORD_PTR result = 0;
        SendMessageTimeoutW(HWND_BROADCAST, msg, wParam, lParam, SMTO_ABORTIFHUNG | SMTO_NORMAL, 100, &result);
      };

      static const wchar_t kShellState[] = L"ShellState";
      static const wchar_t kIconMetrics[] = L"IconMetrics";
      broadcast(WM_SETTINGCHANGE, 0, reinterpret_cast<LPARAM>(kShellState));
      broadcast(WM_SETTINGCHANGE, 0, reinterpret_cast<LPARAM>(kIconMetrics));

      HDC hdc = GetDC(nullptr);
      int bpp = 32;
      if (hdc) {
        const int planes = GetDeviceCaps(hdc, PLANES);
        const int bits = GetDeviceCaps(hdc, BITSPIXEL);
        if (planes > 0 && bits > 0) {
          bpp = planes * bits;
        }
        ReleaseDC(nullptr, hdc);
      }
      const LPARAM res = MAKELPARAM(GetSystemMetrics(SM_CXSCREEN), GetSystemMetrics(SM_CYSCREEN));
      broadcast(WM_DISPLAYCHANGE, static_cast<WPARAM>(bpp), res);
    }
  }  // namespace

  void WinPlatformWorkarounds::blank_hdr_states(std::chrono::milliseconds delay) {
    // Runs SYNCHRONOUSLY (no detached thread): the caller
    // (StateMachine::handle_verification_completed) must finish this HDR
    // off->on "blank" settle BEFORE it releases the capture-start gate, so the
    // toggle can never run under a live encoder. Doing it asynchronously let the
    // gate open first, then this ~1s off->on dropped the virtual display to SDR
    // and back mid-stream, tripping two capture reinits and sending an HDR-mode
    // false->true flip that faults strict HDR decoders (webOS Aurora "decoder
    // reported error" -> disconnect). The work is bounded (DisplayConfig calls +
    // the delay) and the host's capture gate budget (6s) comfortably covers it.
    try {
      auto api = std::make_shared<display_device::WinApiLayer>();
      display_device::WinDisplayDevice display(api);
      display_device::win_utils::blankHdrStates(display, delay);
    } catch (...) {
    }
  }

  void WinPlatformWorkarounds::refresh_shell() {
    refresh_shell_after_display_change();
  }
}  // namespace display_helper::v2
