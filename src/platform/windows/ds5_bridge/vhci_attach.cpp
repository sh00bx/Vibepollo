/**
 * @file src/platform/windows/ds5_bridge/vhci_attach.cpp
 * @brief usbip-win2 CLI driver (see header).
 */
// clang-format off
#include <windows.h>
// clang-format on

#include <set>
#include <string>

#include "src/logging.h"
#include "src/platform/windows/ds5_bridge/vhci_attach.h"

using namespace std::literals;

namespace platf::ds5_bridge {

  namespace {

    std::string find_usbip_exe() {
      static const char *const candidates[] = {
        "C:\\Program Files\\USBip\\usbip.exe",
        "C:\\Program Files\\usbip-win2\\usbip.exe",
        "C:\\Program Files (x86)\\USBip\\usbip.exe",
      };
      for (auto *c : candidates) {
        if (GetFileAttributesA(c) != INVALID_FILE_ATTRIBUTES) return c;
      }
      return "usbip.exe";  // fall back to PATH
    }

    // Run a command line and capture stdout. Returns exit code (-1 on spawn fail,
    // -2 when the child had to be killed). The CLI normally finishes in ~0.1 s; a
    // wedged usbip.exe (driver stuck after attach, #188) must not hang the bridge.
    int run_capture(const std::string &cmd, std::string &out, DWORD timeout_ms = 5000) {
      out.clear();
      SECURITY_ATTRIBUTES sa {};
      sa.nLength = sizeof(sa);
      sa.bInheritHandle = TRUE;
      HANDLE rd = nullptr, wr = nullptr;
      if (!CreatePipe(&rd, &wr, &sa, 0)) return -1;
      SetHandleInformation(rd, HANDLE_FLAG_INHERIT, 0);

      STARTUPINFOA si {};
      si.cb = sizeof(si);
      si.dwFlags = STARTF_USESTDHANDLES;
      si.hStdOutput = wr;
      si.hStdError = wr;
      si.hStdInput = INVALID_HANDLE_VALUE;
      PROCESS_INFORMATION pi {};
      std::string mutable_cmd = cmd;
      mutable_cmd.push_back('\0');
      if (!CreateProcessA(nullptr, mutable_cmd.data(), nullptr, nullptr, TRUE,
                          CREATE_NO_WINDOW, nullptr, nullptr, &si, &pi)) {
        CloseHandle(rd);
        CloseHandle(wr);
        return -1;
      }
      CloseHandle(wr);  // close our write end so the pipe breaks at child exit

      // Poll instead of a blocking ReadFile, which would wait forever on a hung child.
      const ULONGLONG deadline = GetTickCount64() + timeout_ms;
      char buf[512];
      for (;;) {
        DWORD avail = 0;
        if (!PeekNamedPipe(rd, nullptr, 0, nullptr, &avail, nullptr)) break;  // broken pipe = EOF
        if (avail > 0) {
          DWORD n = 0;
          if (!ReadFile(rd, buf, avail < sizeof(buf) ? avail : (DWORD) sizeof(buf), &n, nullptr) || n == 0) break;
          out.append(buf, n);
          continue;
        }
        // Exited but a grandchild still holds the write end: take what the child wrote
        // before exiting (it may have landed after the peek above), then stop.
        if (WaitForSingleObject(pi.hProcess, 0) == WAIT_OBJECT_0) {
          while (PeekNamedPipe(rd, nullptr, 0, nullptr, &avail, nullptr) && avail > 0) {
            DWORD n = 0;
            if (!ReadFile(rd, buf, avail < sizeof(buf) ? avail : (DWORD) sizeof(buf), &n, nullptr) || n == 0) break;
            out.append(buf, n);
          }
          break;
        }
        if (GetTickCount64() >= deadline) break;
        Sleep(10);
      }
      CloseHandle(rd);

      const ULONGLONG now = GetTickCount64();
      const DWORD remaining = now < deadline ? (DWORD) (deadline - now) : 0;
      int rc;
      if (WaitForSingleObject(pi.hProcess, remaining) == WAIT_OBJECT_0) {
        DWORD code = 0;
        GetExitCodeProcess(pi.hProcess, &code);
        rc = (int) code;
      } else {
        BOOST_LOG(warning) << "ds5-bridge: '"sv << cmd << "' still running after "sv << timeout_ms
                           << " ms; killing it (output so far: "sv << out << ")"sv;
        TerminateProcess(pi.hProcess, 1);
        WaitForSingleObject(pi.hProcess, 1000);
        rc = -2;
      }
      CloseHandle(pi.hProcess);
      CloseHandle(pi.hThread);
      return rc;
    }

    // Parse the "Port NN:" numbers currently listed by `usbip port`. False when the
    // listing itself failed: an empty set would then be a guess, not a snapshot.
    bool list_ports(const std::string &exe, std::set<int> &ports) {
      ports.clear();
      std::string out;
      if (run_capture("\"" + exe + "\" port", out) != 0) return false;
      size_t pos = 0;
      while ((pos = out.find("Port ", pos)) != std::string::npos) {
        pos += 5;
        int v = 0;
        bool any = false;
        while (pos < out.size() && out[pos] >= '0' && out[pos] <= '9') {
          v = v * 10 + (out[pos] - '0');
          any = true;
          ++pos;
        }
        if (any) ports.insert(v);
      }
      return true;
    }

  }  // namespace

  int vhci_attach(const std::string &busid) {
    const std::string exe = find_usbip_exe();
    std::set<int> before, after;
    // Without a reliable before/after pair the new port could be another pad's,
    // and teardown would detach that one.
    const bool have_before = list_ports(exe, before);
    std::string out;
    std::string cmd = "\"" + exe + "\" attach -r 127.0.0.1 -b " + busid;
    int rc = run_capture(cmd, out);
    if (rc != 0) {
      BOOST_LOG(warning) << "ds5-bridge: usbip attach -b "sv << busid
                         << " failed (rc="sv << rc << "): "sv << out;
      return -1;
    }
    if (!have_before || !list_ports(exe, after)) {
      BOOST_LOG(warning) << "ds5-bridge: vhci attach busid="sv << busid
                         << " ok but the port listing failed (detach will be manual)"sv;
      return -1;
    }
    for (int p : after) {
      if (!before.count(p)) {
        BOOST_LOG(info) << "ds5-bridge: vhci attached busid="sv << busid << " -> port "sv << p;
        return p;
      }
    }
    BOOST_LOG(info) << "ds5-bridge: vhci attach busid="sv << busid
                    << " ok but new port not identified (detach will be manual)"sv;
    return -1;
  }

  void vhci_detach(int port) {
    if (port < 0) return;
    const std::string exe = find_usbip_exe();
    std::string out;
    std::string cmd = "\"" + exe + "\" detach -p " + std::to_string(port);
    int rc = run_capture(cmd, out);
    if (rc != 0) {
      BOOST_LOG(warning) << "ds5-bridge: usbip detach -p "sv << port << " failed (rc="sv << rc << ")"sv;
    } else {
      BOOST_LOG(info) << "ds5-bridge: vhci detached port "sv << port;
    }
  }

}  // namespace platf::ds5_bridge
