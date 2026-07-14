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

    // Run a command line and capture stdout. Returns exit code (-1 on spawn fail).
    int run_capture(const std::string &cmd, std::string &out) {
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
      CloseHandle(wr);  // close our write end so ReadFile sees EOF at child exit
      char buf[512];
      DWORD n = 0;
      while (ReadFile(rd, buf, sizeof(buf), &n, nullptr) && n > 0) {
        out.append(buf, n);
      }
      CloseHandle(rd);
      WaitForSingleObject(pi.hProcess, 8000);
      DWORD code = 0;
      GetExitCodeProcess(pi.hProcess, &code);
      CloseHandle(pi.hProcess);
      CloseHandle(pi.hThread);
      return (int) code;
    }

    // Parse the "Port NN:" numbers currently listed by `usbip port`.
    std::set<int> list_ports(const std::string &exe) {
      std::set<int> ports;
      std::string out;
      run_capture("\"" + exe + "\" port", out);
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
      return ports;
    }

  }  // namespace

  int vhci_attach(const std::string &busid) {
    const std::string exe = find_usbip_exe();
    auto before = list_ports(exe);
    std::string out;
    std::string cmd = "\"" + exe + "\" attach -r 127.0.0.1 -b " + busid;
    int rc = run_capture(cmd, out);
    if (rc != 0) {
      BOOST_LOG(warning) << "ds5-bridge: usbip attach -b "sv << busid
                         << " failed (rc="sv << rc << "): "sv << out;
      return -1;
    }
    auto after = list_ports(exe);
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
