/**
 * @file src/platform/windows/ctm_bridge.cpp
 * @brief Service-lifetime supervisor for the external CTM bridge agent (ctm-usbip.exe).
 *
 * Vibepollo launches and keeps alive the upstream CTM-USBIP agent so it no longer
 * needs a separate Windows autostart (the old `ctmagent` scheduled task). The agent
 * binary is treated as an opaque upstream artifact: no CTM source is vendored, so
 * upstream updates are a drop-in replacement of ctm-usbip.exe. All Vibepollo-side
 * logic is confined to this file plus the config plumbing (config::ctm) and the web
 * UI, which keeps the integration atomic and easy to maintain against upstream.
 *
 * The supervisor mirrors the display-helper watchdog: a single jthread that, while
 * the feature is enabled, (re)starts the agent via ProcessHandler and otherwise
 * terminates it. ProcessHandler::start() is idempotent (it no-ops while the child is
 * alive and relaunches once it exits), so a periodic tick doubles as crash-restart.
 */

// standard includes
#include <chrono>
#include <filesystem>
#include <mutex>
#include <optional>
#include <stop_token>
#include <string>
#include <thread>
#include <vector>

// platform includes
#include <windows.h>
#include <tlhelp32.h>

// local includes
#include "src/config.h"
#include "src/logging.h"
#include "src/platform/windows/ctm_bridge.h"
#include "src/platform/windows/ipc/process_handler.h"
#include "src/platform/windows/misc.h"

namespace ctm_bridge {
  namespace {
    std::mutex g_mutex;
    std::jthread g_thread;
    bool g_running = false;

    // Poll interval for the supervisor loop, in 100ms steps (5s total).
    constexpr int kTickSteps = 50;

    // How long to wait for an orphaned agent to exit after TerminateProcess.
    constexpr DWORD kAgentForceKillWaitMs = 2000;

    /**
     * @brief The single tracked agent instance.
     *
     * use_job=false mirrors the display helper: lifetime is managed explicitly via
     * terminate(), and launching into the active console session via
     * CreateProcessAsUserW (when Sunshine runs as SYSTEM) does not compose cleanly
     * with a kill-on-close job.
     */
    ProcessHandler &agent_proc() {
      static ProcessHandler h(/*use_job=*/false);
      return h;
    }

    /**
     * @brief Resolve the ctm-usbip.exe path: explicit config override, else
     *        "<install>/tools/ctm-usbip.exe" next to the running executable.
     */
    std::optional<std::filesystem::path> resolve_exe() {
      if (!config::ctm.exe_path.empty()) {
        return std::filesystem::path(platf::from_utf8(config::ctm.exe_path));
      }
      wchar_t module_path[MAX_PATH] = {};
      if (!GetModuleFileNameW(nullptr, module_path, _countof(module_path))) {
        return std::nullopt;
      }
      // The agent ships as a self-contained folder (its own ffmpeg DLLs + maps/
      // profiles/), so it lives in its own tools/ctm subdir; ProcessHandler runs it
      // with that folder as the working directory.
      return std::filesystem::path(module_path).parent_path() / L"tools" / L"ctm" / L"ctm-usbip.exe";
    }

    /**
     * @brief Build the agent command line: "agent <port> [--enet]".
     */
    std::wstring build_args() {
      std::wstring args = L"agent " + std::to_wstring(config::ctm.port);
      if (config::ctm.enet) {
        args += L" --enet";
      }
      return args;
    }

    /**
     * @brief Terminate any ctm-usbip.exe instance not managed by this process.
     *
     * Mirrors display_helper_integration::kill_all_helper_processes. The agent is
     * launched jobless (use_job=false -> CREATE_BREAKAWAY_FROM_JOB) and is only
     * cleaned up on the graceful shutdown path (stop_watchdog), so a crashed or
     * force-killed sunshine.exe orphans it. On the next launch a fresh static
     * ProcessHandler has no knowledge of that orphan and start()s a SECOND
     * ctm-usbip.exe; the two collide on the same USB/IP port and controller
     * passthrough silently dies until a manual taskkill. Vibepollo is the sole
     * supervisor of this binary (it replaced the old `ctmagent` autostart), so
     * reaping every instance before we launch our own enforces the singleton.
     *
     * Called once at supervisor start, before any owned instance exists, so there
     * is nothing of ours to spare; any live ctm-usbip.exe at that point is an
     * orphan from a prior run.
     */
    void reap_orphan_agents() {
      HANDLE snapshot = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
      if (snapshot == INVALID_HANDLE_VALUE) {
        DWORD err = GetLastError();
        BOOST_LOG(error) << "CTM bridge: failed to snapshot processes for orphan cleanup (winerr=" << err << ").";
        return;
      }

      PROCESSENTRY32W entry {};
      entry.dwSize = sizeof(entry);
      std::vector<DWORD> targets;

      if (Process32FirstW(snapshot, &entry)) {
        do {
          if (_wcsicmp(entry.szExeFile, L"ctm-usbip.exe") == 0) {
            targets.push_back(entry.th32ProcessID);
          }
        } while (Process32NextW(snapshot, &entry));
      } else {
        DWORD err = GetLastError();
        if (err != ERROR_NO_MORE_FILES) {
          BOOST_LOG(warning) << "CTM bridge: process enumeration failed during orphan cleanup (winerr=" << err << ").";
        }
      }

      CloseHandle(snapshot);

      for (DWORD pid : targets) {
        HANDLE h = OpenProcess(PROCESS_TERMINATE | SYNCHRONIZE, FALSE, pid);
        if (!h) {
          DWORD err = GetLastError();
          BOOST_LOG(warning) << "CTM bridge: unable to open orphaned agent (pid=" << pid
                             << ", winerr=" << err << ") for termination.";
          continue;
        }

        if (WaitForSingleObject(h, 0) == WAIT_TIMEOUT) {
          BOOST_LOG(warning) << "CTM bridge: terminating orphaned ctm-usbip.exe (pid=" << pid << ").";
          if (!TerminateProcess(h, 1)) {
            DWORD err = GetLastError();
            BOOST_LOG(error) << "CTM bridge: TerminateProcess failed for pid=" << pid << " (winerr=" << err << ").";
          } else if (WaitForSingleObject(h, kAgentForceKillWaitMs) != WAIT_OBJECT_0) {
            BOOST_LOG(warning) << "CTM bridge: orphaned agent pid=" << pid
                               << " did not exit within " << kAgentForceKillWaitMs << " ms.";
          }
        }

        CloseHandle(h);
      }
    }

    void watchdog_proc(std::stop_token st) {
      using namespace std::chrono_literals;
      bool warned_missing = false;
      while (!st.stop_requested()) {
        if (config::ctm.enable) {
          auto exe = resolve_exe();
          if (exe && std::filesystem::exists(*exe)) {
            warned_missing = false;
            // Idempotent: no-op while alive, relaunch after a crash.
            const bool allow_system_fallback = platf::is_running_as_system();
            agent_proc().start(exe->wstring(), build_args(), allow_system_fallback);
          } else if (!warned_missing) {
            BOOST_LOG(warning) << "CTM bridge enabled but ctm-usbip.exe not found"
                               << (exe ? (" at: " + platf::to_utf8(exe->wstring())) : std::string {})
                               << ". Set ctm_path or place the binary under the tools/ folder.";
            warned_missing = true;
          }
        } else {
          // Disabled at runtime: ensure no managed instance lingers.
          agent_proc().terminate();
        }

        for (int i = 0; i < kTickSteps && !st.stop_requested(); ++i) {
          std::this_thread::sleep_for(100ms);
        }
      }
    }
  }  // namespace

  void start_watchdog() {
    std::lock_guard lk(g_mutex);
    if (g_running) {
      return;
    }
    // Reap any agent orphaned by a previous unclean shutdown before launching our
    // own, otherwise the new instance collides with the orphan on the USB/IP port.
    reap_orphan_agents();
    g_running = true;
    g_thread = std::jthread(watchdog_proc);
    BOOST_LOG(info) << "CTM bridge supervisor started.";
  }

  void stop_watchdog() {
    std::jthread local;
    {
      std::lock_guard lk(g_mutex);
      if (!g_running) {
        return;
      }
      g_running = false;
      local = std::move(g_thread);
    }
    local.request_stop();
    if (local.joinable()) {
      local.join();
    }
    agent_proc().terminate();
    BOOST_LOG(info) << "CTM bridge supervisor stopped.";
  }
}  // namespace ctm_bridge
