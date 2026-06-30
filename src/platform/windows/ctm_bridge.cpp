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
#include <algorithm>
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

    // Max poll backoff (100ms steps) after consecutive agent launch failures (~60s).
    constexpr int kMaxBackoffSteps = 600;

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
      int fail_streak = 0;  // consecutive genuine launch failures, drives backoff
      while (!st.stop_requested()) {
        int wait_steps = kTickSteps;
        if (config::ctm.enable) {
          auto exe = resolve_exe();
          if (exe && std::filesystem::exists(*exe)) {
            warned_missing = false;
            // Idempotent: no-op while alive, relaunch after a crash.
            const bool allow_system_fallback = platf::is_running_as_system();
            agent_proc().start(exe->wstring(), build_args(), allow_system_fallback);
            // start() returns false BOTH when the agent is already alive (the happy
            // steady state) AND on a genuine launch failure, so discriminate on whether
            // a process actually exists. A real failure (e.g. CreateProcess fails on a
            // present-but-broken binary) gets capped exponential backoff so we don't
            // respawn-spam every 5s — but we never give up, so it self-heals once the
            // dependency is fixed.
            if (agent_proc().get_process_handle() != nullptr) {
              if (fail_streak > 0) {
                BOOST_LOG(info) << "CTM bridge: ctm-usbip.exe running again after "
                                << fail_streak << " failed launch attempt(s).";
              }
              fail_streak = 0;
            } else {
              if (fail_streak == 0) {
                BOOST_LOG(warning) << "CTM bridge: ctm-usbip.exe failed to launch; "
                                      "backing off (will keep retrying).";
              }
              ++fail_streak;
              const int mult = 1 << std::min(fail_streak - 1, 4);  // 5s,10s,20s,40s,... capped
              wait_steps = std::min(kTickSteps * mult, kMaxBackoffSteps);
            }
          } else if (!warned_missing) {
            BOOST_LOG(warning) << "CTM bridge enabled but ctm-usbip.exe not found"
                               << (exe ? (" at: " + platf::to_utf8(exe->wstring())) : std::string {})
                               << ". Set ctm_path or place the binary under the tools/ folder.";
            warned_missing = true;
          }
        } else {
          // Disabled at runtime: ensure no managed instance lingers, and wait for it to
          // actually exit so we don't leave a half-torn-down USB/IP device behind.
          agent_proc().terminate();
          DWORD exit_code = 0;
          agent_proc().wait_for(exit_code, kAgentForceKillWaitMs);
          fail_streak = 0;
        }

        for (int i = 0; i < wait_steps && !st.stop_requested(); ++i) {
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
    // Terminate then wait for full teardown so the USB/IP driver stack is not left
    // half-torn-down and a subsequent start can't overlap a still-exiting instance.
    agent_proc().terminate();
    DWORD exit_code = 0;
    agent_proc().wait_for(exit_code, kAgentForceKillWaitMs);
    BOOST_LOG(info) << "CTM bridge supervisor stopped.";
  }
}  // namespace ctm_bridge
