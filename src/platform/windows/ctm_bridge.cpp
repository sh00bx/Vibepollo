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

    // An agent that exits sooner than this after launch is treated as a launch failure
    // (crash loop), so the same backoff applies instead of respawning every tick.
    constexpr auto kAgentStableWindow = std::chrono::seconds(30);

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
    std::optional<std::filesystem::path> resolve_exe(const std::string &exe_path) {
      if (!exe_path.empty()) {
        return std::filesystem::path(platf::from_utf8(exe_path));
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
    std::wstring build_args(int port, bool enet) {
      std::wstring args = L"agent " + std::to_wstring(port);
      if (enet) {
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
     * Called on the supervisor's first enabled tick, and again after each runtime
     * disable->enable toggle, before any owned instance exists, so there is nothing of
     * ours to spare; any live ctm-usbip.exe at that point is an orphan from a prior run.
     * It is never called while the feature is disabled, so an externally-managed
     * ctm-usbip.exe is left alone when ctm.enable is false.
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
        HANDLE h = OpenProcess(PROCESS_TERMINATE | SYNCHRONIZE | PROCESS_QUERY_LIMITED_INFORMATION, FALSE, pid);
        if (!h) {
          DWORD err = GetLastError();
          BOOST_LOG(warning) << "CTM bridge: unable to open orphaned agent (pid=" << pid
                             << ", winerr=" << err << ") for termination.";
          continue;
        }

        // Re-verify the image name on the opened handle before terminating. The snapshot
        // above is a point-in-time list of PIDs; PIDs are recycled and Sunshine runs as
        // SYSTEM, so between snapshot and here this PID may already belong to an unrelated
        // (possibly critical) process. Guard against that TOCTOU race.
        {
          wchar_t image_path[MAX_PATH];
          DWORD image_len = _countof(image_path);
          if (!QueryFullProcessImageNameW(h, 0, image_path, &image_len)) {
            BOOST_LOG(warning) << "CTM bridge: could not verify image name for pid=" << pid
                               << " (winerr=" << GetLastError() << "); skipping termination.";
            CloseHandle(h);
            continue;
          }
          const wchar_t *leaf = image_path;
          for (DWORD i = 0; i < image_len; ++i) {
            if (image_path[i] == L'\\' || image_path[i] == L'/') {
              leaf = image_path + i + 1;
            }
          }
          if (_wcsicmp(leaf, L"ctm-usbip.exe") != 0) {
            BOOST_LOG(warning) << "CTM bridge: pid=" << pid << " is no longer ctm-usbip.exe (now "
                               << platf::to_utf8(image_path) << "); skipping (PID recycled).";
            CloseHandle(h);
            continue;
          }
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
      int fail_streak = 0;  // consecutive launch failures (incl. crash-loops), drives backoff
      bool need_reap = true;  // reap orphans on the first enabled tick + after each disable->enable
      std::optional<std::chrono::steady_clock::time_point> launch_time;  // set of the last fresh launch
      while (!st.stop_requested()) {
        // Snapshot the config this tick needs under the lock: config::ctm (incl. a
        // std::string) is rewritten by apply_config on the confighttp hot-reload thread,
        // so an unsynchronized read here would be a torn read (UB).
        bool enable;
        std::string exe_path;
        int port;
        bool enet;
        {
          std::lock_guard<std::mutex> lk(config::ctm_mutex);
          enable = config::ctm.enable;
          exe_path = config::ctm.exe_path;
          port = config::ctm.port;
          enet = config::ctm.enet;
        }

        int wait_steps = kTickSteps;
        if (enable) {
          // Reap orphans only while enabled (never disturb an externally-managed agent
          // when the feature is off) and again on each disable->enable toggle, so the
          // port-collision singleton guard is re-armed on re-enable.
          if (need_reap) {
            reap_orphan_agents();
            need_reap = false;
          }
          auto exe = resolve_exe(exe_path);
          if (exe && std::filesystem::exists(*exe)) {
            warned_missing = false;

            // Crash-loop guard: if the agent we launched has already exited, decide
            // whether that counts as a failure (exited within the stability window)
            // BEFORE relaunching. Otherwise a binary that launches then immediately dies
            // (missing DLL, bound USB/IP port) respawns every tick forever with only a
            // debug log and never backs off.
            HANDLE cur = agent_proc().get_process_handle();
            if (cur != nullptr && launch_time) {
              if (WaitForSingleObject(cur, 0) == WAIT_OBJECT_0) {
                DWORD code = 0;
                GetExitCodeProcess(cur, &code);
                const auto alive = std::chrono::steady_clock::now() - *launch_time;
                if (alive < kAgentStableWindow) {
                  if (fail_streak == 0) {
                    BOOST_LOG(warning) << "CTM bridge: ctm-usbip.exe exited after "
                                       << std::chrono::duration_cast<std::chrono::seconds>(alive).count()
                                       << "s (exit code " << code << "); backing off (will keep retrying).";
                  }
                  ++fail_streak;
                }
                launch_time.reset();
              } else if (fail_streak > 0 &&
                         std::chrono::steady_clock::now() - *launch_time >= kAgentStableWindow) {
                // The relaunched agent has now survived the stability window; clear backoff.
                BOOST_LOG(info) << "CTM bridge: ctm-usbip.exe stable again after "
                                << fail_streak << " failed launch attempt(s).";
                fail_streak = 0;
              }
            }

            // Idempotent: no-op while alive, relaunch after a crash. start() returns
            // true only on an actual fresh launch.
            const bool allow_system_fallback = platf::is_running_as_system();
            if (agent_proc().start(exe->wstring(), build_args(port, enet), allow_system_fallback)) {
              launch_time = std::chrono::steady_clock::now();
            }

            // A genuine CreateProcess failure leaves no live handle (present-but-broken
            // binary). Cap exponential backoff so we don't respawn-spam, but never give
            // up, so it self-heals once the dependency is fixed.
            if (agent_proc().get_process_handle() == nullptr) {
              if (fail_streak == 0) {
                BOOST_LOG(warning) << "CTM bridge: ctm-usbip.exe failed to launch; "
                                      "backing off (will keep retrying).";
              }
              ++fail_streak;
              launch_time.reset();
            }

            if (fail_streak > 0) {
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
          launch_time.reset();
          need_reap = true;  // a subsequent re-enable should reap any orphan first
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
    // Orphan reaping is deferred to the supervisor's first ENABLED tick (see
    // watchdog_proc): reaping here would kill an externally-managed ctm-usbip.exe even
    // when the feature is disabled, and would not re-run on a runtime disable->enable
    // toggle.
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
