/**
 * @file src/platform/windows/ds5_bridge/ds5_bridge.cpp
 * @brief Native DS5 bridge provider supervisor (see header).
 */
#include <atomic>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <mutex>
#include <stop_token>
#include <string>
#include <thread>

#include "src/config.h"
#include "src/logging.h"
#include "src/platform/windows/ds5_bridge/bridge_host.h"
#include "src/platform/windows/ds5_bridge/ds5_bridge.h"

namespace ds5_bridge_provider {
  namespace {
    std::mutex g_mutex;
    std::jthread g_thread;
    bool g_running = false;

    // Poll interval for the supervisor loop, in 100ms steps (5s total).
    constexpr int kTickSteps = 50;

    /// Parse the ds5_lightbar_color config value ("RRGGBB", optional '#';
    /// empty/"off"/"none"/"disabled" or anything malformed disables the synth).
    uint32_t parse_lightbar(const std::string &raw) {
      std::string h;
      for (char c : raw) {
        if (!std::isspace((unsigned char) c) && c != '#') {
          h += (char) std::tolower((unsigned char) c);
        }
      }
      if (h.empty() || h == "off" || h == "none" || h == "disabled") {
        return platf::ds5_bridge::LIGHTBAR_OFF;
      }
      if (h.size() != 6 || h.find_first_not_of("0123456789abcdef") != std::string::npos) {
        return platf::ds5_bridge::LIGHTBAR_OFF;
      }
      return (uint32_t) std::strtoul(h.c_str(), nullptr, 16);
    }

    platf::ds5_bridge::bridge_host &host() {
      static platf::ds5_bridge::bridge_host h;
      return h;
    }

    void watchdog_proc(std::stop_token st) {
      using namespace std::chrono_literals;
      bool warned_conflict = false;
      bool started = false;
      while (!st.stop_requested()) {
        bool enable, ctm_enable, haptics;
        int port;
        std::string lightbar;
        uint32_t kick = 0;
        {
          std::lock_guard<std::mutex> lk(config::ds5b_mutex);
          enable = config::ds5b.native_bridge;
          port = config::ds5b.port;
          haptics = config::ds5b.native_haptics;
          lightbar = config::ds5b.lightbar_color;
          // Trigger-kick config, packed for the sessions' lock-free live read:
          // bit0 enable, bit1 R2, bit2 L2, bit3 haptic source, bit4 rumble
          // source, bits 8-15 carrier freq, bits 16-23 strength.
          if (config::ds5b.trigger_kick) {
            const auto &side = config::ds5b.trigger_kick_side;
            const auto &src = config::ds5b.trigger_kick_source;
            kick = 1u;
            kick |= (side == "l2") ? 4u : (side == "both") ? 6u : 2u;
            kick |= (src == "haptic") ? 8u : (src == "rumble") ? 16u : 24u;
            kick |= (uint32_t) (config::ds5b.trigger_kick_freq & 0xFF) << 8;
            kick |= (uint32_t) (config::ds5b.trigger_kick_strength & 0xFF) << 16;
          }
        }
        {
          std::lock_guard<std::mutex> lk(config::ctm_mutex);
          ctm_enable = config::ctm.enable;
        }

        // Mutual exclusion: the CTM agent and the native provider both bind the
        // usbip loopback (127.0.0.1:3240) and the control port, so only one may
        // run. CTM wins if both are enabled (explicit, logged), so an operator
        // toggling native on without turning CTM off never silently collides.
        bool want = enable && !ctm_enable;
        if (enable && ctm_enable) {
          if (!warned_conflict) {
            BOOST_LOG(warning) << "DS5 native bridge and CTM bridge are both enabled; "
                                  "the native provider yields to CTM. Disable ctm_enable to use it.";
            warned_conflict = true;
          }
        } else {
          warned_conflict = false;
        }

        host().set_haptics(haptics);
        host().set_lightbar(parse_lightbar(lightbar));
        host().set_trigger_kick(kick);
        if (want && !started) {
          started = host().start(port);
        } else if (!want && started) {
          host().stop();
          started = false;
        }

        for (int i = 0; i < kTickSteps && !st.stop_requested(); ++i) {
          std::this_thread::sleep_for(100ms);
        }
      }
      if (started) host().stop();
    }
  }  // namespace

  void start_watchdog() {
    std::lock_guard lk(g_mutex);
    if (g_running) return;
    g_running = true;
    g_thread = std::jthread(watchdog_proc);
    BOOST_LOG(info) << "DS5 native bridge supervisor started.";
  }

  void stop_watchdog() {
    std::jthread local;
    {
      std::lock_guard lk(g_mutex);
      if (!g_running) return;
      g_running = false;
      local = std::move(g_thread);
    }
    local.request_stop();
    if (local.joinable()) local.join();
    BOOST_LOG(info) << "DS5 native bridge supervisor stopped.";
  }
}  // namespace ds5_bridge_provider
