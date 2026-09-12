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
      bool started = false;
      while (!st.stop_requested()) {
        bool enable, haptics, audio_batched, audio_cancel_bits, haptics_handback, mic;
        int audio_cushion;
        int port;
        std::string lightbar;
        {
          std::lock_guard<std::mutex> lk(config::ds5b_mutex);
          enable = config::ds5b.native_bridge;
          port = config::ds5b.port;
          haptics = config::ds5b.native_haptics;
          audio_batched = config::ds5b.native_audio_batched;
          audio_cushion = config::ds5b.native_audio_cushion_frames;
          audio_cancel_bits = config::ds5b.native_audio_cancel_bits;
          haptics_handback = config::ds5b.native_haptics_handback;
          mic = config::ds5b.native_mic;
          lightbar = config::ds5b.lightbar_color;
        }

        // The native provider is the only owner of the usbip loopback
        // (127.0.0.1:3240) and the control port; the external CTM agent it
        // replaced is no longer part of the tree.
        bool want = enable;

        host().set_haptics(haptics);
        host().set_audio_batched(audio_batched);
        host().set_audio_cushion(audio_cushion);
        host().set_audio_cancel_bits(audio_cancel_bits);
        host().set_haptics_handback(haptics_handback);
        host().set_mic(mic);
        host().set_lightbar(parse_lightbar(lightbar));
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
