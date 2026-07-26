/**
 * @file src/platform/windows/game_activity.cpp
 */

#include "game_activity.h"

#include "playnite_integration.h"
#include "src/logging.h"
#include "src/platform/windows/ipc/display_settings_client.h"

#include <algorithm>
#include <atomic>
#include <cctype>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

using namespace std::chrono_literals;

namespace platf::game_activity {
  namespace {
    constexpr auto POLL_INTERVAL = 100ms;
    constexpr auto FOREGROUND_CACHE_LIFETIME = 250ms;
    constexpr auto HEURISTIC_PROMOTION_DELAY = 300ms;
    // Long enough to outlast an alt-tab round trip. Every demotion is a real mode
    // set on the virtual display, and a mode set that lands while the display stack
    // is already churning can stall for seconds.
    constexpr auto DEMOTION_DELAY = 1500ms;
    // Repeated switching costs far more than running at the wrong rate for a few
    // extra seconds, so once the state has flipped this often inside the window,
    // every further change (including the otherwise-immediate promotion) has to
    // prove itself first.
    constexpr auto FLAP_WINDOW = 30s;
    constexpr int FLAP_THRESHOLD = 3;
    constexpr auto FLAP_EXTRA_DELAY = 4s;
    // How long after a mode set the display stack is still considered unsettled.
    constexpr auto DISPLAY_TRANSITION_SETTLE_TIME = 1s;
    constexpr auto RETRY_DELAY = 2s;
    constexpr auto EXPECTED_TRANSITION_LIFETIME = 5s;

    struct cached_foreground_t {
      RECT rect {};
      foreground_app::state_t state;
      std::chrono::steady_clock::time_point sampled_at {};
    };

    std::mutex g_foreground_cache_mutex;
    std::vector<cached_foreground_t> g_foreground_cache;

    // Process-wide because the consumers (capture/encode) never see the per-display
    // refresh target that owns the transition.
    std::atomic<int> g_mode_changes_in_flight {0};
    std::atomic<long long> g_mode_change_settled_at_ms {0};

    std::mutex g_refresh_targets_mutex;
    std::unordered_map<std::string, std::weak_ptr<refresh_target_t>> g_refresh_targets;

    std::string refresh_target_key(std::string device_id) {
      std::ranges::transform(device_id, device_id.begin(), [](const unsigned char value) {
        return static_cast<char>(std::tolower(value));
      });
      return device_id;
    }

    long long steady_now_ms() {
      return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch()
      )
        .count();
    }

    bool same_rect(const RECT &lhs, const RECT &rhs) {
      return lhs.left == rhs.left && lhs.top == rhs.top &&
             lhs.right == rhs.right && lhs.bottom == rhs.bottom;
    }

    void publish_foreground(const RECT &rect, const foreground_app::state_t &state) {
      const auto now = std::chrono::steady_clock::now();
      std::scoped_lock lock {g_foreground_cache_mutex};
      std::erase_if(g_foreground_cache, [&](const auto &entry) {
        return now - entry.sampled_at > EXPECTED_TRANSITION_LIFETIME;
      });
      const auto existing = std::find_if(g_foreground_cache.begin(), g_foreground_cache.end(), [&](const auto &entry) {
        return same_rect(entry.rect, rect);
      });
      if (existing != g_foreground_cache.end()) {
        existing->state = state;
        existing->sampled_at = now;
        return;
      }
      g_foreground_cache.push_back({rect, state, now});
    }

    signal_t foreground_signal(const foreground_app::state_t &foreground) {
      if (foreground.source == "playnite-status") {
        return {signal_source_e::playnite, true, foreground.foreground_pid, foreground.foreground_exe};
      }
      if (foreground.matches_active_app &&
          (foreground.source == "process" || foreground.source == "playnite-cache")) {
        return {signal_source_e::tracked_process, true, foreground.foreground_pid, foreground.foreground_exe};
      }
      if (foreground.valid_window && foreground.fullscreen_on_capture_display) {
        return {signal_source_e::fullscreen_foreground, true, foreground.foreground_pid, foreground.foreground_exe};
      }
      return {};
    }

    signal_t playnite_foreground_signal(
      const foreground_app::state_t &foreground,
      const std::vector<platf::playnite::active_game_status_t> &active_games
    ) {
      if (!foreground.valid_window || foreground.foreground_exe.empty()) {
        return {};
      }

      // Walk newest-first so a recently started second game wins when Playnite has more
      // than one running-game claim. A background claim alone never promotes refresh.
      for (auto game = active_games.rbegin(); game != active_games.rend(); ++game) {
        if (game->active && foreground_app::playnite_foreground_matches_for_tests(
                              {},
                              game->id,
                              game->exe,
                              game->install_dir,
                              foreground.foreground_exe
                            )) {
          return {
            signal_source_e::playnite,
            true,
            foreground.foreground_pid,
            foreground.foreground_exe,
          };
        }
      }
      return {};
    }
  }  // namespace

  state_t reduce_signals(std::span<const signal_t> signals) {
    state_t result;
    for (const auto &signal : signals) {
      if (!signal.active || signal.source <= result.source) {
        continue;
      }
      result.active = true;
      result.source = signal.source;
      result.pid = signal.pid;
      result.executable = signal.executable;
    }
    return result;
  }

  const char *source_name(const signal_source_e source) {
    switch (source) {
      case signal_source_e::playnite:
        return "playnite";
      case signal_source_e::tracked_process:
        return "tracked-process";
      case signal_source_e::fullscreen_foreground:
        return "fullscreen-foreground";
      default:
        return "none";
    }
  }

  foreground_app::state_t foreground_snapshot(const std::optional<RECT> &capture_rect) {
    if (capture_rect) {
      const auto now = std::chrono::steady_clock::now();
      std::scoped_lock lock {g_foreground_cache_mutex};
      const auto cached = std::find_if(g_foreground_cache.begin(), g_foreground_cache.end(), [&](const auto &entry) {
        return same_rect(entry.rect, *capture_rect) && now - entry.sampled_at <= FOREGROUND_CACHE_LIFETIME;
      });
      if (cached != g_foreground_cache.end()) {
        return cached->state;
      }
    }
    return foreground_app::snapshot(capture_rect);
  }

  struct refresh_target_t::impl_t {
    explicit impl_t(refresh_target_options_t options):
        options {std::move(options)},
        applied_high {this->options.initial_high},
        candidate_high {this->options.initial_high},
        candidate_since {std::chrono::steady_clock::now()},
        worker {[this](std::stop_token stop_token) {
          run(stop_token);
        }} {
    }

    ~impl_t() {
      worker.request_stop();
      if (worker.joinable()) {
        worker.join();
      }
    }

    void run(std::stop_token stop_token) {
      SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
      state_t previous_state;
      auto retry_after = std::chrono::steady_clock::time_point {};

      while (!stop_token.stop_requested()) {
        const auto foreground = foreground_app::snapshot(options.capture_rect);
        publish_foreground(options.capture_rect, foreground);

        std::vector<signal_t> signals;
        signals.reserve(3);
        const auto playnite_games = platf::playnite::get_active_game_statuses();
        signals.push_back(playnite_foreground_signal(foreground, playnite_games));
        signals.push_back(foreground_signal(foreground));

        const auto resolved = reduce_signals(signals);
        if (resolved.active != previous_state.active || resolved.source != previous_state.source ||
            resolved.pid != previous_state.pid || resolved.executable != previous_state.executable) {
          BOOST_LOG(debug) << "Game activity: display='" << options.display_name
                           << "' active=" << (resolved.active ? "1" : "0")
                           << " source=" << source_name(resolved.source)
                           << " pid=" << resolved.pid
                           << " exe='" << resolved.executable << "'";
          previous_state = resolved;
        }

        const auto now = std::chrono::steady_clock::now();
        if (resolved.active != candidate_high) {
          candidate_high = resolved.active;
          candidate_since = now;
        }

        if (now - flap_window_start > FLAP_WINDOW) {
          flap_window_start = now;
          flap_count = 0;
        }
        const auto base_delay =
          candidate_high && resolved.source >= signal_source_e::playnite ?
            0ms :
            (candidate_high ? HEURISTIC_PROMOTION_DELAY : DEMOTION_DELAY);
        const auto required_delay =
          flap_count >= FLAP_THRESHOLD ?
            std::chrono::duration_cast<std::chrono::milliseconds>(base_delay + FLAP_EXTRA_DELAY) :
            std::chrono::duration_cast<std::chrono::milliseconds>(base_delay);
        if (candidate_high != applied_high &&
            now - candidate_since >= required_delay &&
            now >= retry_after) {
          const auto numerator = candidate_high ? options.high_refresh_numerator : options.base_refresh_numerator;
          const auto denominator = candidate_high ? options.high_refresh_denominator : options.base_refresh_denominator;
          begin_expected_transition();
          const bool applied = display_helper_client::send_refresh_rate(options.device_id, numerator, denominator);
          finish_expected_transition(applied);
          if (applied) {
            applied_high = candidate_high;
            ++flap_count;
            BOOST_LOG(info) << "Virtual display refresh: display='" << options.display_name
                            << "' source=" << source_name(resolved.source)
                            << " rate=" << numerator << '/' << denominator;
          } else {
            retry_after = now + RETRY_DELAY;
            BOOST_LOG(warning) << "Virtual display refresh: failed to apply " << numerator << '/' << denominator
                               << " to device='" << options.device_id << "'";
          }
        }

        std::this_thread::sleep_for(POLL_INTERVAL);
      }
    }

    void begin_expected_transition() {
      g_mode_changes_in_flight.fetch_add(1, std::memory_order_acq_rel);
      std::scoped_lock lock {transition_mutex};
      transition_expected = true;
      transition_in_progress = true;
      transition_succeeded = false;
      transition_deadline = std::chrono::steady_clock::now() + EXPECTED_TRANSITION_LIFETIME;
    }

    void finish_expected_transition(const bool success) {
      g_mode_change_settled_at_ms.store(
        steady_now_ms() +
          std::chrono::duration_cast<std::chrono::milliseconds>(DISPLAY_TRANSITION_SETTLE_TIME).count(),
        std::memory_order_release
      );
      g_mode_changes_in_flight.fetch_sub(1, std::memory_order_acq_rel);
      {
        std::scoped_lock lock {transition_mutex};
        transition_in_progress = false;
        transition_succeeded = success;
        if (!success) {
          transition_expected = false;
        }
      }
      transition_cv.notify_all();
    }

    bool wait_for_expected_refresh_change(const std::chrono::milliseconds timeout) {
      std::unique_lock lock {transition_mutex};
      if (!transition_expected || std::chrono::steady_clock::now() > transition_deadline) {
        transition_expected = false;
        return false;
      }
      if (transition_in_progress) {
        transition_cv.wait_for(lock, timeout, [&] {
          return !transition_in_progress;
        });
      }
      const bool accepted = transition_expected && !transition_in_progress && transition_succeeded &&
                            std::chrono::steady_clock::now() <= transition_deadline;
      transition_expected = false;
      return accepted;
    }

    refresh_target_options_t options;
    bool applied_high {false};
    bool candidate_high {false};
    std::chrono::steady_clock::time_point candidate_since {};
    std::chrono::steady_clock::time_point flap_window_start {std::chrono::steady_clock::now()};
    int flap_count {0};

    std::mutex transition_mutex;
    std::condition_variable transition_cv;
    bool transition_expected {false};
    bool transition_in_progress {false};
    bool transition_succeeded {false};
    std::chrono::steady_clock::time_point transition_deadline {};

    std::jthread worker;
  };

  refresh_target_t::refresh_target_t(refresh_target_options_t options):
      impl_ {std::make_unique<impl_t>(std::move(options))} {
  }

  refresh_target_t::~refresh_target_t() = default;

  bool refresh_target_t::wait_for_expected_refresh_change(const std::chrono::milliseconds timeout) {
    return impl_ && impl_->wait_for_expected_refresh_change(timeout);
  }

  bool display_mode_change_in_flight() {
    return g_mode_changes_in_flight.load(std::memory_order_acquire) > 0 ||
           steady_now_ms() < g_mode_change_settled_at_ms.load(std::memory_order_acquire);
  }

  std::shared_ptr<refresh_target_t> make_refresh_target(refresh_target_options_t options) {
    if (options.device_id.empty() || options.base_refresh_numerator == 0 ||
        options.base_refresh_denominator == 0 || options.high_refresh_numerator == 0 ||
        options.high_refresh_denominator == 0) {
      return {};
    }

    const auto target_key = refresh_target_key(options.device_id);
    std::scoped_lock lock {g_refresh_targets_mutex};
    std::erase_if(g_refresh_targets, [](const auto &entry) {
      return entry.second.expired();
    });
    if (const auto existing = g_refresh_targets.find(target_key);
        existing != g_refresh_targets.end()) {
      if (auto target = existing->second.lock()) {
        BOOST_LOG(debug) << "Virtual display refresh: display='" << options.display_name
                         << "' reusing active controller for device='" << options.device_id
                         << "'; original stream retains refresh policy ownership";
        return target;
      }
    }

    auto target = std::shared_ptr<refresh_target_t>(new refresh_target_t(std::move(options)));
    g_refresh_targets[target_key] = target;
    return target;
  }

}  // namespace platf::game_activity
