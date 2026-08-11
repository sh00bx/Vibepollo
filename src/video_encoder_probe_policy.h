/**
 * @file src/video_encoder_probe_policy.h
 * @brief Value-level ownership rules for encoder capability probes.
 */
#pragma once

#include <optional>
#include <string>

namespace video::encoder_probe_policy {

  struct cache_key_t {
    std::string encoder_configuration;
    std::string adapter_identity;
    bool adapter_identity_resolved = false;

    bool operator==(const cache_key_t &other) const {
      return encoder_configuration == other.encoder_configuration &&
             adapter_identity == other.adapter_identity &&
             adapter_identity_resolved == other.adapter_identity_resolved;
    }
  };

  struct probe_observation_t {
    std::optional<std::string> required_adapter;
    std::optional<std::string> observed_adapter;
  };

  /**
   * A positive result belongs to the adapter returned by the initialized probe
   * display. A pending or selected adapter may reject a mismatched result, but
   * it cannot replace a missing observation or take ownership of the result.
   */
  inline std::optional<cache_key_t> own_successful_cache_key(
    const cache_key_t &requested_key,
    const probe_observation_t &observation
  ) {
    if (!observation.observed_adapter || observation.observed_adapter->empty()) {
      return std::nullopt;
    }
    if (observation.required_adapter &&
        (*observation.required_adapter != *observation.observed_adapter)) {
      return std::nullopt;
    }

    auto owned_key = requested_key;
    owned_key.adapter_identity = *observation.observed_adapter;
    owned_key.adapter_identity_resolved = true;
    return owned_key;
  }

  template<class adapter_t>
  struct capture_output_t {
    std::string display_name;
    adapter_t adapter;
  };

  template<class adapter_t>
  struct probe_output_choice_t {
    std::string display_name;
    adapter_t adapter;
    bool from_fallback = false;
  };

  /**
   * Choose the output an encoder probe runs on when a virtual display is
   * pending. The adapter hint that scoped the search names the adapter the
   * driver was asked to render on, not the one Windows enumerates the display
   * under, so the scoped search is an optimisation rather than a requirement:
   * when it comes up empty, probing any capture-ready output beats deferring
   * the probe. The fallback then carries its OWN adapter — never the hint —
   * so the post-initialization adapter check downstream agrees with what was
   * actually probed. The fallback search is only performed when the scoped one
   * fails, so a successful scoped probe costs no extra output enumeration.
   */
  template<class adapter_t, class resolve_fallback_output_t>
  std::optional<probe_output_choice_t<adapter_t>> choose_probe_output(
    const std::optional<capture_output_t<adapter_t>> &scoped_output,
    resolve_fallback_output_t &&resolve_fallback_output
  ) {
    if (scoped_output) {
      return probe_output_choice_t<adapter_t> {
        .display_name = scoped_output->display_name,
        .adapter = scoped_output->adapter,
        .from_fallback = false,
      };
    }

    const std::optional<capture_output_t<adapter_t>> fallback_output = resolve_fallback_output();
    if (!fallback_output) {
      return std::nullopt;
    }
    return probe_output_choice_t<adapter_t> {
      .display_name = fallback_output->display_name,
      .adapter = fallback_output->adapter,
      .from_fallback = true,
    };
  }

  inline bool cache_key_matches(
    const cache_key_t &requested_key,
    const std::optional<cache_key_t> &cached_key
  ) {
    return requested_key.adapter_identity_resolved &&
           cached_key && cached_key->adapter_identity_resolved &&
           requested_key == *cached_key;
  }

}  // namespace video::encoder_probe_policy
