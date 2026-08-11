#include "../tests_common.h"
#include "src/video_encoder_probe_policy.h"

#include <optional>
#include <string>
#include <utility>

namespace {
  using video::encoder_probe_policy::cache_key_matches;
  using video::encoder_probe_policy::cache_key_t;
  using video::encoder_probe_policy::choose_probe_output;
  using video::encoder_probe_policy::own_successful_cache_key;
  using video::encoder_probe_policy::probe_observation_t;

  cache_key_t key(std::string adapter) {
    return {
      .encoder_configuration = "encoder=nvenc|hevc=auto|av1=auto",
      .adapter_identity = std::move(adapter),
      .adapter_identity_resolved = true,
    };
  }

  // The probe output policy is adapter-type agnostic; the identity string
  // stands in for platf::adapter_id_t here.
  using adapter_t = std::string;
  using capture_output_t = video::encoder_probe_policy::capture_output_t<adapter_t>;

  std::optional<capture_output_t> output(std::string display_name, adapter_t adapter) {
    return capture_output_t {std::move(display_name), std::move(adapter)};
  }

  auto no_output() {
    return []() {
      return std::optional<capture_output_t> {std::nullopt};
    };
  }
}  // namespace

TEST(EncoderProbePolicy, MatchingObservedAdapterReusesItsCacheEntry) {
  const auto pending_key = key("luid=nvidia");
  const auto owned_key = own_successful_cache_key(
    pending_key,
    probe_observation_t {
      .required_adapter = "luid=nvidia",
      .observed_adapter = "luid=nvidia",
    }
  );

  ASSERT_TRUE(owned_key);
  EXPECT_TRUE(cache_key_matches(key("luid=nvidia"), owned_key));
}

TEST(EncoderProbePolicy, DifferentObservedAdapterMisses) {
  const auto cached_key = std::optional<cache_key_t> {key("luid=amd")};

  EXPECT_FALSE(cache_key_matches(key("luid=nvidia"), cached_key));
}

TEST(EncoderProbePolicy, PendingHintCannotOwnMismatchedObservedProbe) {
  const auto pending_key = key("luid=nvidia");
  const auto owned_key = own_successful_cache_key(
    pending_key,
    probe_observation_t {
      .required_adapter = "luid=nvidia",
      .observed_adapter = "luid=amd",
    }
  );

  EXPECT_FALSE(owned_key);
  EXPECT_FALSE(cache_key_matches(pending_key, owned_key));
}

TEST(EncoderProbePolicy, UnresolvedProbeCannotPublishPositiveEntry) {
  const auto pending_key = key("luid=nvidia");

  EXPECT_FALSE(own_successful_cache_key(
    pending_key,
    probe_observation_t {
      .required_adapter = "luid=nvidia",
      .observed_adapter = std::nullopt,
    }
  ));
  EXPECT_FALSE(own_successful_cache_key(
    pending_key,
    probe_observation_t {
      .required_adapter = std::nullopt,
      .observed_adapter = std::string {},
    }
  ));
}

TEST(EncoderProbePolicy, CapabilitiesRemainAssociatedWithObservedKey) {
  struct cached_capabilities_t {
    int hevc_mode;
    bool hdr;
  };

  const auto owned_key = own_successful_cache_key(
    key("luid=nvidia"),
    probe_observation_t {
      .required_adapter = "luid=nvidia",
      .observed_adapter = "luid=nvidia",
    }
  );
  ASSERT_TRUE(owned_key);
  const auto cached = std::pair {*owned_key, cached_capabilities_t {3, true}};

  EXPECT_TRUE(cache_key_matches(key("luid=nvidia"), cached.first));
  EXPECT_EQ(cached.second.hevc_mode, 3);
  EXPECT_TRUE(cached.second.hdr);
  EXPECT_FALSE(cache_key_matches(key("luid=amd"), cached.first));
}

TEST(EncoderProbeOutputPolicy, ScopedOutputWinsAndIsNotChargedForAFallbackSearch) {
  bool fallback_searched = false;

  const auto chosen = choose_probe_output<adapter_t>(
    output("scoped-output", "luid=nvidia"),
    [&]() {
      fallback_searched = true;
      return output("any-output", "luid=amd");
    }
  );

  ASSERT_TRUE(chosen);
  EXPECT_EQ(chosen->display_name, "scoped-output");
  EXPECT_EQ(chosen->adapter, "luid=nvidia");
  EXPECT_FALSE(chosen->from_fallback);
  EXPECT_FALSE(fallback_searched);
}

TEST(EncoderProbeOutputPolicy, FallbackOutputCarriesItsOwnAdapterNotTheHint) {
  // The regression this guards: at the logon screen the hinted render adapter
  // owns no output while the display sits on another adapter. Probing that
  // display under the hint's adapter would fail the downstream adapter check.
  const auto chosen = choose_probe_output<adapter_t>(
    std::optional<capture_output_t> {std::nullopt},
    [&]() {
      return output("any-output", "luid=igpu");
    }
  );

  ASSERT_TRUE(chosen);
  EXPECT_EQ(chosen->display_name, "any-output");
  EXPECT_EQ(chosen->adapter, "luid=igpu");
  EXPECT_TRUE(chosen->from_fallback);
}

TEST(EncoderProbeOutputPolicy, NoCaptureReadyOutputAtAllYieldsNoChoice) {
  EXPECT_FALSE(choose_probe_output<adapter_t>(
    std::optional<capture_output_t> {std::nullopt},
    no_output()
  ));
}
