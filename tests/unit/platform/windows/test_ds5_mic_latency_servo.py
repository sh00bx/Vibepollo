#!/usr/bin/env python3
"""Exercise the production DS5 microphone uplink ring against arrival jitter.

Usage: python3 test_ds5_mic_latency_servo.py [repository]
The Opus codec and the steady clock are the only boundaries; the ring geometry
(ds5_mic.h) and feed()/pull()/push_locked()/plc_locked() (ds5_mic.cpp) are the
production sources. Regression: a run of WiFi stalls used to grow the ring by
five concealment frames each and nothing ever gave that delay back, so the
uplink parked at the RING_CAP ceiling (permanently late speech).
"""
from pathlib import Path
import re
import subprocess
import sys
import tempfile

root = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else Path(__file__).resolve().parents[4]
header = (root / "src/platform/windows/ds5_bridge/ds5_mic.h").read_text()
source = (root / "src/platform/windows/ds5_bridge/ds5_mic.cpp").read_text()

geometry = re.findall(r"^  constexpr int DS5_MIC_[A-Z_0-9]+ =.*$", header, re.M)
limits = re.findall(r"^    static constexpr (?:size_t|uint64_t|int) [A-Z_]+ =.*$", header, re.M)
if len(geometry) < 6 or len(limits) < 5:
    raise RuntimeError("Missing ring geometry / limits in ds5_mic.h")
if not any("MAX_FILL" in line for line in limits):
    raise RuntimeError("Missing MAX_FILL latency ceiling in ds5_mic.h")

members = []
for signature in (
    r"void ds5_mic_uplink::push_locked\(const int16_t \*pcm, int frames\)",
    r"void ds5_mic_uplink::plc_locked\(int frames\)",
    r"void ds5_mic_uplink::feed\(uint16_t seq, const uint8_t \*opus, size_t len\)",
    r"size_t ds5_mic_uplink::pull\(uint8_t \*dst, size_t bytes\)",
):
    match = re.search(r"^  " + signature + r" \{[\s\S]*?^  \}", source, re.M)
    if not match:
        raise RuntimeError(f"Missing production member matching {signature}")
    members.append(match.group())

harness = r'''
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <vector>

// GEOMETRY

typedef std::int32_t opus_int32;
#define OPUS_RESET_STATE 4028

struct OpusDecoder {
  int state;
};

static OpusDecoder g_decoder {0};
static std::uint64_t g_now_us = 0;

// The pad's clock and the codec are the boundaries: every packet is one valid
// 10 ms frame, and concealment produces a frame of the same size.
static std::uint64_t mic_now_us() {
  return g_now_us;
}

static int opus_decoder_ctl(OpusDecoder *, int) {
  return 0;
}

static int opus_packet_get_nb_samples(const std::uint8_t *, opus_int32, int) {
  return DS5_MIC_FRAME_SAMPLES;
}

static int opus_decode(OpusDecoder *, const std::uint8_t *, opus_int32, std::int16_t *, int, int) {
  return DS5_MIC_FRAME_SAMPLES;
}

struct ds5_mic_uplink {
  struct stats_t {
    std::uint64_t frames {}, plc {}, bad {}, drop_bytes {}, pulled_bytes {}, silence_bytes {}, starts {};
    std::size_t fill_bytes {};
  };

  // LIMITS

  void push_locked(const std::int16_t *pcm, int frames);
  void plc_locked(int frames);
  void feed(std::uint16_t seq, const std::uint8_t *opus, std::size_t len);
  std::size_t pull(std::uint8_t *dst, std::size_t bytes);

  OpusDecoder *dec_ {&g_decoder};
  std::mutex mtx_;
  std::vector<std::uint8_t> ring_ = std::vector<std::uint8_t>(RING_CAP, 0);
  std::size_t head_ {0};
  std::size_t count_ {0};
  bool primed_ {false};
  bool live_ {false};
  std::uint16_t last_seq_ {0};
  std::uint64_t last_arrival_us_ {0};
  stats_t st_ {};
  std::vector<std::int16_t> pcm_ =
    std::vector<std::int16_t>((std::size_t) DS5_MIC_FRAME_SAMPLES * DS5_MIC_CHANNELS, 0);
};

// MEMBERS

static int failures = 0;

static void check(bool ok, const char *what) {
  if (!ok) {
    std::printf("FAIL: %s\n", what);
    ++failures;
  } else {
    std::printf("ok: %s\n", what);
  }
}

int main() {
  ds5_mic_uplink mic;
  std::uint8_t packet[71] {};
  std::uint8_t endpoint[DS5_MIC_BYTES_PER_MS] {};
  std::uint16_t seq = 0;

  // The capture endpoint drains exactly its nominal bytes every 1 ms.
  auto drain_for = [&](std::uint64_t us) {
    for (std::uint64_t t = 0; t < us; t += 1000) {
      g_now_us += 1000;
      mic.pull(endpoint, sizeof(endpoint));
    }
  };

  // Steady state: one 10 ms packet every 10 ms, no loss.
  for (int i = 0; i < 200; ++i) {
    mic.feed(seq++, packet, sizeof(packet));
    drain_for(10000);
  }
  check(mic.count_ <= ds5_mic_uplink::PREBUFFER + 2 * (std::size_t) DS5_MIC_FRAME_BYTES,
        "steady state settles at the prebuffer level");
  check(mic.st_.plc == 0, "no concealment without a gap");
  check(mic.st_.drop_bytes == 0, "the latency servo never fires in steady state");

  // Ten WiFi stalls of 60 ms, each followed by the late packets in a burst --
  // the TV's control channel is reliable, so they do arrive.
  for (int s = 0; s < 10; ++s) {
    drain_for(60000);
    for (int i = 0; i < 6; ++i) {
      mic.feed(seq++, packet, sizeof(packet));
    }
    for (int i = 0; i < 20; ++i) {
      mic.feed(seq++, packet, sizeof(packet));
      drain_for(10000);
    }
  }
  check(mic.count_ <= ds5_mic_uplink::MAX_FILL, "ring level stays under the latency ceiling");
  check(mic.count_ <= ds5_mic_uplink::PREBUFFER + 4 * (std::size_t) DS5_MIC_FRAME_BYTES,
        "the servo takes the accumulated delay back out");
  check(mic.count_ < ds5_mic_uplink::RING_CAP / 2, "the uplink does not park at the ring cap");

  // Loss the TV numbered is still concealed: those packets never arrive, so the
  // concealment replaces them instead of adding delay.
  const std::uint64_t plc_before = mic.st_.plc;
  seq = (std::uint16_t) (seq + 3);
  mic.feed(seq++, packet, sizeof(packet));
  check(mic.st_.plc == plc_before + 3, "a sequence gap is concealed frame for frame");

  // A jitter stall with the buffer still full must not synthesize anything.
  drain_for(10000);
  const std::uint64_t plc_jitter = mic.st_.plc;
  g_now_us += 40000;
  mic.feed(seq++, packet, sizeof(packet));
  check(mic.count_ >= ds5_mic_uplink::PREBUFFER, "buffer still covers the stall");
  check(mic.st_.plc == plc_jitter, "a covered jitter stall adds no concealment delay");

  return failures ? 1 : 0;
}
'''

harness = harness.replace("// GEOMETRY", "\n".join(line.strip() for line in geometry))
harness = harness.replace("// LIMITS", "\n".join("  " + line.strip() for line in limits))
harness = harness.replace("// MEMBERS", "\n\n".join(members))

with tempfile.TemporaryDirectory(prefix="ds5-mic-latency-servo-") as temporary:
    cpp = Path(temporary) / "test.cpp"
    cpp.write_text(harness)
    exe = Path(temporary) / "test"
    subprocess.run(
        ["g++", "-std=c++20", "-Wall", "-Wextra", "-Werror", "-Wno-unused-parameter", str(cpp), "-o", str(exe)],
        check=True,
    )
    subprocess.run([str(exe)], check=True)
    print("production DS5 mic uplink latency regressions passed")
