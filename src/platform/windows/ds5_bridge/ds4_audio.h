/**
 * @file src/platform/windows/ds5_bridge/ds4_audio.h
 * @brief Build paced DualShock 4 0x17 SBC audio reports from the iso-OUT PCM
 *        the game renders to the virtual DS4 speaker endpoint.
 *
 * The DS4's speaker/headset audio over Bluetooth is SBC in the 0x14..0x19
 * output-report family. The virtual DS4 exposes the real pad's 32 kHz stereo
 * UAC1 speaker endpoint; the usbip iso pacer feeds the rendered PCM here at
 * real-time cadence, and the session's pacer pulls one 0x17 report (four
 * 109-byte SBC frames = 512 PCM frames = 16 ms) per tick.
 *
 * SBC shape (probed, CTM-USBIP "Layout B", cross-checked against DS4Windows'
 * DualShock4BluetoothAudioProtocol): joint stereo, bitpool 48, 16 blocks,
 * 8 subbands, loudness allocation, 32 kHz.
 */
#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <vector>

#include "src/platform/windows/ds5_bridge/ds4_reports.h"

namespace platf::ds5_bridge {

  class ds4_audio_builder {
  public:
    ds4_audio_builder();
    ~ds4_audio_builder();

    /// Feed raw iso-OUT PCM (int16 interleaved stereo @32 kHz). Called from the
    /// usbip iso pacer thread at real-time cadence.
    void feed_pcm(const uint8_t *pcm, size_t len);

    /// Assemble the next 462-byte 0x17 report. Returns false while there is
    /// nothing to play (endpoint idle, warm-up, or the silence squelch) — the
    /// pacer sends nothing then, so an idle desktop does not stream SBC silence
    /// at the pad (CTM measured that a continuous silence stream wedges the
    /// DS4's audio path).
    bool build_0x17(uint8_t out[DS4_0X17_LEN], uint8_t route);

    int pace_base_us() const { return DS4_PACE_BASE_US; }

    /// Diagnostics counters (reports built / PCM frames dropped on overflow /
    /// silence-fill pulls while active).
    uint64_t built() const { return built_.load(std::memory_order_relaxed); }
    uint64_t dropped_frames() const { return dropped_.load(std::memory_order_relaxed); }
    uint64_t underruns() const { return underrun_.load(std::memory_order_relaxed); }

  private:
    // PCM ring, interleaved stereo frames. 300 ms cap; overflow drops oldest
    // (the rate servo on the session pacer is the real rate-matching, this cap
    // only bounds a stall).
    static constexpr int RING_CAP_FRAMES = 9600;   // 300 ms @32 kHz
    static constexpr int RING_TRIM_FRAMES = 3200;  // refill level after an overflow trim
    static constexpr int WARMUP_FRAMES = 1024;     // 32 ms before the first pull
    static constexpr int64_t FEED_IDLE_MS = 250;   // no PCM for this long -> stream over
    static constexpr int64_t SILENCE_HANGOVER_MS = 3000;  // keep encoding this long past the last energy

    std::mutex mtx_;
    std::vector<int16_t> ring_;  // interleaved L/R
    bool warmed_ = false;
    int64_t last_feed_ms_ = 0;
    int64_t last_energy_ms_ = 0;

    void *sbc_ = nullptr;  // sbc_t*, kept opaque so sbc.h stays out of this header
    uint16_t ctr_ = 0;

    std::atomic<uint64_t> built_ {0};
    std::atomic<uint64_t> dropped_ {0};
    std::atomic<uint64_t> underrun_ {0};
  };

}  // namespace platf::ds5_bridge
