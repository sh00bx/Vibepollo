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
    /// DS4's audio path). A stream that ends is first flushed with a bounded
    /// run of encoded silence; see SILENCE_TAIL_REPORTS.
    bool build_0x17(uint8_t out[DS4_0X17_LEN], uint8_t route);

    int pace_base_us() const { return DS4_PACE_BASE_US; }

    /// True while the startup burst is still being handed to the pad. The pad
    /// plays from its own buffer and has no rate feedback, so a stream fed at
    /// exactly real time leaves it with a zero cushion and every bit of air
    /// jitter (30-67 ms measured on this link) becomes an audible dropout.
    /// DS4Windows' clocked transport solves it the same way: eight 0x17 reports
    /// presented ~4 ms apart build roughly 100 ms of controller-side coverage
    /// before steady state settles onto one 16 ms report per 16 ms.
    bool priming() const { return prime_left_.load(std::memory_order_relaxed) > 0; }
    int prime_pace_us() const { return PRIME_PACE_US; }

    /// Whether the report build_0x17() last returned belongs to that burst, and
    /// so must reach the controller UNPACED. Pacer thread only.
    bool last_report_was_prime() const { return last_primed_; }

    /// Diagnostics counters (reports built / PCM frames dropped on overflow /
    /// silence-fill pulls while active / streams flushed with a silence tail).
    uint64_t built() const { return built_.load(std::memory_order_relaxed); }
    uint64_t dropped_frames() const { return dropped_.load(std::memory_order_relaxed); }
    uint64_t underruns() const { return underrun_.load(std::memory_order_relaxed); }
    /// Stream ends closed out with a tail, split by what ended them: the
    /// endpoint going away (the game closed its speaker session) versus the
    /// silence squelch (the endpoint stayed open but went quiet).
    uint64_t flushes() const { return flushed_.load(std::memory_order_relaxed); }
    uint64_t flushes_idle() const { return flushed_idle_.load(std::memory_order_relaxed); }
    uint64_t primed() const { return primed_.load(std::memory_order_relaxed); }

  private:
    // PCM ring, interleaved stereo frames. 300 ms cap; overflow drops oldest
    // (the rate servo on the session pacer is the real rate-matching, this cap
    // only bounds a stall).
    static constexpr int RING_CAP_FRAMES = 16000;  // 500 ms @32 kHz
    static constexpr int RING_TRIM_FRAMES = 3200;  // refill level after an overflow trim
    // The startup burst carries SILENCE, not buffered audio, and leaves the ring
    // alone. What the pad needs up front is TIME, and 128 ms of injected quiet
    // buys exactly as much of it as 128 ms of the effect would -- but for free:
    // priming out of the ring would mean buffering 128 ms of the source first,
    // i.e. delaying the effect by that much on top of the source cushion, and
    // it drained the ring to nothing right when steady state began (measured
    // 2026-08-26: underrun == PRIME_REPORTS on every single stream start).
    //
    // LATENCY BUDGET (measured complaint 2026-08-26: "ziemlich verzögert").
    // The burst is permanent delay, not just startup delay: the pad plays it
    // out ahead of the real audio for as long as the stream lives, so every
    // effect is heard PRIME_REPORTS*16 ms late. It buys exactly that much
    // tolerance for a late report, 1:1 -- there is no setting that is both
    // tight and jitter-proof. 5 reports = 80 ms sits just above the 30-67 ms
    // air jitter measured on this link, and a starvation event is no longer
    // catastrophic now that the end-of-stream disarm exists (it used to leave
    // the pad looping forever). Was 8 (128 ms) until the latency complaint.
    static constexpr int PRIME_REPORTS = 5;      // 80 ms of pad-side coverage
    static constexpr int PRIME_PACE_US = 4000;   // spacing of the burst
    // Buffered before the first real pull, so the steady 16 ms grid has slack
    // while the endpoint refills. Paid once per stream, not per effect: within
    // the silence hangover the stream stays up and a follow-up effect skips it.
    static constexpr int WARMUP_FRAMES = 1024;   // 32 ms @32 kHz
    static constexpr int64_t FEED_IDLE_MS = 250;   // no PCM for this long -> stream over
    static constexpr int64_t SILENCE_HANGOVER_MS = 3000;  // keep encoding this long past the last energy
    // Bounded silence flush appended when a stream ends. The pad plays out of
    // its own speaker buffer, and when the 0x17 stream just stops it keeps
    // repeating what is still in there -- a game that closes its speaker
    // session the moment an effect finishes (AC4 does) leaves the tail of that
    // effect looping in the controller. Overwriting the buffer with silence
    // first ends the sound cleanly. Bounded on purpose: an UNBOUNDED silence
    // stream is what wedges the DS4's audio path (CTM, 2026-07-25) and is the
    // whole reason the squelch above exists; 256 ms is several times the pad's
    // buffer and still far short of a stream.
    static constexpr int SILENCE_TAIL_REPORTS = 16;  // 16 ms per report -> 256 ms

    // ...and then say so explicitly. Silence alone did NOT stop the repeat in
    // practice (2026-08-26): the pad keeps looping the last effect even with
    // the app gone, so it latches rather than simply replaying whatever the
    // buffer holds. Target byte 0x00 is the documented "no audio target"
    // (CTM map ds4_usb_over_ds4_bt, op.9) -- the low nibble must carry 0x02 or
    // 0x04 for any output at all, so zero disarms the plane. The next stream's
    // first report carries a real target again and re-arms it.
    //
    // These ride at the END of the tail, after the buffer is already silent, so
    // a pad that only needed the silence is unaffected.
    static constexpr int STOP_REPORTS = 8;  // 128 ms of "target off"

    std::mutex mtx_;
    std::vector<int16_t> ring_;  // interleaved L/R
    bool warmed_ = false;
    bool streaming_ = false;  // last build emitted real audio
    bool enc_stale_ = true;   // filterbank history predates the current stream
    int tail_left_ = 0;       // silence reports still owed to the flush
    bool last_primed_ = false;              // pacer thread only
    std::atomic<int> prime_left_ {0};       // burst reports still owed to the pad
    int64_t last_feed_ms_ = 0;
    int64_t last_energy_ms_ = 0;

    void *sbc_ = nullptr;  // sbc_t*, kept opaque so sbc.h stays out of this header
    uint16_t ctr_ = 0;

    /// Re-arm the encoder with the wire shape, dropping the analysis
    /// filterbank's history. Caller holds mtx_.
    void reset_encoder_locked();

    std::atomic<uint64_t> built_ {0};
    std::atomic<uint64_t> dropped_ {0};
    std::atomic<uint64_t> underrun_ {0};
    std::atomic<uint64_t> flushed_ {0};
    std::atomic<uint64_t> flushed_idle_ {0};
    std::atomic<uint64_t> primed_ {0};
  };

}  // namespace platf::ds5_bridge
