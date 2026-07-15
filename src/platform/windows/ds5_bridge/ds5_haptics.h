/**
 * @file src/platform/windows/ds5_bridge/ds5_haptics.h
 * @brief Phase 2 — build paced DualSense 0x36 audio/haptic BT reports from the
 *        iso-OUT PCM the game renders to the virtual DS5 audio endpoint.
 *
 * The DS5's fine "HD" haptics are voice-coil actuators driven by an audio
 * stream, not by the 0x31 rumble/trigger report (which Phase 1 already carries).
 * The game writes 4-channel/16-bit/48 kHz PCM to the virtual UAC endpoint
 * (usbip_ds5_device EP 0x01); ch0/1 are the speaker and ch2/3 the voice coils.
 * This builder FIR-decimates the voice-coil channels to the DS5's 3 kHz int8
 * haptic grid, packs them into the 0x12 sub-packet of a fixed 398-byte 0x36
 * report (geometry ported from our device-proven ds5_av_play.c), signs it with
 * the DS5 BT CRC and hands it to the session, which sends it as a PACED
 * OUTPUT_REPORT the TV injects via raw-ACL — symmetric to the 0x31 path.
 *
 * Phase 2a implements haptics; the 0x13 speaker slot carries a valid Opus
 * silence frame so the report is well-formed (live speaker audio is Phase 2b).
 *
 * License-clean: DSP + report geometry are our own (ds5_av_capture.cpp /
 * ds5_av_play.c, sh00bx authorship). No CTM source consulted.
 */
#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <vector>

namespace platf::ds5_bridge {

  // Full DS5 0x36 BT output report length. FIXED — the DS5 HID descriptor
  // rejects a shorter report (no audio/haptics). Geometry from ds5_av_play.c.
  constexpr int DS5_0X36_LEN = 398;

  /**
   * @brief Turns iso-OUT PCM into paced DS5 0x36 haptic reports.
   *
   * feed_pcm() runs on the usbip server thread (real-time-ish, no allocation on
   * the hot path); build_0x36() runs on the session's pacer. The only shared
   * state is the latest 64-byte haptic snapshot, guarded by hap_mtx_.
   */
  class ds5_haptic_builder {
  public:
    ds5_haptic_builder();

    /// Feed raw iso-OUT PCM (int16 interleaved, 4ch @48 kHz: ch0/1 speaker,
    /// ch2/3 voice-coil). Decimates the voice-coil channels and updates the
    /// latest haptic snapshot (RMS-gated). Safe to call from any thread.
    void feed_pcm(const uint8_t *pcm, size_t len);

    /// Assemble the next 398-byte 0x36 report into @p out. Returns true while
    /// the report carries live haptic content (within the grace window); the
    /// pacer stops driving once this goes false (DS5 falls quiet, no idle hum).
    bool build_0x36(uint8_t out[DS5_0X36_LEN]);

  private:
    // -- FIR-decimation DSP (ported from ds5_av_capture.cpp) -----------------
    static constexpr int SR = 48000;
    static constexpr int CHANS = 4;            // ch0/1 speaker, ch2/3 voice-coil
    static constexpr int SPK_L = 0, SPK_R = 1;  // (unused in 2a; documents layout)
    static constexpr int HAP_L = 2, HAP_R = 3;
    static constexpr int DECIM = 16;           // 48000 -> 3000 Hz
    static constexpr int PROC_BLOCK = 128;     // multiple of DECIM (phase-consistent)
    static constexpr int HAP_OUT = 32;         // decimated stereo frames per 64B payload
    static constexpr int NTAP = 129;           // Kaiser-windowed sinc LP @1400 Hz
    static constexpr double FIR_FC = 1400.0;
    static constexpr double FIR_BETA = 8.6;
    static constexpr double ACTIVE_RMS = 0.0015;  // voice-coil RMS squelch

    void build_fir();
    void process_block(const float *hapL, const float *hapR);
    int8_t quantize_one(double d);

    std::array<double, NTAP> fir_ {};
    std::array<float, PROC_BLOCK> prevHL_ {};   // overlap-save history (ch2/3)
    std::array<float, PROC_BLOCK> prevHR_ {};
    bool primed_ = false;
    std::array<float, PROC_BLOCK> curL_ {};     // current-block accumulation
    std::array<float, PROC_BLOCK> curR_ {};
    int cur_n_ = 0;
    std::array<float, HAP_OUT> decL_ {};
    std::array<float, HAP_OUT> decR_ {};
    int dec_n_ = 0;
    uint64_t rng_ = 0x9E3779B97F4A7C15ULL;

    // -- latest haptic snapshot (shared with the pacer) ----------------------
    static constexpr int HAPTIC_BYTES = 64;
    std::mutex hap_mtx_;
    std::array<int8_t, HAPTIC_BYTES> latest_haptic_ {};
    std::chrono::steady_clock::time_point latest_haptic_ts_ {};  // last snapshot store
    std::chrono::steady_clock::time_point last_signal_ts_ {};    // last above-squelch block
    bool have_haptic_ = false;

    // Continuity vs idle-gating are decoupled: every decimated block updates the
    // snapshot (so an active effect streams smoothly, no RMS-gate dropouts), while
    // idle-gating keys off last_signal_ts_ (the last above-squelch block) so the
    // DS5 still falls quiet after real silence instead of humming. HAPTIC_STALE
    // only bridges an actual feed stall (game stopped writing audio); it is well
    // above the ~10.7 ms snapshot cadence so normal jitter never zeroes the coil.
    static constexpr auto HAPTIC_STALE = std::chrono::milliseconds(35);
    static constexpr auto GRACE = std::chrono::milliseconds(300);

    // -- 398-byte 0x36 skeleton + Opus-silence speaker slot ------------------
    std::array<uint8_t, DS5_0X36_LEN> skeleton_ {};
    uint8_t out_seq_ = 0;
    uint8_t pktctr_ = 0;

    void build_skeleton();
    void encode_opus_silence();  // fills the 0x13 slot in skeleton_ once
  };

}  // namespace platf::ds5_bridge
