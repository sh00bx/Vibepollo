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
 * Phase 2b also carries live speaker audio: the game's ch0/1 PCM is jitter-
 * buffered and Opus-encoded per 10 ms into the 0x13 sub-packet (exactly-once,
 * in-order — Opus is stateful, unlike the latest-wins int8 haptic), with PLC on
 * underrun so the 0x13 is always a valid frame.
 *
 * License-clean: DSP + report geometry are our own (ds5_av_capture.cpp /
 * ds5_av_play.c, sh00bx authorship). No CTM source consulted.
 */
#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <vector>

// Global forward declaration so `enc_` resolves to the SAME ::OpusEncoder in
// every TU regardless of whether <opus/opus.h> was included first (opus.h does
// `typedef struct OpusEncoder OpusEncoder;` at global scope). Declaring it inside
// the namespace instead would inject a distinct platf::ds5_bridge::OpusEncoder in
// opus-less TUs (bridge_host.cpp) and ODR-clash with the opus.h type in this .cpp.
struct OpusEncoder;

namespace platf::ds5_bridge {

  // Full DS5 0x36 BT output report length. FIXED — the DS5 HID descriptor
  // rejects a shorter report (no audio/haptics). Geometry from ds5_av_play.c.
  constexpr int DS5_0X36_LEN = 398;

  // Batched audio/haptic report. The DS5's BT output reports 0x31..0x39 are a
  // SIZE LADDER in 64-byte steps (payload 77/141/205/269/333/397/461/525/546 per
  // the BT HID report descriptor); 0x39 is the top and the only step that is not
  // +64, because its L2CAP PDU (4 + 1 HID prefix + 547 = 552 B) is exactly the
  // payload maximum of one 3-DH3 EDR baseband packet. It carries TWO 10 ms Opus
  // frames and TWO 64-byte coil blocks under sub-block headers whose bit 6 marks
  // the doubling (0x12|0xC0 = 0xD2 with length byte 64 => 128 bytes on the wire).
  //
  // Same audio, ~47 reports/s instead of ~94: half the on-air ACL packets and
  // ~31% fewer bytes, which is aimed straight at the TV-link queue ratchet. The
  // costs are +10.67 ms of buffering and that the per-frame audio SetState no
  // longer fits — it moves to a periodic 0x32 (DS5_0X32_LEN).
  //
  // Layout cross-checked against two independent implementations (awalol
  // DS5Dongle src/audio.cpp, Kodzinho DualSense-Bluetooth-Audio); no vendor spec
  // exists, so it is a hypothesis until the pad is measured.
  constexpr int DS5_0X39_LEN = 547;
  constexpr int DS5_0X32_LEN = 142;   // standalone audio SetState (batched mode)
  constexpr int DS5_AUDIO_REPORT_MAX = DS5_0X39_LEN;  // pacer buffer sizing

  // Speaker clocking, shared with the session pacer (bridge_host.cpp).
  //
  // The DS5 drains one 480-sample 0x36 per ~10.667 ms — an effective ~45 kHz
  // device clock (probed; the nominal "48 kHz" frames play 6.25% slow).
  // Sending at exactly the drain rate means the TV-side inject queue can never
  // shrink — every BT NOCP gap (30-80 ms stalls come in storms on this link)
  // ratchets it up permanently, parking ~100 ms of latency and turning each
  // further gap into a drop burst (hard Opus discontinuities; 529 dropped
  // frames in one 19-min session).
  //
  // The pacer therefore runs a RATE SERVO on the TV's inject-queue telemetry
  // (CTMB_MSG_PACE_FEEDBACK): base period = the true drain cadence; backlog or
  // drops stretch it (up to DS5_PACE_ADJ_MAX) until the queue bleeds empty,
  // then it decays back. The speaker resample ratio tracks the live period
  // (audio-clock recovery), so production always equals consumption and the
  // only artifact is a sub-1% pitch wobble at multi-second time constants.
  // Without feedback (old TV app) the pacer falls back to a fixed
  // DS5_PACE_FALLBACK_ADJ_US margin — the pre-servo behavior.
  constexpr int DS5_SPK_FRAME = 480;      // samples/ch per 0x36 Opus frame
  constexpr int DS5_PACE_BASE_US = 10667;         // true 93.75/s drain cadence
  // Batched cadence: one 0x39 carries two frames, so the report period doubles
  // while the audio/haptic sample rate the pad drains is unchanged. The servo
  // adjustment (pace_adj_us_) keeps its meaning — it is a per-report stretch.
  constexpr int DS5_PACE_BASE_BATCHED_US = 2 * DS5_PACE_BASE_US;  // ~46.9/s
  constexpr int DS5_PACE_ADJ_MAX_US = 140;        // slowest: ~92.5/s (-1.3%)
  constexpr int DS5_PACE_FALLBACK_ADJ_US = 35;    // no-feedback static margin (-0.33%)

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
    ~ds5_haptic_builder();

    /// Feed raw iso-OUT PCM (int16 interleaved, 4ch @48 kHz: ch0/1 speaker,
    /// ch2/3 voice-coil). Decimates the voice-coil channels into the latest
    /// haptic snapshot (RMS-gated) and queues the speaker channels for Opus.
    /// Safe to call from any thread.
    void feed_pcm(const uint8_t *pcm, size_t len);

    /// Assemble the next 398-byte 0x36 report into @p out. Returns true while
    /// the report carries live haptic content (within the grace window); the
    /// pacer stops driving once this goes false (DS5 falls quiet, no idle hum).
    bool build_0x36(uint8_t out[DS5_0X36_LEN]);

    /// Batched form: the same content as two consecutive 0x36 ticks packed into
    /// one 547-byte 0x39 (two Opus frames, two coil blocks). Same return
    /// semantics as build_0x36.
    bool build_0x39(uint8_t out[DS5_0X39_LEN]);

    /// Build the current report form; @p out must hold DS5_AUDIO_REPORT_MAX.
    bool build_audio(uint8_t *out) {
      return batched_ ? build_0x39(out) : build_0x36(out);
    }

    /// Select the report form. Set once before the pacer starts (a mid-stream
    /// flip would hand the pad a report whose counters jump by a different step).
    void set_batched(bool on) {
      batched_ = on;
      recompute_cushion();   // the cushion floor depends on frames-per-tick
    }

    /// Speaker jitter cushion in 10.67 ms frames (set before the pacer starts).
    /// Lower = less pad-speaker latency, less tolerance for a late feed chunk;
    /// watch spkplc in the ds5-haptics line, which reports the cost directly.
    void set_cushion_frames(int n) {
      cushion_frames_ = n;
      recompute_cushion();
    }

    /// AudioControl byte of the audio SetState (inline in 0x36, standalone 0x32).
    /// Set before the pacer starts.
    void set_audio_control(uint8_t v);

    /// The clamped cushion actually in force (frames).
    int cushion_frames() const {
      return eff_cushion_;
    }

    bool batched() const {
      return batched_;
    }

    /// Wire length of the current report form.
    int report_len() const {
      return batched_ ? DS5_0X39_LEN : DS5_0X36_LEN;
    }

    /// Base pacer period for the current report form (before the rate servo).
    int pace_base_us() const {
      return batched_ ? DS5_PACE_BASE_BATCHED_US : DS5_PACE_BASE_US;
    }

    /// Speaker frames the pacer consumes per tick: 0x36 carries one 0x13 block,
    /// 0x39 carries two. The feed's rate match runs on the per-FRAME period, so
    /// it must divide the tick period by this — see the RATE MATCH note below.
    int spk_frames_per_tick() const {
      return batched_ ? 2 : 1;
    }

    /// Standalone audio SetState (0x32, 142 B). In batched mode the 0x39 has no
    /// room for the per-frame SetState block, so the audio Allow bits/volumes are
    /// re-asserted with this report instead — same payload, just carried
    /// separately and rarely. Always fills @p out and signs it.
    void build_setstate_0x32(uint8_t out[DS5_0X32_LEN]);

    /// Whether this title has ever fed above-squelch voice-coil energy, i.e.
    /// whether it renders HD haptics at all. Latches for the session: a title
    /// that drives the coils keeps doing so, and the burst structure of real
    /// haptics (seconds of silence between effects) would make a sliding window
    /// flap. Used by the 0x31 path to decide whether the game's
    /// USE_RUMBLE_NOT_HAPTICS flag contradicts its own audio (see
    /// bridge_host.cpp::on_game_output). Safe from any thread.
    bool coil_ever_active() const {
      return coil_ever_.load(std::memory_order_relaxed);
    }

    /// Rate-servo hook (pacer thread): the live pacer period. The speaker
    /// resample ratio follows it so production == consumption at any servo
    /// setting (audio-clock recovery; no systematic PLC or ring growth).
    void set_pace_us(int us) {
      pace_us_.store(us, std::memory_order_relaxed);
    }

  private:
    // -- FIR-decimation DSP (ported from ds5_av_capture.cpp) -----------------
    static constexpr int SR = 48000;
    static constexpr int CHANS = 4;            // ch0/1 speaker, ch2/3 voice-coil
    static constexpr int SPK_L = 0, SPK_R = 1;  // speaker pair (2b: -> 0x13 Opus)
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

    // -- haptic snapshot: latest-wins (shared with the pacer) ----------------
    // The device-proven ds5_av_play.c model: the feed overwrites the newest 64-B
    // coil snapshot, the pacer sends the latest one each tick. This is correct
    // because the iso-OUT feed is real-time-paced upstream (usbip_ds5_device
    // completes iso URBs on a virtual audio clock), so the producer runs at one
    // snapshot per 10.667 ms — the same cadence the pacer sends at (93.75/s, the
    // DS5 drain clock), so each snapshot is sent exactly once modulo jitter.
    // (A FIFO ring was tried while the feed still flooded at ~150x real-time;
    // it only reshuffled the incoherent flood.)
    static constexpr int HAPTIC_BYTES = 64;
    mutable std::mutex hap_mtx_;
    std::array<int8_t, HAPTIC_BYTES> latest_frame_ {};  // newest coil snapshot
    // Previous snapshot, kept only for the batched form: a 0x39 needs TWO
    // consecutive coil blocks (21.33 ms of signal). Sending the latest one twice
    // would replay the same 10.67 ms and read as a time-stretched buzz, so the
    // pair is [prev, latest] in production order. Under the normal 1:1 cadence
    // (feed ~93.75 blocks/s, pacer ~46.9 reports/s) both are fresh; if only one
    // is, the report falls back to latest-only and the first half stays silent.
    std::array<int8_t, HAPTIC_BYTES> prev_frame_ {};
    bool have_prev_ = false;
    std::chrono::steady_clock::time_point latest_ts_ {};       // when it was produced
    std::chrono::steady_clock::time_point last_signal_ts_ {};  // last above-squelch block
    bool have_haptic_ = false;
    // Latched once any block clears ACTIVE_RMS — "this title renders HD
    // haptics". Read from the usbip output thread, so it is an atomic rather
    // than hap_mtx_-guarded state (the 0x31 path must not block on the feed).
    std::atomic<bool> coil_ever_ {false};

    // A snapshot older than this = the feed stalled (endpoint closed / paused);
    // zero the coil so it stops buzzing the last frame. Generous vs the ~10.67 ms
    // production cadence to tolerate URB jitter (ds5_av_play.c uses 15 ms on lossy
    // WiFi; our feed is local + paced, so jitter is low, but keep headroom).
    static constexpr auto HAPTIC_STALE = std::chrono::milliseconds(30);
    // Same idea for the batched form, whose report period is ~21.33 ms: a
    // perfectly healthy snapshot can be that old at build time, so the 30 ms
    // window would read a live feed as stalled every other report.
    static constexpr auto HAPTIC_STALE_BATCHED = std::chrono::milliseconds(45);
    // Idle-gate on real signal activity so the DS5 falls quiet after true silence
    // (no idle hum) rather than on buffer state. Driven by haptic OR speaker.
    static constexpr auto GRACE = std::chrono::milliseconds(300);

    // -- speaker (0x13 Opus) — ported from ds5_av_play.c ---------------------
    // The game's ch0/1 speaker PCM. Unlike the stateless int8 haptic, Opus is a
    // stateful codec, so the pacer must encode exactly one in-order frame per
    // tick: a sample ring (jitter buffer) feeds the encoder, PLC covers
    // underruns.
    //
    // RATE MATCH: the feed resamples 48 kHz onto the DS5's effective drain
    // clock, derived from the LIVE pacer period (pace_us_, rate-servo'd by the
    // session) so production, wire rate and device drain stay coupled at any
    // servo setting — see the clocking block at namespace scope. The drain clock
    // is one frame per pace_us_/spk_frames_per_tick(), NOT per tick: 0x39 pops
    // two frames per tick, and feeding it the tick period would produce at half
    // the consumed rate (chronic underrun, ~50 % PLC).
    static constexpr int OPUS_FRAME = DS5_SPK_FRAME;  // one 0x13 frame (samples/ch, nominal 48k)
    std::atomic<int> pace_us_ {DS5_PACE_BASE_US + DS5_PACE_FALLBACK_ADJ_US};
    static constexpr int OPUS_BYTES = 200;          // 0x13 payload size (fixed, 160k CBR)
    static constexpr double SPK_RMS = 0.0005;       // speaker activity squelch (ds5_av_capture.cpp)
    static constexpr int SPK_CONCEAL = 8;           // max consecutive PLC-driven sends (~85 ms)
    static constexpr int SPK_RING_FRAMES = 9600;    // ~200 ms of stereo cushion cap
    // Speaker jitter cushion, in 10.67 ms frames. Historically a hard 4 (~43 ms) with
    // drain at 2x. It is pure LATENCY on the pad speaker, so it is worth tuning down —
    // but the usable margin is NOT the cushion itself: the pacer pops
    // spk_frames_per_tick() frames at once, so what absorbs late production is
    // (cushion - frames_per_tick) frames. At the default 4 that is 32 ms for 0x36 but
    // only 21 ms for 0x39. Hence the floor below.
    static constexpr int SPK_CUSHION_DEFAULT = 4;
    // Cap so the latency-drain threshold (2*n frames, see recompute_cushion)
    // stays below the ring capacity (SPK_RING_FRAMES = 9600 sample-frames
    // = 20 Opus frames): spk_push
    // drops-oldest at that cap, so spk_count_ never exceeds it and for n >= 10
    // the drain condition `spk_count_ > spk_lat_drain_` could never fire.
    static constexpr int SPK_CUSHION_MAX = 9;
    int cushion_frames_ = SPK_CUSHION_DEFAULT;
    size_t spk_lat_target_ = (size_t) SPK_CUSHION_DEFAULT * OPUS_FRAME;
    size_t spk_lat_drain_ = (size_t) (2 * SPK_CUSHION_DEFAULT) * OPUS_FRAME;
    /// Clamp the requested cushion to something the pop pattern can actually sustain
    /// and derive the drain threshold. Floor is frames-per-tick + 1: a cushion that
    /// only covers the pops themselves leaves ZERO margin for a late feed chunk and
    /// degenerates into prime/underrun alternation (that is the 50 % PLC failure the
    /// batched path already hit once, from the other direction).
    void recompute_cushion() {
      const int floor_n = spk_frames_per_tick() + 1;
      int n = cushion_frames_ < floor_n ? floor_n : cushion_frames_;
      if (n > SPK_CUSHION_MAX) n = SPK_CUSHION_MAX;
      // Belt and braces to SPK_CUSHION_MAX: keep the drain threshold at least
      // one poppable frame below the ring cap, so the valve stays reachable
      // even if the constants above drift.
      const int drain_max_n = (SPK_RING_FRAMES - OPUS_FRAME) / (2 * OPUS_FRAME);
      if (n > drain_max_n) n = drain_max_n;
      spk_lat_target_ = (size_t) n * OPUS_FRAME;
      spk_lat_drain_ = (size_t) (2 * n) * OPUS_FRAME;
      eff_cushion_ = n;
    }
    int eff_cushion_ = SPK_CUSHION_DEFAULT;
    std::mutex spk_mtx_;
    std::array<int16_t, SPK_RING_FRAMES * 2> spk_ring_ {};  // interleaved stereo
    size_t spk_head_ = 0, spk_count_ = 0;                   // frames (L+R pairs)
    bool spk_priming_ = true;                               // prefill before draining
    // Pacer-thread-only speaker state (no lock needed):
    ::OpusEncoder *enc_ = nullptr;                          // persistent, 48k/2ch/CELT
    std::array<float, OPUS_FRAME * 2> last_pcm_f_ {};       // PLC source (last good frame)
    int plc_run_ = 0;                                       // consecutive underruns
    // Speaker gain envelope. Both seams around a dropout used to be steps:
    // concealment jumped straight to 0.6 on entry, and the first real frame
    // after it came back at FULL gain. Measured on the modelled envelope, the
    // resume seam was a 1.0 discontinuity and the entry a 0.4 one — on a link
    // that produces dropouts in storms (529 dropped frames in one 19-min
    // session), i.e. an audible click at both ends of every one of them.
    //
    // spk_env_ is the gain the LAST emitted sample carried. Concealment now
    // interpolates from it to its decay target across the frame, and recovery
    // ramps from it back to 1.0 over SPK_RESUME_RAMP samples. Neither branch
    // can produce a jump because both start where the other stopped.
    static constexpr int SPK_RESUME_RAMP = 240;             // samples/ch (~5 ms @ 48k)
    float spk_env_ = 1.0f;                                  // gain at the end of the last frame
    int spk_resume_left_ = 0;                               // samples/ch of resume ramp remaining
    std::chrono::steady_clock::time_point last_spk_ts_ {};  // last frame popped (idle gate)

    void spk_push(const int16_t *pcm4, size_t nframe);  // extract ch0/1, resample, ring-push (feed thread)
    int spk_pop_frame(int16_t out[OPUS_FRAME * 2]);      // pop 480 stereo (pacer thread)

    // 48 kHz -> drain-clock resampler (feed thread only): pending input stereo
    // frames plus a fractional read position (step tracks pace_us_), polyphase Kaiser-sinc
    // interpolated. Catmull-Rom was tried first but a cubic has no anti-alias
    // stopband — downsampling folds >22 kHz content back into the audible band
    // and its interpolation sidelobes smear HF detail ("not quite as clear").
    // 64 taps @ fc 21 kHz / beta 9 leaves ~60 dB in the folded region; phase
    // rows are linearly blended so the irrational step stays artifact-free.
    static constexpr int RS_TAPS = 64;
    static constexpr int RS_PHASES = 128;
    static constexpr double RS_FC = 21000.0;  // LP cutoff (Hz, at 48 kHz input)
    static constexpr double RS_BETA = 9.0;
    std::vector<float> rs_filt_;     // (RS_PHASES+1) x RS_TAPS, DC-normalized rows
    std::vector<int16_t> rs_pend_;   // interleaved stereo input awaiting resample
    double rs_pos_ = RS_TAPS / 2 - 1;  // fractional input index (needs 31 left history)

    void build_resampler();

    // -- diagnostics (periodic log; cheap atomics) ---------------------------
    std::atomic<uint64_t> dbg_feed_frames_ {0};
    std::atomic<uint32_t> dbg_blocks_ {0}, dbg_gate_blocks_ {0};
    std::atomic<uint32_t> dbg_sends_ {0}, dbg_stale_ {0};
    std::atomic<uint32_t> dbg_spk_pop_ {0}, dbg_spk_plc_ {0};
    double dbg_max_hrms_ = 0.0;  // under hap_mtx_
    uint32_t dbg_build_ctr_ = 0;  // pacer thread only

    // -- 398-byte 0x36 skeleton (sub-packet headers + audio SetState) --------
    // The 0x12 haptic and 0x13 Opus payloads are written per tick in build_0x36.
    std::array<uint8_t, DS5_0X36_LEN> skeleton_ {};
    uint8_t audio_control_ = 0x00;   // SetState byte 7, see set_audio_control
    // -- 547-byte 0x39 skeleton (batched; no SetState block — see 0x32) ------
    std::array<uint8_t, DS5_0X39_LEN> skeleton39_ {};
    bool batched_ = false;
    uint8_t out_seq_ = 0;
    uint8_t pktctr_ = 0;

    /// One speaker frame: pop 480 stereo (or PLC), Opus-encode into @p dst.
    /// Updates last_spk_ts_/plc_run_ exactly as the single-frame path did.
    /// Pacer thread only.
    void encode_speaker_frame(uint8_t *dst, std::chrono::steady_clock::time_point now);

    /// DS5 BT output CRC over the 0xA2 seed byte + bytes [0 .. len-4), written
    /// into the last four bytes. Same for every report form.
    static void sign_report(uint8_t *out, int len);

    void build_skeleton();
  };

}  // namespace platf::ds5_bridge
