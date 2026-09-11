/**
 * @file src/platform/windows/ds5_bridge/ds5_haptics.cpp
 * @brief Phase 2 DS5 0x36 haptic report builder (see ds5_haptics.h).
 *
 * DSP (Kaiser-sinc FIR decimate + TPDF-dithered int8 quantize) is ported from
 * our ds5_av_capture.cpp; the 398-byte 0x36 geometry from our ds5_av_play.c.
 * Both are sh00bx authorship — no CTM source consulted.
 */
#define _USE_MATH_DEFINES
#include <cmath>
#include <cstring>

#include <opus/opus.h>

#include "src/logging.h"
#include "src/platform/windows/ds5_bridge/ds5_haptics.h"
#include "src/platform/windows/ds5_bridge/ds5_reports.h"

namespace platf::ds5_bridge {

  namespace {
    // Report 0x36 sub-packet offsets (canonical, from ds5_av_play.c).
    // (OPUS_FRAME / OPUS_BYTES live on the class — used by member arrays too.)
    constexpr int OFF_SETSTATE = 13;   // SetState (0x10) payload
    constexpr int OFF_HAPTIC = 78;     // voice-coil (0x12) payload
    constexpr int OFF_OPUS = 144;      // Opus speaker (0x13) payload

    // Batched 0x39 geometry (see the ladder note in ds5_haptics.h). No SetState
    // block fits here — it goes out as a periodic standalone 0x32.
    constexpr int OFF39_HAPTIC_A = 12;    // first  64-byte coil block
    constexpr int OFF39_HAPTIC_B = 76;    // second 64-byte coil block
    constexpr int OFF39_OPUS_A = 142;     // first  200-byte Opus frame
    constexpr int OFF39_OPUS_B = 342;     // second 200-byte Opus frame

    // Non-destructive "audio-only" SetState (0x10) payload: asserts only the
    // audio Allow bits so it never fights Moonlight's own trigger/rumble/LED
    // writes on the same pad (ds5_av_play.c, DS5Dongle state layout). 63 bytes.
    const uint8_t state_audio_data[63] = {
      0xB0, 0x82,                 // ValidFlags: audio-only
      0x00, 0x00,                 // RumbleEmulation R/L (not allowed)
      // VolumeHeadphones=127 (7-bit field), VolumeSpeaker=255 (= "max").
      // The speaker firmware's usable range is 0x3d..0x64 (the old 0x80..0xFF
      // theory from the CTM slider is refuted, sweep 2026-09-11), so anything
      // >= 0x64 is full scale. These two bytes rarely reach the pad as sent:
      // the TV client rewrites both volume bytes in every audio mode from its
      // own sliders. Windows attenuates digitally on top (the endpoint exposes
      // no hardware volume unit).
      0x7f, 0xff,
      0x00,                       // VolumeMic (not allowed)
      0x00,                       // AudioControl (Auto mic / default); set_audio_control() overrides
      0x00,                       // MuteLightMode (ignored)
      0x00,                       // MuteControl: all UNMUTED
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // RightTriggerFFB
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // LeftTriggerFFB
      0x00, 0x00, 0x00, 0x00,     // HostTimestamp
      0x00,                       // MotorPowerLevel (not allowed)
      0x02,                       // AudioControl2: SpeakerCompPreGain=2
      0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  // [38..43] light/haptic-LPF/anim (not allowed)
      0x00, 0x00, 0x00            // [44,45,46] LedRed/Green/Blue (not allowed)
      // remaining [47..62] implicitly zero (array is 63 bytes)
    };

    double bessel_i0(double x) {
      double sum = 1.0, term = 1.0, halfx = x / 2.0;
      for (int k = 1; k < 64; ++k) {
        term *= (halfx / k);
        double t2 = term * term;
        sum += t2;
        if (t2 < sum * 1e-16) break;
      }
      return sum;
    }
  }  // namespace

  ds5_haptic_builder::ds5_haptic_builder() {
    build_fir();
    build_resampler();
    build_skeleton();
    // One persistent Opus encoder: 48k / 2ch / RESTRICTED_LOWDELAY (CELT-only, no
    // SILK look-ahead) / 10 ms / 160 kbps CBR / complexity 0 — the DS5-proven
    // config from ds5_av_play.c. Used only on the pacer thread (build_0x36), so no
    // lock. If creation fails, build_0x36 emits a zeroed (silent) 0x13.
    int err = 0;
    enc_ = opus_encoder_create(SR, 2, OPUS_APPLICATION_RESTRICTED_LOWDELAY, &err);
    if (err != OPUS_OK || !enc_) {
      if (enc_) { opus_encoder_destroy(enc_); enc_ = nullptr; }
      // Near-impossible with valid args; log it and fall back to a zeroed 0x13
      // (build_0x36 still ships a well-formed report; only the speaker is silent).
      BOOST_LOG(error) << "ds5-haptics: opus_encoder_create failed ("
                       << opus_strerror(err) << "); speaker disabled, haptics unaffected";
    } else {
      opus_encoder_ctl(enc_, OPUS_SET_EXPERT_FRAME_DURATION(OPUS_FRAMESIZE_10_MS));
      opus_encoder_ctl(enc_, OPUS_SET_BITRATE(160000));
      opus_encoder_ctl(enc_, OPUS_SET_VBR(0));
      // Complexity 0 was the TV-tool's ARM budget; on the host one stereo frame
      // per 10.7 ms is nothing — buy the full encoder quality back.
      opus_encoder_ctl(enc_, OPUS_SET_COMPLEXITY(10));
    }
    // last_signal_ts_ defaults to the epoch and have_haptic_ starts false, so
    // build_0x36 idle-gates (sends nothing) until the first real signal block.
  }

  ds5_haptic_builder::~ds5_haptic_builder() {
    if (enc_) opus_encoder_destroy(enc_);
  }

  // Polyphase fractional-delay bank for the 48 kHz -> drain-clock speaker resample:
  // RS_PHASES+1 rows of an RS_TAPS Kaiser-windowed sinc low-pass, row p sampled
  // at fractional offset p/RS_PHASES, each row normalized to unity DC gain (the
  // per-phase sinc sampling otherwise ripples the gain by up to ~0.5%).
  void ds5_haptic_builder::build_resampler() {
    const int C = RS_TAPS / 2;  // output taps span input indices i0-C+1 .. i0+C
    const double i0b = bessel_i0(RS_BETA);
    rs_filt_.assign((size_t) (RS_PHASES + 1) * RS_TAPS, 0.0f);
    for (int p = 0; p <= RS_PHASES; ++p) {
      float *row = rs_filt_.data() + (size_t) p * RS_TAPS;
      const double a = (double) p / RS_PHASES;
      double sum = 0.0;
      for (int k = 0; k < RS_TAPS; ++k) {
        const double t = (k - (C - 1)) - a;      // tap offset from the output point
        const double x = 2.0 * RS_FC / SR * t;
        const double s = (x == 0.0) ? 1.0 : std::sin(M_PI * x) / (M_PI * x);
        const double r = t / C;
        const double w = (std::abs(r) >= 1.0) ? 0.0 : bessel_i0(RS_BETA * std::sqrt(1.0 - r * r)) / i0b;
        row[k] = (float) (2.0 * RS_FC / SR * s * w);
        sum += row[k];
      }
      for (int k = 0; k < RS_TAPS; ++k) row[k] = (float) (row[k] / sum);
    }
  }

  // Kaiser-windowed sinc low-pass @FIR_FC, unity DC gain (ds5_av_capture.cpp).
  void ds5_haptic_builder::build_fir() {
    const double M = (NTAP - 1) / 2.0;
    const double i0b = bessel_i0(FIR_BETA);
    double sum = 0.0;
    for (int i = 0; i < NTAP; ++i) {
      double t = i - M;
      double y = 2.0 * FIR_FC / SR * t;
      double s = (y == 0.0) ? 1.0 : std::sin(M_PI * y) / (M_PI * y);
      double r = (i - M) / M;
      double w = bessel_i0(FIR_BETA * std::sqrt(1.0 - r * r)) / i0b;
      fir_[i] = s * w;
      sum += fir_[i];
    }
    for (int i = 0; i < NTAP; ++i) fir_[i] /= sum;
  }

  int8_t ds5_haptic_builder::quantize_one(double d) {
    // TPDF-dithered tanh soft-clip to int8 (ds5_av_capture.cpp). rng_ is only
    // touched on the feed thread, so no lock is needed here.
    auto frand = [this]() -> double {
      uint64_t x = rng_;
      x ^= x << 13; x ^= x >> 7; x ^= x << 17;
      rng_ = x;
      return (double) (x >> 11) * (1.0 / 9007199254740992.0);
    };
    double g = std::tanh(d);
    double dith = frand() - frand();
    double v = std::round(g * 127.0 + dith);
    if (v > 127.0) v = 127.0;
    if (v < -128.0) v = -128.0;
    return (int8_t) v;
  }

  // Overlap-save FIR decimation of the two voice-coil channels by DECIM,
  // accumulating HAP_OUT decimated stereo frames into one 64-byte snapshot.
  void ds5_haptic_builder::process_block(const float *hapL, const float *hapR) {
    const float *prevL = primed_ ? prevHL_.data() : nullptr;
    const float *prevR = primed_ ? prevHR_.data() : nullptr;
    const int ndec = PROC_BLOCK / DECIM;
    for (int j = 0; j < ndec; ++j) {
      int base = j * DECIM;
      double accL = 0.0, accR = 0.0;
      for (int k = 0; k < NTAP; ++k) {
        int idx = base + k;
        float vL, vR;
        if (idx < PROC_BLOCK) {
          vL = prevL ? prevL[idx] : 0.0f;
          vR = prevR ? prevR[idx] : 0.0f;
        } else {
          vL = hapL[idx - PROC_BLOCK];
          vR = hapR[idx - PROC_BLOCK];
        }
        accL += (double) vL * fir_[k];
        accR += (double) vR * fir_[k];
      }
      decL_[dec_n_] = (float) accL;
      decR_[dec_n_] = (float) accR;
      dec_n_++;
      if (dec_n_ >= HAP_OUT) {
        double hrms = 0.0;
        for (int i = 0; i < HAP_OUT; ++i)
          hrms += (double) decL_[i] * decL_[i] + (double) decR_[i] * decR_[i];
        hrms = std::sqrt(hrms / (HAP_OUT * 2));
        std::array<int8_t, HAPTIC_BYTES> snap {};
        for (int i = 0; i < HAP_OUT; ++i) {
          snap[2 * i] = quantize_one(decL_[i]);
          snap[2 * i + 1] = quantize_one(decR_[i]);
        }
        auto now = std::chrono::steady_clock::now();
        dbg_blocks_.fetch_add(1, std::memory_order_relaxed);
        if (hrms > ACTIVE_RMS) dbg_gate_blocks_.fetch_add(1, std::memory_order_relaxed);
        std::lock_guard<std::mutex> lk(hap_mtx_);
        // Latest-wins: overwrite the newest snapshot; the pacer sends it next tick.
        // The outgoing snapshot is retained as prev_frame_ so the batched form can
        // send two CONSECUTIVE coil blocks (21.33 ms) instead of the same block
        // twice; the 0x36 path never reads it.
        // Only a snapshot that is still fresh is a genuine PREDECESSOR of the one
        // arriving now. After a feed gap (game silent, endpoint closed) the old
        // latest_frame_ is seconds stale; carrying it forward unconditionally
        // would make the first 0x39 after every gap replay it as its first
        // 10.67 ms — an audible click at the start of each effect.
        if (have_haptic_ && (now - latest_ts_) < HAPTIC_STALE_BATCHED) {
          std::memcpy(prev_frame_.data(), latest_frame_.data(), HAPTIC_BYTES);
          have_prev_ = true;
        } else {
          have_prev_ = false;
        }
        std::memcpy(latest_frame_.data(), snap.data(), HAPTIC_BYTES);
        latest_ts_ = now;
        have_haptic_ = true;
        if (hrms > ACTIVE_RMS) {
          last_signal_ts_ = now;
          coil_ever_.store(true, std::memory_order_relaxed);
        }
        if (hrms > dbg_max_hrms_) dbg_max_hrms_ = hrms;
        dec_n_ = 0;
      }
    }
    std::memcpy(prevHL_.data(), hapL, sizeof(float) * PROC_BLOCK);
    std::memcpy(prevHR_.data(), hapR, sizeof(float) * PROC_BLOCK);
    primed_ = true;
  }

  void ds5_haptic_builder::feed_pcm(const uint8_t *pcm, size_t len) {
    const int16_t *s = reinterpret_cast<const int16_t *>(pcm);
    size_t nframe = (len / 2) / CHANS;  // 4ch int16 frames
    dbg_feed_frames_.fetch_add(nframe, std::memory_order_relaxed);
    // Voice-coil (ch2/3) -> FIR-decimated haptic snapshot.
    for (size_t f = 0; f < nframe; ++f) {
      const int16_t *fr = s + f * CHANS;
      curL_[cur_n_] = fr[HAP_L] / 32768.0f;
      curR_[cur_n_] = fr[HAP_R] / 32768.0f;
      if (++cur_n_ >= PROC_BLOCK) {
        process_block(curL_.data(), curR_.data());
        cur_n_ = 0;
      }
    }
    // Speaker (ch0/1) -> jitter-buffered for per-tick Opus encoding (Phase 2b).
    spk_push(s, nframe);
  }

  // Extract ch0/1 from the 4ch frames, resample 48 kHz -> drain clock (the DS5's
  // real 0x36 drain clock — see the RATE MATCH note in the header) and push
  // onto the speaker ring, dropping the oldest on overflow so queued latency
  // stays bounded.
  void ds5_haptic_builder::spk_push(const int16_t *pcm4, size_t nframe) {
    // Stage the stereo input (resampler state is feed-thread-only).
    size_t base = rs_pend_.size() / 2;
    rs_pend_.resize((base + nframe) * 2);
    for (size_t i = 0; i < nframe; ++i) {
      const int16_t *fr = pcm4 + i * CHANS;
      rs_pend_[(base + i) * 2] = fr[SPK_L];
      rs_pend_[(base + i) * 2 + 1] = fr[SPK_R];
    }
    // Elastic ratio (audio-clock recovery): one 480-sample frame is consumed
    // per frame period, so the output rate is 480e6/frame_us Hz and the input
    // step per output sample follows the live servo period. The frame period is
    // the pacer tick divided by the frames the tick carries (2 for 0x39).
    const double pace_us = (double) pace_us_.load(std::memory_order_relaxed) /
                           (double) spk_frames_per_tick();
    const double step = (double) SR * pace_us / ((double) OPUS_FRAME * 1e6);
    const size_t npend = rs_pend_.size() / 2;
    const int C = RS_TAPS / 2;  // output at i0+a reads inputs i0-C+1 .. i0+C
    std::lock_guard<std::mutex> lk(spk_mtx_);
    // Emit every output sample whose full tap neighborhood is staged. Blend the
    // two bracketing phase rows so the irrational step never quantizes to a
    // phase grid (a nearest-phase pick would modulate HF by the grid error).
    while ((size_t) rs_pos_ + (size_t) C < npend) {
      const size_t i0 = (size_t) rs_pos_;
      const double ph = (rs_pos_ - (double) i0) * RS_PHASES;
      const int p0 = (int) ph;
      const float pf = (float) (ph - p0);
      const float *r0 = rs_filt_.data() + (size_t) p0 * RS_TAPS;
      const float *r1 = r0 + RS_TAPS;
      const int16_t *in = rs_pend_.data() + (i0 - (C - 1)) * 2;
      float accL = 0.0f, accR = 0.0f;
      for (int k = 0; k < RS_TAPS; ++k) {
        const float c = r0[k] + (r1[k] - r0[k]) * pf;
        accL += c * in[k * 2];
        accR += c * in[k * 2 + 1];
      }
      auto clamp16 = [](float v) -> int16_t {
        if (v > 32767.0f) v = 32767.0f;
        if (v < -32768.0f) v = -32768.0f;
        return (int16_t) v;
      };
      if (spk_count_ == SPK_RING_FRAMES) {
        spk_head_ = (spk_head_ + 1) % SPK_RING_FRAMES;  // full -> drop oldest
        spk_count_--;
      }
      size_t w = (spk_head_ + spk_count_) % SPK_RING_FRAMES;
      spk_ring_[w * 2] = clamp16(accL);
      spk_ring_[w * 2 + 1] = clamp16(accR);
      spk_count_++;
      rs_pos_ += step;
    }
    // Retire consumed input, keeping C-1 frames of left-tap history.
    if ((size_t) rs_pos_ > (size_t) (C - 1)) {
      size_t keep_from = (size_t) rs_pos_ - (C - 1);
      rs_pend_.erase(rs_pend_.begin(), rs_pend_.begin() + keep_from * 2);
      rs_pos_ -= (double) keep_from;
    }
    if (rs_pend_.size() / 2 > 8192) {  // backstop against a stalled consumer
      rs_pend_.clear();
      rs_pos_ = C - 1;
    }
  }

  // Pop exactly OPUS_FRAME stereo frames; 1 on success, 0 if underrun/priming
  // (caller runs PLC). Latency-drain + jitter-buffer prime ported from ds5_av_play.c.
  int ds5_haptic_builder::spk_pop_frame(int16_t out[OPUS_FRAME * 2]) {
    std::lock_guard<std::mutex> lk(spk_mtx_);
    // Latency drain: if the queue ratcheted above the drain threshold, skip whole
    // 10 ms frames back to target so speaker latency stays tight to the haptic.
    // Hysteresis (target < drain) avoids dropping every tick.
    if (spk_count_ > spk_lat_drain_) {
      size_t drop = spk_count_ - spk_lat_target_;
      drop -= drop % OPUS_FRAME;
      spk_head_ = (spk_head_ + drop) % SPK_RING_FRAMES;
      spk_count_ -= drop;
    }
    // Prime: after start/underrun, hold silence until a target cushion rebuilds,
    // so one late chunk can't re-trigger underruns mid-effect.
    if (spk_priming_) {
      if (spk_count_ < spk_lat_target_) return 0;
      spk_priming_ = false;
    }
    if (spk_count_ < (size_t) OPUS_FRAME) {
      spk_priming_ = true;
      return 0;
    }
    for (int i = 0; i < OPUS_FRAME; ++i) {
      size_t r = (spk_head_ + i) % SPK_RING_FRAMES;
      out[i * 2] = spk_ring_[r * 2];
      out[i * 2 + 1] = spk_ring_[r * 2 + 1];
    }
    spk_head_ = (spk_head_ + OPUS_FRAME) % SPK_RING_FRAMES;
    spk_count_ -= OPUS_FRAME;
    return 1;
  }

  // Fixed 398-byte skeleton: sub-packet headers + audio SetState. seq (buf[1]),
  // pktctr (buf[10]), the 0x12 haptic payload, the 0x13 Opus payload and the CRC
  // are written per tick in build_0x36 (ds5_av_play.c report layout).
  void ds5_haptic_builder::build_skeleton() {
    skeleton_.fill(0);
    uint8_t *buf = skeleton_.data();
    buf[0] = 0x36;                                     // report id
    // 0x91 timing sub-packet (CTM parity): payload = [0xFE][latency ms x5],
    // plus the audio sequence counter at buf[10] (pktctr_, advanced per send).
    // The five latency bytes size the pad's own audio jitter buffer; the TV
    // client live-patches them to its latency slider (default 100 ms) on every
    // outbound 0x36, so this value is only on-air when that patch is off —
    // match the slider default rather than the old 255 (max buffering).
    buf[2] = 0x11 | 0x80; buf[3] = 7; buf[4] = 0xFE;
    buf[5] = buf[6] = buf[7] = buf[8] = buf[9] = 100;
    buf[11] = 0x10 | 0x80; buf[12] = 63;               // SetState (0x10) sub-packet
    std::memcpy(buf + OFF_SETSTATE, state_audio_data, 63);
    buf[76] = 0x12 | 0x80; buf[77] = HAPTIC_BYTES;     // voice-coil (0x12) sub-packet
    buf[142] = 0x13 | 0x80; buf[143] = OPUS_BYTES;     // Opus speaker (0x13) sub-packet

    // Batched 547-byte 0x39 skeleton. Differences to 0x36, all forced by the
    // 546-byte payload ceiling:
    //  - the 0x91 timing payload is the SHORT form (6 bytes: presence bitmask +
    //    four buffer-length bytes + the audio counter at buf[9]) instead of
    //    [0xFE][latency x5][counter],
    //  - the coil and Opus sub-blocks set bit 6 in their id (0xD2 / 0xD3) to mark
    //    a doubled payload while the length byte keeps naming ONE block, and
    //  - there is no room for the SetState block, so the audio Allow bits ride a
    //    separate periodic 0x32 (build_setstate_0x32).
    skeleton39_.fill(0);
    uint8_t *b39 = skeleton39_.data();
    b39[0] = 0x39;
    b39[2] = 0x11 | 0x80; b39[3] = 6; b39[4] = 0x7E;
    b39[5] = b39[6] = b39[7] = b39[8] = 100;           // latency ms (TV patches [5..8])
    b39[10] = 0x12 | 0xC0; b39[11] = HAPTIC_BYTES;     // two coil blocks follow
    b39[140] = 0x13 | 0xC0; b39[141] = OPUS_BYTES;     // two Opus frames follow
  }

  void ds5_haptic_builder::set_audio_control(uint8_t v) {
    audio_control_ = v;
    skeleton_[OFF_SETSTATE + 7] = v;   // state_audio_data[7]
  }

  // DS5 BT output CRC: CRC32 over the 0xA2 seed byte + bytes [0 .. len-4).
  void ds5_haptic_builder::sign_report(uint8_t *out, int len) {
    const uint8_t seed = PS_OUTPUT_CRC_SEED;
    uint32_t crc = ds5_crc32_update(0xFFFFFFFFu, &seed, 1);
    crc = ds5_crc32_update(crc, out, len - 4);
    crc = ~crc;
    out[len - 4] = (uint8_t) (crc & 0xFF);
    out[len - 3] = (uint8_t) ((crc >> 8) & 0xFF);
    out[len - 2] = (uint8_t) ((crc >> 16) & 0xFF);
    out[len - 1] = (uint8_t) ((crc >> 24) & 0xFF);
  }

  // Standalone audio SetState. Same 63-byte payload the 0x36 carries inline, in
  // the 142-byte 0x32 report the pad also accepts (0x32 = the 141-byte rung of
  // the same output-report ladder). Sent rarely and re-asserted, so a lost one
  // self-heals; the TV patches its volume/route bytes exactly as it does for the
  // inline block (ds5_patch_output already accepts 0x32).
  void ds5_haptic_builder::build_setstate_0x32(uint8_t out[DS5_0X32_LEN]) {
    std::memset(out, 0, DS5_0X32_LEN);
    out[0] = 0x32;
    out[1] = (uint8_t) ((out_seq_ & 0x0F) << 4);
    out_seq_++;
    out[2] = 0x10 | 0x80; out[3] = 63;
    std::memcpy(out + 4, state_audio_data, 63);
    out[4 + 7] = audio_control_;
    sign_report(out, DS5_0X32_LEN);
  }

  // One speaker frame into @p dst: pop 480 stereo (or conceal), Opus-encode.
  // Split out of build_0x36 unchanged so the batched form can call it twice —
  // the encoder is stateful, so two frames per report must be encoded in order
  // through the SAME encoder, which is exactly what two calls do.
  void ds5_haptic_builder::encode_speaker_frame(uint8_t *dst, std::chrono::steady_clock::time_point now) {
    // ---- Speaker (0x13): pop 480 stereo, PLC on underrun, Opus-encode. -----
    // Encode EVERY tick so the encoder state stays continuous and in-order
    // (Opus is stateful — a dropped/duplicated frame desyncs the DS5 decoder).
    int16_t pcm_i[OPUS_FRAME * 2];
    float pcm_f[OPUS_FRAME * 2];
    if (spk_pop_frame(pcm_i)) {
      double srms = 0.0;
      for (int i = 0; i < OPUS_FRAME * 2; ++i) {
        pcm_f[i] = pcm_i[i] / 32768.0f;
        srms += (double) pcm_f[i] * pcm_f[i];
      }
      srms = std::sqrt(srms / (OPUS_FRAME * 2));
      // last_pcm_f_ keeps the CLEAN frame on purpose: concealment continues the
      // signal, not the resume ramp that sits on top of it below.
      std::memcpy(last_pcm_f_.data(), pcm_f, sizeof(pcm_f));
      // Arm the ramp on the seam OUT of concealment.
      if (plc_run_ > 0) spk_resume_left_ = SPK_RESUME_RAMP;
      plc_run_ = 0;
      const float from = spk_env_;
      for (int i = 0; i < OPUS_FRAME && spk_resume_left_ > 0; ++i) {
        const float t = 1.0f - (float) spk_resume_left_ / (float) SPK_RESUME_RAMP;
        spk_env_ = from + (1.0f - from) * t;
        pcm_f[i * 2] *= spk_env_;
        pcm_f[i * 2 + 1] *= spk_env_;
        spk_resume_left_--;
      }
      spk_env_ = 1.0f;  // the rest of the frame, and every clean frame, is unity
      // Activity keys on AMPLITUDE, not on buffer state: usbaudio streams
      // digital silence continuously whenever any session holds the endpoint
      // open, so "a frame was popped" is true forever. Un-gated, that kept
      // should_send high at 100/s for the whole session — the continuous
      // reliable 0x36 flood behind the Phase 2b link-drop regression. Squelch
      // from ds5_av_capture.cpp (the reference gated at the supply instead).
      if (srms > SPK_RMS) last_spk_ts_ = now;
      dbg_spk_pop_.fetch_add(1, std::memory_order_relaxed);
    } else {
      // Packet-loss concealment: a decayed copy of the last good frame. Hard
      // silence would click; this fades a sustained gap smoothly to zero over
      // ~8 frames. The 0x13 stays a valid 200-byte Opus frame either way.
      float g = 1.0f;
      for (int k = 0; k <= plc_run_; ++k) g *= 0.6f;
      if (g < 0.02f) g = 0.0f;
      // Slide from wherever the envelope stands to this frame's decay target
      // instead of stepping onto it. The old code applied g flat across all 480
      // samples, so entering concealment from a clean frame was a 1.0 -> 0.6
      // jump — the same class of click as the resume seam, just quieter.
      const float from = spk_env_;
      for (int i = 0; i < OPUS_FRAME; ++i) {
        const float t = (float) (i + 1) / (float) OPUS_FRAME;
        const float e = from + (g - from) * t;
        pcm_f[i * 2] = last_pcm_f_[i * 2] * e;
        pcm_f[i * 2 + 1] = last_pcm_f_[i * 2 + 1] * e;
      }
      spk_env_ = g;
      plc_run_++;
      // Abandon any resume ramp in flight: an underrun inside the 5 ms window
      // means concealment owns the envelope again, and it now picks it up from
      // exactly where the ramp had got to.
      spk_resume_left_ = 0;
      dbg_spk_plc_.fetch_add(1, std::memory_order_relaxed);
    }
    if (enc_) {
      int n = opus_encode_float(enc_, pcm_f, OPUS_FRAME, dst, OPUS_BYTES);
      if (n < 0) std::memset(dst, 0, OPUS_BYTES);
      else if (n < OPUS_BYTES) std::memset(dst + n, 0, OPUS_BYTES - n);
    }
  }

  bool ds5_haptic_builder::build_0x36(uint8_t out[DS5_0X36_LEN]) {
    std::memcpy(out, skeleton_.data(), DS5_0X36_LEN);

    auto now = std::chrono::steady_clock::now();

    // ---- Speaker (0x13): one frame per tick (see encode_speaker_frame). ---
    encode_speaker_frame(out + OFF_OPUS, now);

    // Active while audible signal is inside the grace tail AND the ring is not
    // in a sustained underrun (SPK_CONCEAL caps how long PLC alone may keep
    // driving the link if the feed stalls — CTM conceals ~8 frames, then quiets).
    bool spk_active = (now - last_spk_ts_) < GRACE && plc_run_ <= SPK_CONCEAL;

    // ---- Haptic (0x12): latest-wins, fresh within staleness ---------------
    bool hap_active = false;
    {
      std::lock_guard<std::mutex> lk(hap_mtx_);
      hap_active = have_haptic_ && (now - last_signal_ts_) < GRACE;
      bool fresh = have_haptic_ && (now - latest_ts_) < HAPTIC_STALE;
      if (hap_active && fresh) {
        std::memcpy(out + OFF_HAPTIC, latest_frame_.data(), HAPTIC_BYTES);
      } else if (hap_active) {
        // Within grace but the feed went stale -> zeroed coil (skeleton's 0x12 is
        // already zero) rather than buzz a frozen frame.
        dbg_stale_.fetch_add(1, std::memory_order_relaxed);
      }
      // else (idle): the skeleton's zeroed 0x12 passes through -> silence.
    }

    // ---- Idle gate: drive the DS5 while haptic OR speaker is active --------
    bool should_send = hap_active || spk_active;
    if (should_send) {
      dbg_sends_.fetch_add(1, std::memory_order_relaxed);
      // Advance the BT seq (buf[1]) and audio counter (buf[10]) ONLY on a report
      // that is actually sent, so the DS5 sees contiguous counters across idle
      // gaps (holes glitch its Opus playback on resume — ds5_av_play.c does this).
      out[1] = (uint8_t) ((out_seq_ & 0x0F) << 4);
      out_seq_++;
      out[10] = pktctr_++;
      sign_report(out, DS5_0X36_LEN);
    }

    if (++dbg_build_ctr_ >= 200) {  // ~every 2 s at 100 Hz
      dbg_build_ctr_ = 0;
      double maxh;
      { std::lock_guard<std::mutex> lk(hap_mtx_); maxh = dbg_max_hrms_; dbg_max_hrms_ = 0.0; }
      BOOST_LOG(info) << "ds5-haptics: feedframes=" << dbg_feed_frames_.exchange(0)
                      << " blocks=" << dbg_blocks_.exchange(0)
                      << " gate=" << dbg_gate_blocks_.exchange(0)
                      << " sends=" << dbg_sends_.exchange(0)
                      << " stale=" << dbg_stale_.exchange(0)
                      << " spkpop=" << dbg_spk_pop_.exchange(0)
                      << " spkplc=" << dbg_spk_plc_.exchange(0)
                      << " maxhrms=" << maxh;
    }
    return should_send;
  }

  // Batched form: one report = two consecutive 0x36 ticks. Everything that makes
  // the 0x36 path correct is preserved and simply applied twice:
  //  - two IN-ORDER encoder calls (Opus is stateful; skipping or reordering a
  //    frame desyncs the pad's decoder),
  //  - the coil pair is [prev, latest] so 21.33 ms of real signal goes out rather
  //    than the newest block played twice,
  //  - the audio counter advances by TWO, matching the two frames carried, and
  //  - the idle gate, grace window and seq handling are unchanged.
  bool ds5_haptic_builder::build_0x39(uint8_t out[DS5_0X39_LEN]) {
    std::memcpy(out, skeleton39_.data(), DS5_0X39_LEN);

    auto now = std::chrono::steady_clock::now();

    encode_speaker_frame(out + OFF39_OPUS_A, now);
    encode_speaker_frame(out + OFF39_OPUS_B, now);

    bool spk_active = (now - last_spk_ts_) < GRACE && plc_run_ <= SPK_CONCEAL;

    // Staleness window scales with the report period (HAPTIC_STALE_BATCHED).
    bool hap_active = false;
    {
      std::lock_guard<std::mutex> lk(hap_mtx_);
      hap_active = have_haptic_ && (now - last_signal_ts_) < GRACE;
      bool fresh = have_haptic_ && (now - latest_ts_) < HAPTIC_STALE_BATCHED;
      if (hap_active && fresh) {
        // Without a previous block (first report after a gap) the older half
        // stays silent rather than duplicating the newer one.
        if (have_prev_) {
          std::memcpy(out + OFF39_HAPTIC_A, prev_frame_.data(), HAPTIC_BYTES);
        }
        std::memcpy(out + OFF39_HAPTIC_B, latest_frame_.data(), HAPTIC_BYTES);
      } else if (hap_active) {
        dbg_stale_.fetch_add(1, std::memory_order_relaxed);
      }
    }

    bool should_send = hap_active || spk_active;
    if (should_send) {
      dbg_sends_.fetch_add(1, std::memory_order_relaxed);
      out[1] = (uint8_t) ((out_seq_ & 0x0F) << 4);
      out_seq_++;
      out[9] = pktctr_;
      pktctr_ = (uint8_t) (pktctr_ + 2);
      sign_report(out, DS5_0X39_LEN);
    }

    if (++dbg_build_ctr_ >= 100) {  // ~every 2 s at 47 Hz
      dbg_build_ctr_ = 0;
      double maxh;
      { std::lock_guard<std::mutex> lk(hap_mtx_); maxh = dbg_max_hrms_; dbg_max_hrms_ = 0.0; }
      BOOST_LOG(info) << "ds5-haptics(0x39): feedframes=" << dbg_feed_frames_.exchange(0)
                      << " blocks=" << dbg_blocks_.exchange(0)
                      << " gate=" << dbg_gate_blocks_.exchange(0)
                      << " sends=" << dbg_sends_.exchange(0)
                      << " stale=" << dbg_stale_.exchange(0)
                      << " spkpop=" << dbg_spk_pop_.exchange(0)
                      << " spkplc=" << dbg_spk_plc_.exchange(0)
                      << " maxhrms=" << maxh;
    }
    return should_send;
  }

}  // namespace platf::ds5_bridge
