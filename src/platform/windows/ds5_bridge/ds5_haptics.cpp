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

#include "src/platform/windows/ds5_bridge/ds5_haptics.h"
#include "src/platform/windows/ds5_bridge/ds5_reports.h"

namespace platf::ds5_bridge {

  namespace {
    // Report 0x36 sub-packet offsets (canonical, from ds5_av_play.c).
    constexpr int OFF_SETSTATE = 13;   // SetState (0x10) payload
    constexpr int OFF_HAPTIC = 78;     // voice-coil (0x12) payload
    constexpr int OFF_OPUS = 144;      // Opus speaker (0x13) payload
    constexpr int OPUS_BYTES = 200;
    constexpr int OPUS_FRAME = 480;    // 10 ms @48k stereo (samples per channel)

    // Non-destructive "audio-only" SetState (0x10) payload: asserts only the
    // audio Allow bits so it never fights Moonlight's own trigger/rumble/LED
    // writes on the same pad (ds5_av_play.c, DS5Dongle state layout). 63 bytes.
    const uint8_t state_audio_data[63] = {
      0xB0, 0x82,                 // ValidFlags: audio-only
      0x00, 0x00,                 // RumbleEmulation R/L (not allowed)
      0x7f, 0x7f,                 // VolumeHeadphones=127, VolumeSpeaker=127
      0x00,                       // VolumeMic (not allowed)
      0x00,                       // AudioControl (Auto mic / default)
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
    build_skeleton();
    // last_signal_ts_ defaults to the epoch and have_haptic_ starts false, so
    // build_0x36 idle-gates (sends nothing) until the first real signal block.
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
        // Always publish the block (continuity: an active effect must stream every
        // ~10.7 ms with no RMS-gate dropouts, or the coil actuation is choppy). The
        // squelch only decides idle-gating, not whether to store.
        std::array<int8_t, HAPTIC_BYTES> snap {};
        for (int i = 0; i < HAP_OUT; ++i) {
          snap[2 * i] = quantize_one(decL_[i]);
          snap[2 * i + 1] = quantize_one(decR_[i]);
        }
        auto now = std::chrono::steady_clock::now();
        std::lock_guard<std::mutex> lk(hap_mtx_);
        latest_haptic_ = snap;
        latest_haptic_ts_ = now;
        have_haptic_ = true;
        if (hrms > ACTIVE_RMS) last_signal_ts_ = now;
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
    for (size_t f = 0; f < nframe; ++f) {
      const int16_t *fr = s + f * CHANS;
      curL_[cur_n_] = fr[HAP_L] / 32768.0f;
      curR_[cur_n_] = fr[HAP_R] / 32768.0f;
      if (++cur_n_ >= PROC_BLOCK) {
        process_block(curL_.data(), curR_.data());
        cur_n_ = 0;
      }
    }
  }

  // Encode one 10 ms Opus silence frame into the 0x13 slot of the skeleton. The
  // DS5 rejects a malformed 0x13, so even the (Phase-2a) silent speaker needs a
  // valid Opus payload. LOWDELAY/CELT-only, matching ds5_av_play.c's encoder.
  void ds5_haptic_builder::encode_opus_silence() {
    uint8_t *slot = skeleton_.data() + OFF_OPUS;
    std::memset(slot, 0, OPUS_BYTES);
    int err = 0;
    OpusEncoder *enc = opus_encoder_create(SR, 2, OPUS_APPLICATION_RESTRICTED_LOWDELAY, &err);
    if (err != OPUS_OK || !enc) {
      if (enc) opus_encoder_destroy(enc);
      return;  // zero-filled fallback (worst case: speaker click, haptic unaffected)
    }
    opus_encoder_ctl(enc, OPUS_SET_EXPERT_FRAME_DURATION(OPUS_FRAMESIZE_10_MS));
    opus_encoder_ctl(enc, OPUS_SET_BITRATE(160000));
    opus_encoder_ctl(enc, OPUS_SET_VBR(0));
    opus_encoder_ctl(enc, OPUS_SET_COMPLEXITY(0));
    float silence[OPUS_FRAME * 2] = {0};
    int n = opus_encode_float(enc, silence, OPUS_FRAME, slot, OPUS_BYTES);
    if (n > 0 && n < OPUS_BYTES) std::memset(slot + n, 0, OPUS_BYTES - n);
    opus_encoder_destroy(enc);
  }

  // Fixed 398-byte skeleton: sub-packet headers + audio SetState + Opus silence.
  // Only seq (buf[1]), pktctr (buf[10]), the haptic payload and the CRC change
  // per tick (ds5_av_play.c report layout).
  void ds5_haptic_builder::build_skeleton() {
    skeleton_.fill(0);
    uint8_t *buf = skeleton_.data();
    buf[0] = 0x36;                                     // report id
    buf[2] = 0x11 | 0x80; buf[3] = 7; buf[4] = 0xFE;   // config (0x11) sub-packet
    buf[5] = buf[6] = buf[7] = buf[8] = buf[9] = 255;  // audio_buffer_length=255
    buf[11] = 0x10 | 0x80; buf[12] = 63;               // SetState (0x10) sub-packet
    std::memcpy(buf + OFF_SETSTATE, state_audio_data, 63);
    buf[76] = 0x12 | 0x80; buf[77] = HAPTIC_BYTES;     // voice-coil (0x12) sub-packet
    buf[142] = 0x13 | 0x80; buf[143] = OPUS_BYTES;     // Opus speaker (0x13) sub-packet
    encode_opus_silence();
  }

  bool ds5_haptic_builder::build_0x36(uint8_t out[DS5_0X36_LEN]) {
    std::memcpy(out, skeleton_.data(), DS5_0X36_LEN);
    out[1] = (uint8_t) ((out_seq_ & 0x0F) << 4);
    out_seq_++;
    out[10] = pktctr_++;

    bool should_send = false;
    {
      std::lock_guard<std::mutex> lk(hap_mtx_);
      auto now = std::chrono::steady_clock::now();
      if (have_haptic_ && (now - latest_haptic_ts_) <= HAPTIC_STALE) {
        for (int i = 0; i < HAPTIC_BYTES; ++i)
          out[OFF_HAPTIC + i] = (uint8_t) latest_haptic_[i];
      }
      // else: the skeleton's zeroed 0x12 payload is copied through -> silence.
      // Idle-gate on real signal activity (not snapshot freshness): continuous
      // near-zero blocks during a lull still stop within GRACE, no idle buzz.
      should_send = have_haptic_ && (now - last_signal_ts_) < GRACE;
    }

    // DS5 BT output CRC: CRC32 over the 0xA2 seed byte + bytes [0 .. len-4).
    const uint8_t seed = PS_OUTPUT_CRC_SEED;
    uint32_t crc = ds5_crc32_update(0xFFFFFFFFu, &seed, 1);
    crc = ds5_crc32_update(crc, out, DS5_0X36_LEN - 4);
    crc = ~crc;
    out[394] = (uint8_t) (crc & 0xFF);
    out[395] = (uint8_t) ((crc >> 8) & 0xFF);
    out[396] = (uint8_t) ((crc >> 16) & 0xFF);
    out[397] = (uint8_t) ((crc >> 24) & 0xFF);
    return should_send;
  }

}  // namespace platf::ds5_bridge
