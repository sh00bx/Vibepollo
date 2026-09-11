/**
 * @file src/platform/windows/ds5_bridge/ds4_audio.cpp
 * @brief DS4 SBC audio builder (see header).
 */
#include "src/platform/windows/ds5_bridge/ds4_audio.h"

#include <chrono>
#include <cstring>

#include <sbc/sbc.h>

#include "src/logging.h"

using namespace std::literals;

namespace platf::ds5_bridge {

  namespace {
    int64_t now_ms() {
      return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
    }

    /// The probed wire shape. Applied after every sbc_init/sbc_reinit, both of
    /// which reset the struct to library defaults.
    void apply_wire_shape(sbc_t *sbc) {
      sbc->frequency = SBC_FREQ_32000;
      sbc->blocks = SBC_BLK_16;
      sbc->subbands = SBC_SB_8;
      sbc->mode = SBC_MODE_JOINT_STEREO;
      sbc->allocation = SBC_AM_LOUDNESS;
      sbc->bitpool = 48;
      sbc->endian = SBC_LE;
    }
  }  // namespace

  ds4_audio_builder::ds4_audio_builder() {
    ring_.reserve(RING_CAP_FRAMES * 2);
    auto *sbc = new sbc_t;
    if (sbc_init(sbc, 0) != 0) {
      delete sbc;
      BOOST_LOG(error) << "ds4-audio: sbc_init failed; DS4 speaker audio disabled"sv;
      return;
    }
    apply_wire_shape(sbc);
    sbc_ = sbc;
    // Sanity: the probed wire geometry depends on this exact shape.
    const int codesize = (int) sbc_get_codesize(sbc);
    const int framelen = (int) sbc_get_frame_length(sbc);
    if (codesize != DS4_SBC_PCM_PER_FRAME * 2 * 2 || framelen != DS4_SBC_FRAME_BYTES) {
      BOOST_LOG(error) << "ds4-audio: unexpected SBC geometry (codesize="sv << codesize
                       << ", framelen="sv << framelen << "); DS4 speaker audio disabled"sv;
      sbc_finish(sbc);
      delete sbc;
      sbc_ = nullptr;
    }
  }

  /* The analysis filterbank carries eight blocks of input history. A stream
   * that resumes after a gap would otherwise encode its first frames against
   * history from before the gap, which rings as a click on the very transient
   * (an effect's attack) the game cares most about. Re-arming is cheap and
   * happens at most once per stream. */
  void ds4_audio_builder::reset_encoder_locked() {
    if (!sbc_) return;
    auto *sbc = (sbc_t *) sbc_;
    if (sbc_reinit(sbc, 0) != 0) {
      // Encoder is unusable from here; the geometry check below would have to
      // fail too, so drop the path rather than emit garbage SBC.
      BOOST_LOG(error) << "ds4-audio: sbc_reinit failed; DS4 speaker audio disabled"sv;
      sbc_finish(sbc);
      delete sbc;
      sbc_ = nullptr;
      return;
    }
    apply_wire_shape(sbc);
  }

  ds4_audio_builder::~ds4_audio_builder() {
    if (sbc_) {
      auto *sbc = (sbc_t *) sbc_;
      sbc_finish(sbc);
      delete sbc;
    }
  }

  void ds4_audio_builder::feed_pcm(const uint8_t *pcm, size_t len) {
    if (!pcm || len < 4) return;
    const auto *s = (const int16_t *) pcm;
    size_t frames = len / 4;  // stereo int16
    bool energy = false;
    for (size_t i = 0; i < frames * 2; ++i) {
      // 16 LSBs of headroom count as silence: Windows' mixer dithers.
      if (s[i] > 16 || s[i] < -16) { energy = true; break; }
    }
    std::lock_guard<std::mutex> lk(mtx_);
    const int64_t now = now_ms();
    last_feed_ms_ = now;
    if (energy) last_energy_ms_ = now;
    size_t have = ring_.size() / 2;
    if (have + frames > (size_t) RING_CAP_FRAMES) {
      // Stall recovery: drop the oldest down to the trim level in one cut so
      // playback re-centers instead of running 300 ms late for the session.
      size_t target = (size_t) RING_TRIM_FRAMES;
      size_t drop = have + frames > target ? (have + frames - target) : 0;
      if (drop > have) drop = have;
      ring_.erase(ring_.begin(), ring_.begin() + (ptrdiff_t) (drop * 2));
      dropped_.fetch_add(drop, std::memory_order_relaxed);
    }
    ring_.insert(ring_.end(), s, s + frames * 2);
  }

  bool ds4_audio_builder::build_0x17(uint8_t out[DS4_0X17_LEN], uint8_t route) {
    if (!sbc_) return false;
    int16_t pcm[DS4_AUDIO_PCM_PER_REPORT * 2];
    {
      std::lock_guard<std::mutex> lk(mtx_);
      const int64_t now = now_ms();
      const bool fed = (now - last_feed_ms_) < FEED_IDLE_MS;
      const bool audible = (now - last_energy_ms_) < SILENCE_HANGOVER_MS;
      size_t have = ring_.size() / 2;
      // Over only once the endpoint has gone quiet AND the ring is dry. While
      // frames remain we keep draining them and the last, partial report is
      // padded below instead of dropped, so an effect is never clipped short of
      // its own tail.
      const bool source_over = !fed && have == 0;

      if (source_over || !audible) {
        // Re-arm the warm-up either way: the squelch keeps draining the ring in
        // real time, so without this the NEXT stream would start with nothing
        // buffered and run its whole life one hiccup from an underrun.
        warmed_ = false;
        if (source_over) {
          ring_.clear();
        } else {
          // Squelched while the endpoint still feeds: consume in real time so
          // the ring cannot age and a resume is instant, but put no SBC-silence
          // STREAM on the air (see SILENCE_TAIL_REPORTS).
          size_t take = have < (size_t) DS4_AUDIO_PCM_PER_REPORT ? have : (size_t) DS4_AUDIO_PCM_PER_REPORT;
          ring_.erase(ring_.begin(), ring_.begin() + (ptrdiff_t) (take * 2));
        }
        if (streaming_) {
          // First tick after real audio: owe the pad a silence flush so what it
          // keeps repeating out of its buffer is silence, not the last effect.
          streaming_ = false;
          tail_left_ = SILENCE_TAIL_REPORTS + STOP_REPORTS;
          prime_left_.store(0, std::memory_order_relaxed);
          flushed_.fetch_add(1, std::memory_order_relaxed);
          if (source_over) flushed_idle_.fetch_add(1, std::memory_order_relaxed);
        }
        if (tail_left_ <= 0) return false;
        tail_left_--;
        if (tail_left_ < STOP_REPORTS) route = DS4_ROUTE_NONE;
        if (tail_left_ == 0) enc_stale_ = true;
        std::memset(pcm, 0, sizeof pcm);
      } else {
        if (!warmed_) {
          // Only wait for the warm-up cushion while the endpoint is still
          // feeding; a drained stream's remainder plays out as-is.
          if (fed && have < (size_t) WARMUP_FRAMES) return false;
          warmed_ = true;
        }
        if (enc_stale_) {
          reset_encoder_locked();
          if (!sbc_) return false;
          enc_stale_ = false;
        }
        if (!streaming_) {
          // New stream: hand the pad its cushion before settling on the grid.
          streaming_ = true;
          prime_left_.store(PRIME_REPORTS, std::memory_order_relaxed);
        }
        tail_left_ = 0;
        if (prime_left_.load(std::memory_order_relaxed) > 0) {
          // Burst: silence, and the ring keeps every frame it holds so the
          // cushion the warm-up just built is still there for steady state.
          std::memset(pcm, 0, sizeof pcm);
        } else {
          size_t take = have < (size_t) DS4_AUDIO_PCM_PER_REPORT ? have : (size_t) DS4_AUDIO_PCM_PER_REPORT;
          std::memcpy(pcm, ring_.data(), take * 2 * sizeof(int16_t));
          if (take < (size_t) DS4_AUDIO_PCM_PER_REPORT) {
            // Underrun inside an active stream: pad with silence rather than
            // skipping the report — a missing 16 ms report is a harder click
            // than a silence tail.
            std::memset(pcm + take * 2, 0, (DS4_AUDIO_PCM_PER_REPORT - take) * 2 * sizeof(int16_t));
            underrun_.fetch_add(1, std::memory_order_relaxed);
          }
          ring_.erase(ring_.begin(), ring_.begin() + (ptrdiff_t) (take * 2));
        }
      }
    }

    auto *sbc = (sbc_t *) sbc_;
    uint8_t sbc_payload[DS4_AUDIO_SBC_BYTES];
    const uint8_t *in = (const uint8_t *) pcm;
    for (int f = 0; f < DS4_SBC_FRAMES_PER_REPORT; ++f) {
      ssize_t written = 0;
      ssize_t consumed = sbc_encode(sbc, in + f * DS4_SBC_PCM_PER_FRAME * 4,
                                    DS4_SBC_PCM_PER_FRAME * 4,
                                    sbc_payload + f * DS4_SBC_FRAME_BYTES,
                                    DS4_SBC_FRAME_BYTES, &written);
      if (consumed <= 0 || written != DS4_SBC_FRAME_BYTES) {
        return false;  // encoder misbehaved; skip this report rather than send garbage
      }
    }
    ds4_build_audio_0x17(sbc_payload, ctr_, route, out);
    ctr_ = (uint16_t) (ctr_ + DS4_SBC_FRAMES_PER_REPORT);
    built_.fetch_add(1, std::memory_order_relaxed);
    int prime = prime_left_.load(std::memory_order_relaxed);
    last_primed_ = prime > 0;
    if (last_primed_) {
      prime_left_.store(prime - 1, std::memory_order_relaxed);
      primed_.fetch_add(1, std::memory_order_relaxed);
    }
    return true;
  }

}  // namespace platf::ds5_bridge
