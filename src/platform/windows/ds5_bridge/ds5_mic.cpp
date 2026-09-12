/**
 * @file src/platform/windows/ds5_bridge/ds5_mic.cpp
 * @brief DS5 microphone uplink decoder + per-session PCM ring (see header).
 */
#include <algorithm>
#include <chrono>
#include <cstring>

#include <opus/opus.h>

#include "src/logging.h"
#include "src/platform/windows/ds5_bridge/ds5_mic.h"

namespace platf::ds5_bridge {

  namespace {
    uint64_t mic_now_us() {
      using namespace std::chrono;
      return (uint64_t) duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
    }
  }  // namespace

  ds5_mic_uplink::ds5_mic_uplink() {
    int err = OPUS_OK;
    dec_ = opus_decoder_create(DS5_MIC_RATE, DS5_MIC_CHANNELS, &err);
    if (!dec_ || err != OPUS_OK) {
      if (dec_) {
        opus_decoder_destroy(dec_);
        dec_ = nullptr;
      }
      BOOST_LOG(error) << "ds5-mic: opus_decoder_create failed (" << opus_strerror(err)
                       << "); microphone uplink disabled for this session";
    }
    ring_.assign(RING_CAP, 0);
    pcm_.assign((size_t) DS5_MIC_FRAME_SAMPLES * DS5_MIC_CHANNELS, 0);
  }

  ds5_mic_uplink::~ds5_mic_uplink() {
    if (dec_) opus_decoder_destroy(dec_);
  }

  // mtx_ held. Append `frames` interleaved stereo frames; on overflow discard
  // the OLDEST audio (what someone is waiting to hear is the newest; keeping
  // stale samples would grow the delay for the rest of the stream).
  void ds5_mic_uplink::push_locked(const int16_t *pcm, int frames) {
    if (frames <= 0) return;
    const size_t bytes = (size_t) frames * DS5_MIC_BYTES_PER_FRAME;
    if (bytes > RING_CAP) return;  // cannot happen (one frame << capacity)
    if (count_ + bytes > RING_CAP) {
      const size_t excess = count_ + bytes - RING_CAP;
      head_ = (head_ + excess) % RING_CAP;
      count_ -= excess;
      st_.drop_bytes += excess;
    }
    const uint8_t *src = reinterpret_cast<const uint8_t *>(pcm);
    size_t tail = (head_ + count_) % RING_CAP;
    const size_t first = std::min(bytes, RING_CAP - tail);
    std::memcpy(ring_.data() + tail, src, first);
    if (first < bytes) std::memcpy(ring_.data(), src + first, bytes - first);
    count_ += bytes;
    if (!primed_ && count_ >= PREBUFFER) primed_ = true;
  }

  // mtx_ held. Synthesize `frames` x 10 ms of concealment from the decoder
  // state (opus_decode with a NULL packet), keeping the ring's timeline
  // continuous across a lost packet so the next real one lands on time.
  void ds5_mic_uplink::plc_locked(int frames) {
    for (int i = 0; i < frames; ++i) {
      const int n = opus_decode(dec_, nullptr, 0, pcm_.data(), DS5_MIC_FRAME_SAMPLES, 0);
      if (n <= 0) break;
      push_locked(pcm_.data(), n);
      ++st_.plc;
    }
  }

  void ds5_mic_uplink::feed(uint16_t seq, const uint8_t *opus, size_t len) {
    if (!dec_ || !opus || len == 0 || len > (size_t) DS5_MIC_MAX_OPUS) return;
    // One 10 ms packet at 48 kHz, nothing else: the ring's clock is 480
    // frames per packet, and a foreign packet (a 20 ms one, a wrong-rate one)
    // would stretch or squeeze the timeline rather than just sound wrong.
    if (opus_packet_get_nb_samples(opus, (opus_int32) len, DS5_MIC_RATE) != DS5_MIC_FRAME_SAMPLES) {
      std::lock_guard<std::mutex> lk(mtx_);
      ++st_.bad;
      return;
    }
    const uint64_t now = mic_now_us();
    std::lock_guard<std::mutex> lk(mtx_);
    int gap_frames = 0;
    if (live_ && now - last_arrival_us_ <= IDLE_US) {
      // Loss by the TV's numbering (frames the TV saw but the network lost)
      // and by time (frames the pad sent but the TV never saw -- its 4-bit
      // report sequence is shared with pad state, so this is the only
      // signal for those). Take the larger, cap it: a long gap is a pause,
      // and five frames of extrapolated speech already sound like a smear.
      const int by_seq = (int) ((uint16_t) (seq - last_seq_)) - 1;
      const uint64_t elapsed = now - last_arrival_us_;
      const int by_time = elapsed > PLC_GAP_US ? (int) ((elapsed - 10000) / 10000) : 0;
      gap_frames = std::clamp(std::max(by_seq, by_time), 0, PLC_MAX_FRAMES);
    } else {
      // Idle -> live: a fresh stream. The decoder's state belongs to the old
      // one (Opus packets predict from their predecessors), and so does
      // anything still queued from it.
      opus_decoder_ctl(dec_, OPUS_RESET_STATE);
      head_ = 0;
      count_ = 0;
      primed_ = false;
      live_ = true;
      ++st_.starts;
    }
    if (gap_frames > 0) plc_locked(gap_frames);
    const int n = opus_decode(dec_, opus, (opus_int32) len, pcm_.data(), DS5_MIC_FRAME_SAMPLES, 0);
    if (n > 0) {
      push_locked(pcm_.data(), n);
      ++st_.frames;
    } else {
      ++st_.bad;
    }
    last_seq_ = seq;
    last_arrival_us_ = now;
  }

  size_t ds5_mic_uplink::pull(uint8_t *dst, size_t bytes) {
    if (!dst || bytes == 0) return 0;
    std::lock_guard<std::mutex> lk(mtx_);
    // Stream end: the pad stopped (mic disarmed). Whatever is queued is the
    // tail of that stream -- play it out, then go quiet and re-prebuffer for
    // the next one.
    if (live_ && mic_now_us() - last_arrival_us_ > IDLE_US) {
      live_ = false;
    }
    size_t taken = 0;
    if (primed_ && count_ > 0) {
      taken = std::min(bytes, count_);
      // Whole frames only, so channels never swap across a packet boundary.
      taken -= taken % DS5_MIC_BYTES_PER_FRAME;
      const size_t first = std::min(taken, RING_CAP - head_);
      std::memcpy(dst, ring_.data() + head_, first);
      if (first < taken) std::memcpy(dst + first, ring_.data(), taken - first);
      head_ = (head_ + taken) % RING_CAP;
      count_ -= taken;
      if (count_ == 0) primed_ = false;   // underrun: refill to PREBUFFER before resuming
    }
    if (taken < bytes) {
      std::memset(dst + taken, 0, bytes - taken);
      st_.silence_bytes += bytes - taken;
    }
    st_.pulled_bytes += taken;
    return taken;
  }

  void ds5_mic_uplink::reset() {
    std::lock_guard<std::mutex> lk(mtx_);
    head_ = 0;
    count_ = 0;
    primed_ = false;
    live_ = false;
    last_seq_ = 0;
    last_arrival_us_ = 0;
    if (dec_) opus_decoder_ctl(dec_, OPUS_RESET_STATE);
  }

  ds5_mic_uplink::stats_t ds5_mic_uplink::take_stats() {
    std::lock_guard<std::mutex> lk(mtx_);
    stats_t out = st_;
    out.fill_bytes = count_;
    st_ = stats_t {};
    return out;
  }

}  // namespace platf::ds5_bridge
