/**
 * @file src/platform/windows/ds5_bridge/ds5_mic.h
 * @brief DS5 microphone uplink: Opus frames from the TV -> PCM for the virtual
 *        pad's iso-IN capture endpoint (EP 0x82).
 *
 * Port plan W3-02. Design reference: rhoquinn8217/CTM-USBIP branch
 * ds5-bt-mic-capture (GPL-3; mic_ring.inl / mic_decode.inl) — the inbound
 * mirror of the speaker jitter design: a shallow per-session ring, never wait,
 * drop-oldest on overflow, silence on underflow, one decoder per session kept
 * alive (Opus frames depend on their predecessors). Written from scratch.
 *
 * Source format (measured on the pad, see memory ds5_bt_protocol_reference):
 * with the microphone armed the DualSense sends BT input reports 0x31 whose
 * flag byte has bit 1 set, carrying one Opus packet of 71 bytes from report
 * byte 3: TOC 0xd4 = CELT-only, SWB, STEREO, 10 ms -> 480 samples @ 48 kHz,
 * ~100 packets/s. Bit 0 (pad state) and bit 1 (audio) are mutually exclusive
 * and the report's 4-bit sequence nibble is shared with the pad-state reports,
 * so the TV cannot derive a per-frame mic sequence from it: the TV app numbers
 * the frames itself (ctmb_ds5_mic_t.seq) and this side conceals gaps by seq
 * AND by arrival time.
 *
 * Sink format: the virtual DualSense's config descriptor (usbip_ds5_device.cpp,
 * CONFIG_DESC) declares for the capture AudioStreaming interface a Type I
 * format descriptor `0B 24 02 01 02 02 10 01 80 BB 00` = 2 channels, 2 bytes
 * per subframe, 16 bits, one sample rate 0x00BB80 = 48000 Hz, and endpoint
 * `05 82 05 C4 00 04` = EP 0x82, isochronous ASYNC, wMaxPacketSize 196,
 * bInterval 4 (1 ms at high speed). 48 frames x 4 bytes = 192 bytes per 1 ms
 * packet; the extra 4 bytes are the async endpoint's rate-adjust headroom. So
 * the Opus output (48 kHz stereo s16) is exactly the endpoint format: no
 * resampling, no channel mapping.
 *
 * Threading: feed()/reset() run on the bridge session thread, pull() on the
 * usbip iso pacer thread. One mutex, no blocking calls under it except the
 * Opus decode (~40 us per frame), which the pacer thread never performs.
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <vector>

// Global forward declaration, same reason as OpusEncoder in ds5_haptics.h:
// opus.h typedefs `struct OpusDecoder` at global scope, and a namespaced
// forward declaration would ODR-clash with it in the TU that includes opus.h.
struct OpusDecoder;

namespace platf::ds5_bridge {

  // Capture endpoint geometry (see the header comment for the descriptor).
  constexpr int DS5_MIC_RATE = 48000;
  constexpr int DS5_MIC_CHANNELS = 2;
  constexpr int DS5_MIC_BYTES_PER_FRAME = DS5_MIC_CHANNELS * 2;      // s16
  constexpr int DS5_MIC_BYTES_PER_MS = DS5_MIC_RATE / 1000 * DS5_MIC_BYTES_PER_FRAME;  // 192
  // One Opus packet from the pad = 10 ms = 480 frames.
  constexpr int DS5_MIC_FRAME_SAMPLES = DS5_MIC_RATE / 100;
  constexpr int DS5_MIC_FRAME_BYTES = DS5_MIC_FRAME_SAMPLES * DS5_MIC_BYTES_PER_FRAME;  // 1920
  // The pad's packet is 71 bytes; accept a little slack for a future firmware,
  // reject anything that cannot be one 10 ms packet.
  constexpr int DS5_MIC_MAX_OPUS = 200;

  class ds5_mic_uplink {
  public:
    ds5_mic_uplink();
    ~ds5_mic_uplink();

    ds5_mic_uplink(const ds5_mic_uplink &) = delete;
    ds5_mic_uplink &operator=(const ds5_mic_uplink &) = delete;

    /// False when libopus refused to create the decoder; feed() then drops.
    bool ok() const { return dec_ != nullptr; }

    /// Session thread: one Opus packet from the TV, with the TV's per-session
    /// 16-bit sequence number (starts at 0, wraps). Decodes, conceals any gap
    /// since the previous packet, appends to the ring.
    void feed(uint16_t seq, const uint8_t *opus, size_t len);

    /// Pacer thread: fill @p dst with @p bytes of PCM (a whole number of
    /// frames). Never waits. Silence while the ring is prebuffering, idle or
    /// empty. Returns the number of bytes that came from the ring (telemetry).
    size_t pull(uint8_t *dst, size_t bytes);

    /// Session thread: link dropped / session torn down. Clears the ring and
    /// forgets the stream so the next session cannot open on this one's tail.
    void reset();

    struct stats_t {
      uint64_t frames;        // Opus packets decoded
      uint64_t plc;           // concealment frames synthesized
      uint64_t bad;           // packets rejected (not one 10 ms 48 kHz packet / decode error)
      uint64_t drop_bytes;    // PCM discarded on ring overflow (oldest first)
      uint64_t pulled_bytes;  // PCM handed to the endpoint from the ring
      uint64_t silence_bytes; // endpoint bytes filled with silence instead
      uint64_t starts;        // stream (re)starts (idle -> live)
      size_t fill_bytes;      // ring occupancy now
    };
    /// Snapshot the window counters and clear them (fill_bytes is a level).
    stats_t take_stats();

  private:
    // Ring capacity: ~200 ms. Deep enough to ride out a WiFi hiccup, shallow
    // enough that a full ring costs no more than that in mouth-to-ear delay.
    static constexpr size_t RING_CAP = (size_t) DS5_MIC_BYTES_PER_MS * 200;
    // Prebuffer: the endpoint starts draining once this much is queued, so
    // the normal arrival jitter (10 ms frames over WiFi) never runs it dry.
    static constexpr size_t PREBUFFER = (size_t) DS5_MIC_BYTES_PER_MS * 40;
    // Arrival gap that counts as loss rather than jitter (a frame is 10 ms).
    static constexpr uint64_t PLC_GAP_US = 25000;
    // Longest run of concealment per gap; beyond it the gap is silence.
    static constexpr int PLC_MAX_FRAMES = 5;
    // No packet for this long = the pad stopped streaming (mic disarmed, PTT
    // released): drop back to silence and start the next stream fresh.
    static constexpr uint64_t IDLE_US = 500000;

    void push_locked(const int16_t *pcm, int frames);   // mtx_ held
    void plc_locked(int frames);                        // mtx_ held

    OpusDecoder *dec_ {nullptr};
    std::mutex mtx_;
    std::vector<uint8_t> ring_;   // RING_CAP bytes, circular
    size_t head_ {0};             // oldest byte
    size_t count_ {0};            // bytes queued
    bool primed_ {false};         // ring reached PREBUFFER since the last underrun
    bool live_ {false};           // a stream is in progress (packets arriving)
    uint16_t last_seq_ {0};
    uint64_t last_arrival_us_ {0};
    stats_t st_ {};
    std::vector<int16_t> pcm_;    // decode scratch (DS5_MIC_FRAME_SAMPLES * channels)
  };

}  // namespace platf::ds5_bridge
