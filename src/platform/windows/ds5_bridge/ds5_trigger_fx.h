/**
 * @file src/platform/windows/ds5_bridge/ds5_trigger_fx.h
 * @brief DualSense adaptive-trigger effect encoding: the 11-byte trigger FFB
 *        block that sits at offsets 10 (L2) and 21 (R2) of the DualSense output
 *        "common block".
 *
 * Concept and wire layout: artzox/DS5Dongle (MIT) `src/state_mgr.cpp`
 * — `pack_zones()`, `write_vibration()`, `write_at()`, `run_sequencer()`.
 * Re-expressed here as typed encoders; no upstream code is copied verbatim.
 *
 * ## Block layout
 * | byte  | meaning                                                          |
 * |-------|------------------------------------------------------------------|
 * | 0     | effect type id (see namespace `effect`)                          |
 * | 1-2   | 16-bit zone bitmap, LE — 10 travel zones z0..z9 (0x03FF = full)   |
 * | 3-6   | 30-bit packed per-zone force field: 10 zones x 3 bits, LE from b3 |
 * | 9     | carrier frequency (vibration only, 1..255; lower = heavier knock) |
 * | 7,8,10| not written by any known effect — left zero                      |
 *
 * The 3-bit per-zone field carries force LEVEL 1..8 as wire value 0..7, so
 * "no force" is not expressible in the field: a zone that should be free is
 * left OUT of the bitmap instead (see @ref make_resistance).
 *
 * Two byte-0 families exist for the same effects: the `0x2x` "extended" form
 * and the `0x0x` "simple" form. We always emit the `0x2x` form (what games and
 * the DualSense SDK send) but classify both on the way in.
 *
 * The zone bitmap is evaluated against the live trigger position by the
 * CONTROLLER HARDWARE, not by us and not by the pad firmware — ramps, detents
 * and walls therefore cost nothing at runtime once written.
 *
 * This header is deliberately free of platform headers so it can be unit
 * tested on any host (see tests/unit/test_ds5_trigger_fx.cpp).
 */
#pragma once

#include <array>
#include <cstdint>

namespace platf::ds5_bridge::trigger_fx {

  /// Length of one trigger force-feedback block.
  constexpr int FFB_LEN = 11;
  /// Number of discrete travel zones the hardware resolves.
  constexpr int ZONE_COUNT = 10;
  /// Largest force level on the wire (level 1..8 encoded as 0..7).
  constexpr uint8_t FORCE_WIRE_MAX = 7;
  /// Upper bound on stages in one positional sequence (matches the upstream store).
  constexpr int MAX_STAGES = 5;
  /// Every travel zone active.
  constexpr uint16_t ZONES_FULL_TRAVEL = 0x03FF;

  using ffb_t = std::array<uint8_t, FFB_LEN>;

  /// Effect type ids (byte 0).
  namespace effect {
    constexpr uint8_t none = 0x00;              ///< "no effect written" (not a real id)
    constexpr uint8_t off = 0x05;               ///< release / reset the trigger
    constexpr uint8_t resistance = 0x21;        ///< per-zone feedback force
    constexpr uint8_t resistance_simple = 0x01;
    constexpr uint8_t bow_snap = 0x22;          ///< draw + snap-back against the finger
    constexpr uint8_t bow_snap_simple = 0x02;
    constexpr uint8_t galloping = 0x23;         ///< mechanical, see make_galloping()
    constexpr uint8_t weapon_break = 0x25;      ///< rigid wall with a snap-through
    constexpr uint8_t vibration = 0x26;         ///< buzz, byte 9 = carrier
    constexpr uint8_t vibration_simple = 0x06;
    constexpr uint8_t vibration_alt = 0x27;
  }  // namespace effect

  /// Force effects: placed by trigger POSITION, durations are not part of the
  /// effect. A set containing at least one of these sequences positionally.
  constexpr bool is_mechanical(uint8_t type) {
    return type == effect::resistance || type == effect::resistance_simple ||
           type == effect::bow_snap || type == effect::bow_snap_simple ||
           type == effect::galloping || type == effect::weapon_break;
  }

  /// Buzz effects: these ignore trigger position on their own (a 0x26 with a
  /// non-zero amplitude buzzes whether or not the trigger is pulled), so their
  /// rhythm is time-based and any position gating has to come from outside.
  constexpr bool is_vibration(uint8_t type) {
    return type == effect::vibration || type == effect::vibration_simple ||
           type == effect::vibration_alt;
  }

  /// True for the two byte-0 values that mean "this trigger carries no effect".
  /// Both appear in wrapper traffic that sets the allow-bit with an empty
  /// payload, so they must never be mistaken for a game driving the trigger.
  constexpr bool is_inert(uint8_t type) {
    return type == effect::none || type == effect::off;
  }

  /// Analog trigger position (0..255) at which zone @p z begins. 10 zones over
  /// the travel: zone z starts at z * 25.5, computed without floating point.
  constexpr uint8_t zone_to_pos(uint8_t z) {
    return (uint8_t) (((uint16_t) (z > 9 ? 9 : z) * 51u) / 2u);
  }

  /// Map a 0..100 % force to the 3-bit wire value (force level 1..8 as 0..7).
  /// Rounds to nearest; 0 % maps to wire 0, which is force level 1 and NOT
  /// "no force" — callers that want a genuinely free zone must drop the zone
  /// from the bitmap.
  constexpr uint8_t force_to_wire(int strength100) {
    if (strength100 <= 0) return 0;
    const int w = (strength100 * 7 + 50) / 100;
    return (uint8_t) (w > FORCE_WIRE_MAX ? FORCE_WIRE_MAX : w);
  }

  /// Inverse of force_to_wire(), for logging and read-back.
  constexpr int wire_to_force(uint8_t wire3) {
    return ((wire3 & FORCE_WIRE_MAX) * 100 + 3) / 7;
  }

  /// Write the 16-bit zone bitmap into bytes 1-2.
  ///
  /// The upstream writers mask byte 2 with 0x03 for the full-travel effects and
  /// with 0xFF for the zone-PAIR effects. Both are the same thing here: zones
  /// only go up to z9, so the high byte never exceeds 0x03 anyway. We mask to
  /// 0x03 uniformly, which keeps bytes 1-2 a faithful 10-bit field.
  inline void set_zones(ffb_t &ffb, uint16_t zones) {
    ffb[1] = (uint8_t) (zones & 0xFF);
    ffb[2] = (uint8_t) ((zones >> 8) & 0x03);
  }

  /// Read the zone bitmap back out of bytes 1-2.
  inline uint16_t get_zones(const ffb_t &ffb) {
    return (uint16_t) (ffb[1] | ((uint16_t) (ffb[2] & 0x03) << 8));
  }

  /// Pack a per-zone force table into bytes 3-6: zone z occupies the 3 bits at
  /// offset 3*z of a little-endian 30-bit field. Zones absent from @p zones
  /// contribute nothing, so their bits stay clear.
  ///
  /// This is the one place the force field is built; every encoder below and
  /// the tests go through it.
  inline void pack_zone_forces(ffb_t &ffb, uint16_t zones, const uint8_t forces[ZONE_COUNT]) {
    uint32_t bits = 0;
    for (int z = 0; z < ZONE_COUNT; ++z) {
      if (zones & (1u << z)) {
        bits |= (uint32_t) (forces[z] & FORCE_WIRE_MAX) << (3 * z);
      }
    }
    ffb[3] = (uint8_t) (bits & 0xFF);
    ffb[4] = (uint8_t) ((bits >> 8) & 0xFF);
    ffb[5] = (uint8_t) ((bits >> 16) & 0xFF);
    ffb[6] = (uint8_t) ((bits >> 24) & 0xFF);
  }

  /// pack_zone_forces() with one force for every active zone.
  inline void pack_zones(ffb_t &ffb, uint16_t zones, uint8_t wire3) {
    uint8_t forces[ZONE_COUNT];
    for (int z = 0; z < ZONE_COUNT; ++z) forces[z] = wire3;
    pack_zone_forces(ffb, zones, forces);
  }

  /// Read zone @p z's 3-bit force back out of bytes 3-6 (0 if z is not active).
  inline uint8_t get_zone_force(const ffb_t &ffb, int z) {
    if (z < 0 || z >= ZONE_COUNT) return 0;
    if (!(get_zones(ffb) & (1u << z))) return 0;
    const uint32_t bits = (uint32_t) ffb[3] | ((uint32_t) ffb[4] << 8) |
                          ((uint32_t) ffb[5] << 16) | ((uint32_t) ffb[6] << 24);
    return (uint8_t) ((bits >> (3 * z)) & FORCE_WIRE_MAX);
  }

  /// Lowest and highest active zone. Returns false (and leaves the upstream
  /// fallback span 2..8 in place) when the bitmap is empty — an effect with no
  /// zones has no position of its own.
  inline bool zone_span(const ffb_t &ffb, uint8_t &zlo, uint8_t &zhi) {
    const uint16_t zb = get_zones(ffb);
    zlo = 2;
    zhi = 8;
    if (!zb) return false;
    zlo = 0;
    while (zlo < ZONE_COUNT && !(zb & (1u << zlo))) ++zlo;
    zhi = ZONE_COUNT - 1;
    while (zhi > 0 && !(zb & (1u << zhi))) --zhi;
    return true;
  }

  /// The zone a stage begins at — the key the positional sequencer orders by
  /// and the value the distinct-start-zone invariant is stated over.
  inline uint8_t start_zone(const ffb_t &ffb) {
    uint8_t lo = 0, hi = 0;
    zone_span(ffb, lo, hi);
    return lo;
  }

  // -- encoders -------------------------------------------------------------

  /// Shape of a per-zone resistance profile.
  enum class shape_e : uint8_t {
    constant = 0,   ///< the same force from the start zone to z9
    ramp = 1,       ///< linear force_a -> force_b across start..z9
    two_stage = 2,  ///< force_a everywhere, a wall of force_b at the detent zone
  };

  /// Release the trigger: no force, no buzz. Zero parameters — the type byte
  /// alone is the whole effect.
  inline ffb_t make_off() {
    ffb_t f {};
    f[0] = effect::off;
    return f;
  }

  /**
   * @brief 0x21 feedback: an independent force for each travel zone.
   * @param start        first zone that carries force (zones below stay free)
   * @param shape        constant / ramp / two-stage detent
   * @param strength_a   0..100 %, the base force (ramp: the force at @p start)
   * @param strength_b   0..100 %, ramp end force / detent wall force
   * @param detent_zone  zone carrying @p strength_b in shape_e::two_stage
   *
   * A zone whose computed strength is 0 % is DROPPED from the bitmap rather
   * than encoded with wire 0: the wire has no "zero force" code, so leaving the
   * zone out is the only way to make it genuinely free. That is what lets a
   * ramp start at 0 % (free at rest) or a detent be a bare bump with free
   * travel around it. Skipped for shape_e::constant, where a 0 % request means
   * the caller wants the lightest real force across the whole travel.
   */
  inline ffb_t make_resistance(uint8_t start, shape_e shape, int strength_a,
                               int strength_b = 0, uint8_t detent_zone = 0) {
    ffb_t f {};
    f[0] = effect::resistance;
    const uint8_t z0 = start > 9 ? 0 : start;
    uint16_t zones = 0;
    uint8_t forces[ZONE_COUNT] = {0};
    for (int z = z0; z < ZONE_COUNT; ++z) {
      int s100 = strength_a;
      switch (shape) {
        case shape_e::ramp: {
          const int span = 9 - z0;
          s100 = (span == 0) ? strength_b
                             : strength_a + (strength_b - strength_a) * (z - z0) / span;
          break;
        }
        case shape_e::two_stage:
          s100 = (z == detent_zone) ? strength_b : strength_a;
          break;
        case shape_e::constant:
        default:
          break;
      }
      if (shape != shape_e::constant && s100 <= 0) continue;  // free zone
      zones |= (uint16_t) (1u << z);
      forces[z] = force_to_wire(s100);
    }
    set_zones(f, zones);
    pack_zone_forces(f, zones, forces);
    return f;
  }

  /**
   * @brief 0x22 bow snap, from raw 3-bit wire values.
   *
   * Special layout — this effect does NOT use the per-zone force field:
   *  - bytes 1-2 are a zone PAIR `(1<<draw_start) | (1<<draw_end)`, with
   *    draw_end = draw_start + 4 clamped to z8 (the hardware's draw span);
   *  - byte 3 packs two 3-bit values: draw force in bits 0-2, snap force in
   *    bits 3-5. Both are force level 1..8 encoded as 0..7.
   *
   * With the trigger already held past the end zone the snap force pushes the
   * trigger BACK against the finger — a mechanical recoil, not a buzz. That is
   * the one thing resistance and weapon-break cannot do, and the reason this is
   * the interesting effect for a synthesized gun/engine kick.
   */
  inline ffb_t make_bow_snap_wire(uint8_t draw_start, uint8_t draw_wire3, uint8_t snap_wire3) {
    ffb_t f {};
    f[0] = effect::bow_snap;
    const uint8_t zs = draw_start > 7 ? 7 : draw_start;
    uint8_t ze = (uint8_t) (zs + 4);
    if (ze > 8) ze = 8;
    set_zones(f, (uint16_t) ((1u << zs) | (1u << ze)));
    f[3] = (uint8_t) ((draw_wire3 & FORCE_WIRE_MAX) |
                      ((snap_wire3 & FORCE_WIRE_MAX) << 3));
    return f;
  }

  /// make_bow_snap_wire() taking 0..100 % forces.
  inline ffb_t make_bow_snap(uint8_t draw_start, int draw100, int snap100) {
    return make_bow_snap_wire(draw_start, force_to_wire(draw100), force_to_wire(snap100));
  }

  /**
   * @brief 0x25 weapon break: a rigid wall from @p wall_start that gives way
   *        with a hardware-sharp snap at @p break_pos, after which the travel
   *        is free. Unlike a two-stage detent there is no force past the break.
   *
   * Bytes 1-2 are the zone PAIR `(1<<wall_start) | (1<<break_pos)`; byte 3 is
   * the wall force (3-bit). Hardware ranges: wall start 2..7, break point 3..8
   * and strictly above the start — values outside are clamped here so the
   * caller can see what was actually encoded.
   *
   * The break is CONSUMED when the trigger is pushed through it. Re-arming
   * needs a fresh write that the change suppression does not collapse; the
   * caller owns that (upstream emits one Off frame first).
   */
  inline ffb_t make_weapon_break(uint8_t wall_start, uint8_t break_pos, int wall_strength100) {
    ffb_t f {};
    f[0] = effect::weapon_break;
    uint8_t ws = wall_start > 9 ? 2 : wall_start;
    if (ws < 2) ws = 2;
    if (ws > 7) ws = 7;
    uint8_t we = break_pos;
    if (we <= ws) we = (uint8_t) (ws + 1);
    if (we > 8) we = 8;
    set_zones(f, (uint16_t) ((1u << ws) | (1u << we)));
    f[3] = force_to_wire(wall_strength100);
    return f;
  }

  /**
   * @brief 0x23, the third mechanical id (commonly called "galloping").
   *
   * @warning UNVERIFIED PARAMETERISATION. The reference implementation
   * classifies 0x23 as mechanical and drives it through exactly the same
   * wall/re-arm path as 0x25, but never WRITES one, so nothing outside byte 0
   * is confirmed by that source. We therefore encode it with the 0x25 zone-pair
   * layout — the only layout the reference treats it as having. If the pad
   * turns out to expect the documented gallop parameters (two foot-fall forces
   * plus a rate) in bytes 3-4, this encoder produces a single-foot gallop at
   * best. Verify on hardware before relying on it.
   */
  inline ffb_t make_galloping(uint8_t start, uint8_t end, int strength100) {
    ffb_t f = make_weapon_break(start, end, strength100);
    f[0] = effect::galloping;
    return f;
  }

  /**
   * @brief 0x26 vibration over the given zones, from a raw 3-bit amplitude.
   * @param zones  active zones; ZONES_FULL_TRAVEL for a buzz over the whole pull
   * @param freq   carrier, 1..255 — LOW frequencies read as a heavier knock
   *
   * A vibration does not sustain from a single write and it is not gated by
   * trigger position, so a caller that wants "buzz only while pulled" has to
   * gate on the live analog position itself.
   */
  inline ffb_t make_vibration_wire(uint16_t zones, uint8_t amp_wire3, uint8_t freq) {
    ffb_t f {};
    f[0] = effect::vibration;
    set_zones(f, zones);
    pack_zones(f, zones, amp_wire3);
    f[9] = freq;
    return f;
  }

  /// make_vibration_wire() taking a 0..255 amplitude (the shape the rumble and
  /// haptic envelopes arrive in). Amplitude 0 encodes as Off.
  inline ffb_t make_vibration(uint16_t zones, int amp255, uint8_t freq) {
    if (amp255 <= 0) return make_off();
    const int a = amp255 > 255 ? 255 : amp255;
    return make_vibration_wire(zones, (uint8_t) (a * 7 / 255), freq);
  }

  // -- multi-stage sets -----------------------------------------------------

  /// Outcome of stage_set::add().
  enum class add_result_e : uint8_t {
    added,                 ///< stored
    skipped_duplicate,     ///< byte-identical to a stage already present
    rejected_start_clash,  ///< different bytes but the same start zone
    full,                  ///< MAX_STAGES reached
  };

  /**
   * @brief An ordered set of trigger stages that is guaranteed sequencable.
   *
   * ## The distinct-start-zone invariant
   * Stages are played back by trigger POSITION and are therefore ordered by
   * their start zone. Two stages that start at the same zone have no defined
   * order, and the sequencer has no choice but to fall back to time-based A/B
   * cycling — which is felt as a continuous click, not as a sequence. Captured
   * timelines routinely contain the same effect twice (ready -> fired ->
   * ready), so this is the normal shape of real input, not an edge case.
   *
   * add() enforces the invariant at build time, before anything reaches the
   * pad: a byte-identical repeat is silently skipped (the common case — that
   * third "ready" stage carries no new information), and a stage that merely
   * collides on its start zone is rejected with the offending zone reported so
   * the caller can say which one clashed.
   */
  class stage_set {
  public:
    add_result_e add(const ffb_t &stage, uint8_t *clash_zone = nullptr) {
      if (count_ >= MAX_STAGES) return add_result_e::full;
      const uint8_t z = start_zone(stage);
      for (int i = 0; i < count_; ++i) {
        if (stages_[i] == stage) return add_result_e::skipped_duplicate;
        if (start_zone(stages_[i]) == z) {
          if (clash_zone) *clash_zone = z;
          return add_result_e::rejected_start_clash;
        }
      }
      stages_[count_++] = stage;
      return add_result_e::added;
    }

    void clear() { count_ = 0; }
    int count() const { return count_; }
    bool empty() const { return count_ == 0; }
    const ffb_t &operator[](int i) const { return stages_[i]; }

    /// Indices in play order (ascending start zone). Insertion order is kept
    /// as the identity of a stage so callers can report on what they added.
    void order_by_start(uint8_t out[MAX_STAGES]) const {
      for (int i = 0; i < count_; ++i) out[i] = (uint8_t) i;
      for (int a = 1; a < count_; ++a) {
        for (int b = a; b > 0 && start_zone(stages_[out[b]]) < start_zone(stages_[out[b - 1]]); --b) {
          const uint8_t t = out[b];
          out[b] = out[b - 1];
          out[b - 1] = t;
        }
      }
    }

    /**
     * @brief Can this set be played positionally?
     *
     * Requires every stage to be a known mechanical or vibration effect,
     * distinct start zones, and — for a set of two or more — at least one
     * mechanical stage, because an all-vibration set has no positions to
     * sequence over and its rhythm is time-based. A single stage is always
     * playable: there is nothing to sequence, so the position rules do not
     * apply (upstream reports "not sequencable" here and leaves the one-stage
     * case to its caller; folding it in keeps our callers to one path).
     *
     * add() already guarantees the distinct start zones; they are re-checked
     * here because a set can also be filled from captured data.
     */
    bool sequencable(uint8_t *clash_zone = nullptr) const {
      if (count_ < 1) return false;
      bool any_mech = false;
      for (int i = 0; i < count_; ++i) {
        const uint8_t t = stages_[i][0];
        if (is_mechanical(t)) any_mech = true;
        else if (!is_vibration(t)) return false;
      }
      if (!any_mech && count_ > 1) return false;
      uint8_t order[MAX_STAGES];
      order_by_start(order);
      for (int a = 1; a < count_; ++a) {
        if (start_zone(stages_[order[a]]) == start_zone(stages_[order[a - 1]])) {
          if (clash_zone) *clash_zone = start_zone(stages_[order[a]]);
          return false;
        }
      }
      return true;
    }

  private:
    ffb_t stages_[MAX_STAGES] {};
    int count_ {0};
  };

  /**
   * @brief Positional playback of a stage_set: hands over to the next stage as
   *        the trigger is pulled, resets when it returns below the whole set.
   *
   * Hand-over happens EARLY — when the pull leaves the current stage's own span
   * or @ref ARM_LEAD counts before the next stage begins, whichever comes
   * first — so the next wall is armed ahead of the finger. Arming it under the
   * finger is what produces a click instead of a wall.
   */
  class sequencer {
  public:
    /// How far ahead of the next stage's first zone to arm it, in analog counts.
    static constexpr int ARM_LEAD = 12;

    void reset() {
      stage_ = 0;
      rearm_pulse_ = false;
    }

    /// True once after any stage change; consumed by the read. Callers that
    /// re-send a consumed weapon break use it to force one Off frame first.
    bool take_rearm_pulse() {
      const bool p = rearm_pulse_;
      rearm_pulse_ = false;
      return p;
    }

    /**
     * @brief Index (in @p set's insertion order) of the stage to write now.
     * @param pos        live analog position of this trigger, 0..255
     * @param rearm_pos  position at or below which the sequence resets to its
     *                   first stage; clamped to just under the first stage so a
     *                   re-arm zone inside the sequence cannot strand it
     * @return -1 if the set cannot be played positionally.
     *
     * A single-stage set trivially returns 0; upstream returns "not
     * sequencable" there and leaves the one-stage case to its caller.
     */
    int pick(const stage_set &set, uint8_t pos, uint8_t rearm_pos) {
      const int n = set.count();
      if (n < 1 || n > MAX_STAGES || !set.sequencable()) return -1;
      uint8_t order[MAX_STAGES];
      set.order_by_start(order);
      if (stage_ >= n) stage_ = 0;
      // Reset first, and only once the pull is back below the WHOLE sequence:
      // resetting after the advance would undo a hand-over made in the same
      // call, and an un-clamped re-arm zone sits inside the sequence for any
      // first stage above z0, which strands every later stage.
      {
        int eff = rearm_pos;
        const int below = (int) zone_to_pos(start_zone(set[order[0]])) - 1;
        if (below < eff) eff = below;
        if (eff < 0) eff = 0;
        if (stage_ != 0 && (int) pos <= eff) {
          stage_ = 0;
          rearm_pulse_ = true;
        }
      }
      while (stage_ + 1 < n) {
        uint8_t lo = 0, hi = 0;
        zone_span(set[order[stage_]], lo, hi);
        int sw = (int) zone_to_pos(start_zone(set[order[stage_ + 1]])) - ARM_LEAD;
        const int end_cur = (int) zone_to_pos(hi);
        if (end_cur < sw) sw = end_cur;
        if (sw < 1) sw = 1;
        if ((int) pos >= sw) {
          ++stage_;
          rearm_pulse_ = true;
        } else {
          break;
        }
      }
      return order[stage_];
    }

  private:
    int stage_ {0};
    bool rearm_pulse_ {false};
  };

}  // namespace platf::ds5_bridge::trigger_fx
