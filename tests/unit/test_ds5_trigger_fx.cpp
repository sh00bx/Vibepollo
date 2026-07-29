/**
 * @file tests/unit/test_ds5_trigger_fx.cpp
 * @brief Wire-level tests for the DualSense adaptive-trigger effect encoders.
 *
 * The header under test is platform-independent on purpose (it is pure byte
 * packing), so these run on every host even though the bridge that uses them
 * is Windows-only. Expected byte patterns are written out literally rather
 * than recomputed with the same arithmetic as the encoder — a test that
 * repeats the implementation proves nothing.
 */
#include "src/platform/windows/ds5_bridge/ds5_trigger_fx.h"

#include <gtest/gtest.h>

namespace fx = platf::ds5_bridge::trigger_fx;

namespace {
  /// Bytes 3-6 as the 30-bit force field they encode.
  uint32_t force_field(const fx::ffb_t &f) {
    return (uint32_t) f[3] | ((uint32_t) f[4] << 8) | ((uint32_t) f[5] << 16) |
           ((uint32_t) f[6] << 24);
  }
}  // namespace

// -- force scaling ----------------------------------------------------------

TEST(Ds5TriggerFx, ForceToWireRoundsToNearestAndClamps) {
  EXPECT_EQ(fx::force_to_wire(0), 0);
  EXPECT_EQ(fx::force_to_wire(7), 0);    // 0.49 -> 0
  EXPECT_EQ(fx::force_to_wire(14), 1);   // 0.98 -> 1
  EXPECT_EQ(fx::force_to_wire(50), 4);   // 3.5 -> 4
  EXPECT_EQ(fx::force_to_wire(100), 7);
  EXPECT_EQ(fx::force_to_wire(400), 7);  // clamped, not wrapped
  EXPECT_EQ(fx::force_to_wire(-20), 0);
}

TEST(Ds5TriggerFx, ZoneToPositionSpansTheTravel) {
  EXPECT_EQ(fx::zone_to_pos(0), 0);
  EXPECT_EQ(fx::zone_to_pos(1), 25);
  EXPECT_EQ(fx::zone_to_pos(2), 51);
  EXPECT_EQ(fx::zone_to_pos(4), 102);
  EXPECT_EQ(fx::zone_to_pos(9), 229);
  EXPECT_EQ(fx::zone_to_pos(200), 229);  // clamped to the last zone
}

// -- zone / force packing ---------------------------------------------------

TEST(Ds5TriggerFx, PackZonesFillsThirtyBitsAtFullTravel) {
  fx::ffb_t f {};
  fx::pack_zones(f, fx::ZONES_FULL_TRAVEL, 7);
  // 10 zones x 3 bits, all set: bits 0..29.
  EXPECT_EQ(f[3], 0xFF);
  EXPECT_EQ(f[4], 0xFF);
  EXPECT_EQ(f[5], 0xFF);
  EXPECT_EQ(f[6], 0x3F);  // bits 30-31 stay clear: the field is 30 bits, not 32
  EXPECT_EQ(force_field(f), 0x3FFFFFFFu);
}

TEST(Ds5TriggerFx, ZoneFieldSitsAtThreeTimesTheZoneIndex) {
  for (int z = 0; z < fx::ZONE_COUNT; ++z) {
    fx::ffb_t f {};
    const uint16_t zones = (uint16_t) (1u << z);
    fx::set_zones(f, zones);
    fx::pack_zones(f, zones, 5);
    EXPECT_EQ(force_field(f), 5u << (3 * z)) << "zone " << z;
    EXPECT_EQ(fx::get_zone_force(f, z), 5) << "zone " << z;
  }
}

TEST(Ds5TriggerFx, InactiveZonesContributeNoForceBits) {
  fx::ffb_t f {};
  uint8_t forces[fx::ZONE_COUNT];
  for (auto &v : forces) v = 7;  // every zone asks for full force ...
  const uint16_t zones = (1u << 2) | (1u << 5);  // ... but only two are active
  fx::set_zones(f, zones);
  fx::pack_zone_forces(f, zones, forces);
  EXPECT_EQ(force_field(f), (7u << 6) | (7u << 15));
  EXPECT_EQ(fx::get_zone_force(f, 2), 7);
  EXPECT_EQ(fx::get_zone_force(f, 5), 7);
  EXPECT_EQ(fx::get_zone_force(f, 3), 0);
}

TEST(Ds5TriggerFx, ZoneBitmapUsesTenBitsLittleEndian) {
  fx::ffb_t f {};
  fx::set_zones(f, fx::ZONES_FULL_TRAVEL);
  EXPECT_EQ(f[1], 0xFF);
  EXPECT_EQ(f[2], 0x03);
  EXPECT_EQ(fx::get_zones(f), fx::ZONES_FULL_TRAVEL);
  fx::set_zones(f, (uint16_t) (1u << 9));
  EXPECT_EQ(f[1], 0x00);
  EXPECT_EQ(f[2], 0x02);
}

TEST(Ds5TriggerFx, ZoneSpanReportsLowestAndHighest) {
  fx::ffb_t f {};
  uint8_t lo = 0, hi = 0;
  fx::set_zones(f, (uint16_t) ((1u << 3) | (1u << 7)));
  EXPECT_TRUE(fx::zone_span(f, lo, hi));
  EXPECT_EQ(lo, 3);
  EXPECT_EQ(hi, 7);
  EXPECT_EQ(fx::start_zone(f), 3);

  fx::ffb_t empty {};
  EXPECT_FALSE(fx::zone_span(empty, lo, hi));
}

// -- effect encoders --------------------------------------------------------

TEST(Ds5TriggerFx, OffCarriesTypeOnly) {
  const auto f = fx::make_off();
  EXPECT_EQ(f[0], 0x05);
  for (int i = 1; i < fx::FFB_LEN; ++i) EXPECT_EQ(f[i], 0) << "byte " << i;
}

TEST(Ds5TriggerFx, ConstantResistanceCoversStartToEnd) {
  const auto f = fx::make_resistance(4, fx::shape_e::constant, 100);
  EXPECT_EQ(f[0], 0x21);
  EXPECT_EQ(fx::get_zones(f), 0x03F0u);  // zones 4..9, nothing below
  for (int z = 4; z < 10; ++z) EXPECT_EQ(fx::get_zone_force(f, z), 7) << "zone " << z;
  EXPECT_EQ(f[9], 0);  // no carrier on a force effect
}

TEST(Ds5TriggerFx, ConstantResistanceKeepsAZeroPercentZone) {
  // Constant is the one shape where 0 % means "lightest real force over the
  // whole travel" rather than "free" — a constant profile with no zones at all
  // would be an Off with a 0x21 label.
  const auto f = fx::make_resistance(0, fx::shape_e::constant, 0);
  EXPECT_EQ(fx::get_zones(f), fx::ZONES_FULL_TRAVEL);
  EXPECT_EQ(force_field(f), 0u);
}

TEST(Ds5TriggerFx, RampDropsTheZeroStrengthZoneFromTheBitmap) {
  // The wire has no "zero force" code — force level 1..8 is sent as 0..7 — so
  // a genuinely free zone can only be expressed by leaving it out.
  const auto f = fx::make_resistance(0, fx::shape_e::ramp, 0, 100);
  EXPECT_EQ(f[0], 0x21);
  EXPECT_FALSE(fx::get_zones(f) & (1u << 0)) << "zone 0 is 0 % and must be absent";
  EXPECT_EQ(fx::get_zones(f), 0x03FEu);
  EXPECT_EQ(fx::get_zone_force(f, 0), 0);
  EXPECT_EQ(fx::get_zone_force(f, 9), 7);
  // Monotonically rising through the draw.
  for (int z = 2; z < 10; ++z) {
    EXPECT_GE(fx::get_zone_force(f, z), fx::get_zone_force(f, z - 1)) << "zone " << z;
  }
}

TEST(Ds5TriggerFx, TwoStageDetentWithZeroBaseIsABareBump) {
  const auto f = fx::make_resistance(0, fx::shape_e::two_stage, 0, 80, 5);
  EXPECT_EQ(fx::get_zones(f), (uint16_t) (1u << 5)) << "only the detent zone carries force";
  EXPECT_EQ(fx::get_zone_force(f, 5), 6);
  EXPECT_EQ(force_field(f), 6u << 15);
}

TEST(Ds5TriggerFx, BowSnapPacksDrawAndSnapIntoByteThree) {
  // Special layout: bytes 1-2 are a zone PAIR, byte 3 holds two 3-bit forces.
  const auto f = fx::make_bow_snap_wire(3, 4, 7);
  EXPECT_EQ(f[0], 0x22);
  EXPECT_EQ(f[1], 0x88);  // (1<<3) | (1<<7): draw start 3, end start+4
  EXPECT_EQ(f[2], 0x00);
  EXPECT_EQ(f[3], 0x3C);  // draw 4 in bits 0-2, snap 7 in bits 3-5
  EXPECT_EQ(f[4], 0);
  EXPECT_EQ(f[5], 0);
  EXPECT_EQ(f[6], 0);
}

TEST(Ds5TriggerFx, BowSnapClampsTheDrawSpanToZoneEight) {
  const auto f = fx::make_bow_snap_wire(9, 7, 7);
  EXPECT_EQ(fx::get_zones(f), (uint16_t) ((1u << 7) | (1u << 8)));
  EXPECT_EQ(f[3], 0x3F);
}

TEST(Ds5TriggerFx, BowSnapPercentWrapperMatchesTheWireForm) {
  EXPECT_EQ(fx::make_bow_snap(2, 100, 50), fx::make_bow_snap_wire(2, 7, 4));
}

TEST(Ds5TriggerFx, WeaponBreakEncodesTheWallPairAndForce) {
  const auto f = fx::make_weapon_break(4, 6, 100);
  EXPECT_EQ(f[0], 0x25);
  EXPECT_EQ(fx::get_zones(f), (uint16_t) ((1u << 4) | (1u << 6)));
  EXPECT_EQ(f[3], 7);
}

TEST(Ds5TriggerFx, WeaponBreakClampsToTheHardwareRanges) {
  // Wall start 2..7, break point above the start and at most 8.
  EXPECT_EQ(fx::get_zones(fx::make_weapon_break(0, 1, 50)),
            (uint16_t) ((1u << 2) | (1u << 3)));
  EXPECT_EQ(fx::get_zones(fx::make_weapon_break(9, 9, 50)),
            (uint16_t) ((1u << 7) | (1u << 8)));
  EXPECT_EQ(fx::get_zones(fx::make_weapon_break(5, 3, 50)),
            (uint16_t) ((1u << 5) | (1u << 6))) << "break must sit above the wall";
}

TEST(Ds5TriggerFx, GallopingReusesTheWallLayoutUnderItsOwnTypeId) {
  const auto f = fx::make_galloping(3, 6, 100);
  EXPECT_EQ(f[0], 0x23);
  EXPECT_EQ(fx::get_zones(f), (uint16_t) ((1u << 3) | (1u << 6)));
  EXPECT_TRUE(fx::is_mechanical(f[0]));
}

TEST(Ds5TriggerFx, VibrationCarriesAmplitudeAndCarrier) {
  const auto f = fx::make_vibration_wire(fx::ZONES_FULL_TRAVEL, 7, 35);
  EXPECT_EQ(f[0], 0x26);
  EXPECT_EQ(f[1], 0xFF);
  EXPECT_EQ(f[2], 0x03);
  EXPECT_EQ(force_field(f), 0x3FFFFFFFu);
  EXPECT_EQ(f[9], 35);
  EXPECT_EQ(f[7], 0);
  EXPECT_EQ(f[8], 0);
  EXPECT_EQ(f[10], 0);
}

TEST(Ds5TriggerFx, VibrationFromEnvelopeScalesAndFallsBackToOff) {
  EXPECT_EQ(fx::make_vibration(fx::ZONES_FULL_TRAVEL, 255, 35)[3], 0xFF);
  EXPECT_EQ(fx::make_vibration(fx::ZONES_FULL_TRAVEL, 0, 35), fx::make_off());
  EXPECT_EQ(fx::make_vibration(fx::ZONES_FULL_TRAVEL, 4000, 35),
            fx::make_vibration_wire(fx::ZONES_FULL_TRAVEL, 7, 35));
}

TEST(Ds5TriggerFx, EffectClassification) {
  EXPECT_TRUE(fx::is_mechanical(0x21));
  EXPECT_TRUE(fx::is_mechanical(0x01));  // simple form of the same effect
  EXPECT_TRUE(fx::is_mechanical(0x22));
  EXPECT_TRUE(fx::is_mechanical(0x25));
  EXPECT_TRUE(fx::is_vibration(0x26));
  EXPECT_TRUE(fx::is_vibration(0x06));
  EXPECT_TRUE(fx::is_vibration(0x27));
  EXPECT_FALSE(fx::is_mechanical(0x26));
  EXPECT_FALSE(fx::is_vibration(0x21));
  EXPECT_TRUE(fx::is_inert(0x00));
  EXPECT_TRUE(fx::is_inert(0x05));
  EXPECT_FALSE(fx::is_inert(0x21));
}

// -- the distinct-start-zone invariant --------------------------------------

TEST(Ds5TriggerFxStages, ExactRepeatIsSkipped) {
  // The shape captured timelines actually have: ready -> fired -> ready.
  fx::stage_set set;
  const auto ready = fx::make_resistance(0, fx::shape_e::constant, 40);
  const auto fired = fx::make_weapon_break(4, 6, 90);
  EXPECT_EQ(set.add(ready), fx::add_result_e::added);
  EXPECT_EQ(set.add(fired), fx::add_result_e::added);
  EXPECT_EQ(set.add(ready), fx::add_result_e::skipped_duplicate);
  EXPECT_EQ(set.count(), 2);
  EXPECT_TRUE(set.sequencable());
}

TEST(Ds5TriggerFxStages, DifferentEffectOnTheSameStartZoneIsRejected) {
  fx::stage_set set;
  EXPECT_EQ(set.add(fx::make_weapon_break(4, 6, 90)), fx::add_result_e::added);
  uint8_t clash = 0xFF;
  // Same start zone (4), different bytes — no defined play order, so it would
  // silently degrade into time-based A/B cycling (felt as a constant click).
  EXPECT_EQ(set.add(fx::make_weapon_break(4, 8, 40), &clash),
            fx::add_result_e::rejected_start_clash);
  EXPECT_EQ(clash, 4) << "the caller has to be able to name the clashing zone";
  EXPECT_EQ(set.count(), 1);
}

TEST(Ds5TriggerFxStages, SetIsCappedAtFiveStages) {
  fx::stage_set set;
  for (int z = 0; z < 5; ++z) {
    EXPECT_EQ(set.add(fx::make_resistance((uint8_t) z, fx::shape_e::ramp, 0, 100)),
              fx::add_result_e::added)
      << "stage " << z;
  }
  EXPECT_EQ(set.count(), 5);
  EXPECT_EQ(set.add(fx::make_weapon_break(6, 8, 50)), fx::add_result_e::full);
}

TEST(Ds5TriggerFxStages, OrderIsByStartZoneNotInsertion) {
  fx::stage_set set;
  set.add(fx::make_weapon_break(6, 8, 90));                     // start 6
  set.add(fx::make_resistance(0, fx::shape_e::constant, 30));   // start 0
  set.add(fx::make_weapon_break(3, 5, 60));                     // start 3
  uint8_t order[fx::MAX_STAGES];
  set.order_by_start(order);
  EXPECT_EQ(order[0], 1);
  EXPECT_EQ(order[1], 2);
  EXPECT_EQ(order[2], 0);
}

TEST(Ds5TriggerFxStages, AllVibrationSetsAreNotPositional) {
  fx::stage_set set;
  set.add(fx::make_vibration_wire((uint16_t) (1u << 1), 4, 30));
  set.add(fx::make_vibration_wire((uint16_t) (1u << 5), 7, 60));
  EXPECT_FALSE(set.sequencable()) << "a vibration ignores position; its rhythm is time-based";

  // A single vibration is fine — there is nothing to sequence.
  fx::stage_set one;
  one.add(fx::make_vibration_wire(fx::ZONES_FULL_TRAVEL, 7, 35));
  EXPECT_TRUE(one.sequencable());
}

TEST(Ds5TriggerFxStages, UnknownEffectIdIsNotPositional) {
  fx::stage_set set;
  fx::ffb_t junk {};
  junk[0] = 0x77;
  set.add(junk);
  EXPECT_FALSE(set.sequencable());
}

// -- positional playback ----------------------------------------------------

TEST(Ds5TriggerFxSequencer, SingleStageAlwaysPlays) {
  fx::stage_set set;
  set.add(fx::make_vibration_wire(fx::ZONES_FULL_TRAVEL, 7, 35));
  fx::sequencer seq;
  EXPECT_EQ(seq.pick(set, 0, 0), 0);
  EXPECT_EQ(seq.pick(set, 255, 0), 0);
}

TEST(Ds5TriggerFxSequencer, HandsOverAheadOfTheFinger) {
  fx::stage_set set;
  set.add(fx::make_resistance(0, fx::shape_e::constant, 30));  // idx 0, start z0
  set.add(fx::make_weapon_break(4, 6, 90));                    // idx 1, start z4 = pos 102
  fx::sequencer seq;

  EXPECT_EQ(seq.pick(set, 0, 0), 0);
  EXPECT_EQ(seq.pick(set, 89, 0), 0) << "one count short of the lead-in";
  // Armed ARM_LEAD counts early so the wall is in place before the finger
  // reaches it; arming it under the finger is felt as a click, not a wall.
  EXPECT_EQ(seq.pick(set, 102 - fx::sequencer::ARM_LEAD, 0), 1);
  EXPECT_EQ(seq.pick(set, 200, 0), 1);
}

TEST(Ds5TriggerFxSequencer, ResetsOnlyBelowTheWholeSequence) {
  fx::stage_set set;
  set.add(fx::make_weapon_break(2, 3, 80));  // idx 0, start z2 = pos 51
  set.add(fx::make_weapon_break(6, 8, 90));  // idx 1, start z6 = pos 153
  fx::sequencer seq;

  EXPECT_EQ(seq.pick(set, 141, 255), 1) << "153 - 12 arms the upper wall";
  // A re-arm position INSIDE the sequence must not strand the later stages:
  // it is clamped to just below the first stage.
  EXPECT_EQ(seq.pick(set, 100, 255), 1);
  EXPECT_EQ(seq.pick(set, 50, 255), 0) << "back below zone 2 -> re-armed";
}

TEST(Ds5TriggerFxSequencer, StageChangeRaisesTheRearmPulseOnce) {
  fx::stage_set set;
  set.add(fx::make_resistance(0, fx::shape_e::constant, 30));
  set.add(fx::make_weapon_break(4, 6, 90));
  fx::sequencer seq;

  seq.pick(set, 0, 0);
  EXPECT_FALSE(seq.take_rearm_pulse());
  seq.pick(set, 120, 0);
  EXPECT_TRUE(seq.take_rearm_pulse()) << "a consumed break needs a fresh send to re-arm";
  EXPECT_FALSE(seq.take_rearm_pulse()) << "and the pulse is consumed by the read";
}

TEST(Ds5TriggerFxSequencer, RefusesAnUnsequencableSet) {
  fx::stage_set set;
  set.add(fx::make_vibration_wire((uint16_t) (1u << 1), 4, 30));
  set.add(fx::make_vibration_wire((uint16_t) (1u << 5), 7, 60));
  fx::sequencer seq;
  EXPECT_EQ(seq.pick(set, 128, 0), -1);
}
