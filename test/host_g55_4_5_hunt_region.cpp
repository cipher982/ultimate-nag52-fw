// 4<->5 policy: hysteresis measurement, invariant checks, and the effect of the
// upshift torque assumption on both.
//
// The question: the driver reports constant 4<->5 hunting at 35-50 mph. Is that
// a fixed property of the shipped Standard-profile policy, and does judging the
// 4->5 gate at the torque the truck will carry in 5th -- rather than at the
// momentary torque of the lift that triggered the upshift -- remove it?
//
// Three measurements, all from the policy alone, no vehicle:
//
//  1. REPLICA FIDELITY. A parameterised copy of the policy is checked cell for
//     cell against the real inline functions before any swept number is used.
//  2. INVARIANT. No single operating point admits 5th and then revokes it.
//  3. DEAD BAND. Holding road speed still, how much pedal movement separates
//     "4th may still upshift" from "5th must downshift"? That width is what the
//     driver's foot has to cross, and it is the defect surface.
//
// Build: scripts/test_g55_4_5_hunt_region.sh
#include "g55_road_response_policy.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <initializer_list>
#include <string>

namespace {

// --- vehicle constants (installed configuration) -----------------------------
// diff 4.11, transfer-case high 0.87, fitted tyre ~2567 mm loaded rolling
// circumference: output_rpm = mph * 60 * 1609.344 / (2.567 * 4.11 * 0.87) / 1000
constexpr double kOutputRpmPerMph = 37.36;

// --- shipped shift-table rows (src/maps.cpp, S_PETROL_*) ---------------------
const int kUpshift45[11] = {1500, 1550, 1600, 1700, 1900, 2000, 2500, 3000, 3500, 4500, 6000};
const int kDownshift54[11] = {1000, 1050, 1100, 1200, 1300, 1400, 1500, 1600, 2000, 2400, 2800};

constexpr int kRatio4x1000 = 1000;
constexpr int kRatio5x1000 = 830;

// Assumption values used by the sweep. kNoAssumption reproduces the pre-change
// behaviour: judge the gate at the momentary torque.
constexpr int16_t kNoAssumption = -1;

int row_lookup(const int* row, int pedal_raw) {
    const double x = (double)pedal_raw / 2.5;
    if (x <= 0.0) { return row[0]; }
    if (x >= 100.0) { return row[10]; }
    const int col = (int)(x / 10.0);
    const double frac = (x - col * 10.0) / 10.0;
    return (int)std::lround(row[col] + frac * (row[col + 1] - row[col]));
}

struct Policy {
    int16_t upshift_torque_assumption_nm = kNoAssumption;
    int upshift45_lift_rpm = 0;
    uint16_t floor_min_rpm = G55RoadResponsePolicy::kD5DownshiftFloorMinimumRpm;
    uint16_t floor_max_rpm = G55RoadResponsePolicy::kD5DownshiftFloorMaximumRpm;

    uint16_t threshold(uint16_t stored, int pedal_raw, int torque_nm) const {
        const int32_t pedal_blend = G55RoadResponsePolicy::blend_factor_per_mille(
            pedal_raw,
            G55RoadResponsePolicy::kBlendPedalStartRaw,
            G55RoadResponsePolicy::kBlendPedalFullRaw);
        const int32_t torque_blend = G55RoadResponsePolicy::blend_factor_per_mille(
            torque_nm,
            G55RoadResponsePolicy::kBlendInputTorqueStartNm,
            G55RoadResponsePolicy::kBlendInputTorqueFullNm);
        const int32_t blend = pedal_blend < torque_blend ? pedal_blend : torque_blend;
        if (blend == 0) { return stored; }
        int32_t t = torque_nm;
        if (t < G55RoadResponsePolicy::kFloorInputTorqueMinimumNm) {
            t = G55RoadResponsePolicy::kFloorInputTorqueMinimumNm;
        }
        if (t > G55RoadResponsePolicy::kLoadedInputTorqueMaximumNm) {
            t = G55RoadResponsePolicy::kLoadedInputTorqueMaximumNm;
        }
        const int32_t torque_range = G55RoadResponsePolicy::kLoadedInputTorqueMaximumNm -
            G55RoadResponsePolicy::kFloorInputTorqueMinimumNm;
        const int32_t rpm_range = (int32_t)floor_max_rpm - (int32_t)floor_min_rpm;
        const uint16_t floor_rpm = floor_min_rpm +
            ((t - G55RoadResponsePolicy::kFloorInputTorqueMinimumNm) * rpm_range) / torque_range;
        if (stored >= floor_rpm) { return stored; }
        return stored + ((floor_rpm - stored) * blend) / 1000;
    }

    int16_t assessed_torque(int torque_nm) const {
        if (upshift_torque_assumption_nm == kNoAssumption) { return (int16_t)torque_nm; }
        return torque_nm > upshift_torque_assumption_nm
            ? (int16_t)torque_nm : upshift_torque_assumption_nm;
    }

    int map45(int pedal_raw) const {
        return row_lookup(kUpshift45, pedal_raw) + upshift45_lift_rpm;
    }

    // Replica of G55RoadResponsePolicy::four_to_five_is_allowed_by_d5_policy.
    bool gate_allows(int output_rpm, int pedal_raw, int torque_nm) const {
        const int stored = row_lookup(kDownshift54, pedal_raw);
        const uint16_t predicted5 = (uint16_t)(((uint32_t)output_rpm * kRatio5x1000) / 1000);
        return (uint32_t)predicted5 >=
            (uint32_t)threshold(stored, pedal_raw, assessed_torque(torque_nm)) +
                G55RoadResponsePolicy::kD5UpshiftMarginRpm;
    }

    // Which of the two constraints holds the upshift back at this point?
    const char* binding(int output_rpm, int pedal_raw, int torque_nm) const {
        const bool map_ok = output_rpm * kRatio4x1000 / 1000 > map45(pedal_raw);
        const bool gate_ok = gate_allows(output_rpm, pedal_raw, torque_nm);
        if (map_ok && gate_ok) { return "open"; }
        if (!map_ok && !gate_ok) { return "both"; }
        return map_ok ? "gate" : "map";
    }

    // StandardProfile::should_upshift for 4th. The temperature, time-since-shift
    // and engine-load adders only ever raise the threshold, so this is the most
    // permissive reading of the shipped policy.
    bool admits_upshift(int output_rpm, int pedal_raw, int torque_nm) const {
        if (pedal_raw == 0) { return false; }
        if (output_rpm * kRatio4x1000 / 1000 <= map45(pedal_raw)) { return false; }
        return gate_allows(output_rpm, pedal_raw, torque_nm);
    }

    // StandardProfile::should_downshift for 5th, minus the 200 ms qualifier.
    bool revokes_fifth(int output_rpm, int pedal_raw, int torque_nm) const {
        const int stored = row_lookup(kDownshift54, pedal_raw);
        const uint16_t input5 = (uint16_t)(((uint32_t)output_rpm * kRatio5x1000) / 1000);
        if (input5 < (uint16_t)stored) { return true; }
        return input5 < threshold(stored, pedal_raw, torque_nm);
    }
};

// The policy as shipped: no torque assumption, shipped map row, shipped floor.
Policy shipped() { return Policy{}; }

void verify_replica_matches_shipped() {
    const Policy p = shipped();
    for (int output_rpm = 400; output_rpm <= 4000; output_rpm += 5) {
        for (int pedal = 0; pedal <= 255; pedal += 1) {
            for (int torque = -200; torque <= 600; torque += 25) {
                const uint16_t stored = (uint16_t)row_lookup(kDownshift54, pedal);
                assert(G55RoadResponsePolicy::four_to_five_is_allowed_by_d5_policy(
                           (uint16_t)output_rpm, kRatio5x1000, stored,
                           (uint8_t)pedal, (int16_t)torque) ==
                       p.gate_allows(output_rpm, pedal, torque));
                assert(G55RoadResponsePolicy::should_downshift_from_d5(
                           (uint16_t)(((uint32_t)output_rpm * kRatio5x1000) / 1000),
                           stored, (uint8_t)pedal, (int16_t)torque) ==
                       p.revokes_fifth(output_rpm, pedal, torque));
            }
        }
    }
    std::printf("replica matches the shipped policy over the full grid\n");
}

bool static_contradiction_exists(const Policy& p, long* cells) {
    *cells = 0;
    for (int output_rpm = 400; output_rpm <= 4000; output_rpm += 2) {
        for (int pedal = 1; pedal <= 255; pedal += 1) {
            for (int torque = 0; torque <= 600; torque += 25) {
                if (p.admits_upshift(output_rpm, pedal, torque) &&
                    p.revokes_fifth(output_rpm, pedal, torque)) {
                    (*cells)++;
                }
            }
        }
    }
    return *cells > 0;
}

struct Band {
    int lift_max = -1;
    int press_min = -1;
    bool defined() const { return lift_max >= 0 && press_min >= 0; }
    int width() const { return press_min - lift_max; }
};

// Pedal movement at a fixed road speed and torque that separates "4th may still
// upshift" from "5th must downshift".
Band band_at(const Policy& p, int output_rpm, int torque_nm) {
    Band b;
    for (int pedal = 1; pedal <= 255; pedal++) {
        if (p.admits_upshift(output_rpm, pedal, torque_nm)) { b.lift_max = pedal; }
        if (b.press_min < 0 && p.revokes_fifth(output_rpm, pedal, torque_nm)) { b.press_min = pedal; }
    }
    return b;
}

std::string fmt(const Band& b) {
    if (!b.defined()) { return " n/a "; }
    return std::to_string(b.width());
}

}  // namespace

int main() {
    verify_replica_matches_shipped();

    long cells = 0;
    if (static_contradiction_exists(shipped(), &cells)) {
        std::printf("INVARIANT VIOLATED: %ld operating points admit 5th and revoke it\n", cells);
        return 1;
    }
    std::printf("invariant holds: no operating point admits 5th and revokes it\n");

    std::printf("\nwhat limits the 4->5 upshift (pedal range 1..255, 300 Nm)\n");
    std::printf("  mph |  binding constraint over the admitted pedal range\n");
    for (int mph = 40; mph <= 56; mph += 2) {
        const int out = (int)std::lround(mph * kOutputRpmPerMph);
        std::string seen;
        int lift = -1;
        for (int pedal = 1; pedal <= 255; pedal++) {
            if (shipped().admits_upshift(out, pedal, 300)) { lift = pedal; }
        }
        if (lift < 0) { std::printf("  %3d | 4->5 not permitted at any pedal\n", mph); continue; }
        const char* why = shipped().binding(out, lift, 300);
        std::printf("  %3d | lift<=%3d raw, last admitted pedal limited by: %s\n", mph, lift, why);
        (void)seen;
    }

    std::printf("\npedal dead band vs 4->5 map-row lift (300 Nm)\n");
    std::printf("  mph |");
    for (int lift : {0, 100, 200, 300, 400, 500}) { std::printf("  lift%-4d", lift); }
    std::printf("\n");
    for (int mph = 42; mph <= 54; mph += 2) {
        std::printf("  %3d |", mph);
        for (int lift : {0, 100, 200, 300, 400, 500}) {
            Policy p;
            p.upshift45_lift_rpm = lift;
            std::printf("  %-8s", fmt(band_at(p, (int)std::lround(mph * kOutputRpmPerMph), 300)).c_str());
        }
        std::printf("\n");
    }

    // A foot cannot hold a band this narrow; road load, grade and the
    // supercharger clutch all move the pedal further than this.
    constexpr int kMinimumUsableBandRaw = 25;
    std::printf("\nusability check (band must be >= %d raw counts):\n", kMinimumUsableBandRaw);
    for (int torque : {300, 450}) {
        for (int mph = 42; mph <= 54; mph += 2) {
            const Band b = band_at(shipped(), (int)std::lround(mph * kOutputRpmPerMph), torque);
            if (b.defined() && b.width() < kMinimumUsableBandRaw) {
                std::printf("  WARNING: %d mph / %d Nm: dead band is only %d raw (%.1f%% pedal)\n",
                            mph, torque, b.width(), 100.0 * b.width() / 250.0);
            }
        }
    }

    std::printf("\npedal dead band vs D5 load floor, map lift 0 (300 Nm)\n");
    std::printf("  mph |");
    for (uint16_t lo : {(uint16_t)1500, (uint16_t)1400, (uint16_t)1300, (uint16_t)1200, (uint16_t)1100}) {
        std::printf("  fl%-6u", lo);
    }
    std::printf("\n");
    for (int mph = 42; mph <= 54; mph += 2) {
        std::printf("  %3d |", mph);
        for (uint16_t lo : {(uint16_t)1500, (uint16_t)1400, (uint16_t)1300, (uint16_t)1200, (uint16_t)1100}) {
            Policy p;
            p.floor_min_rpm = lo;
            p.floor_max_rpm = (uint16_t)(lo + 300);
            std::printf("  %-8s", fmt(band_at(p, (int)std::lround(mph * kOutputRpmPerMph), 300)).c_str());
        }
        std::printf("\n");
    }

    std::printf("\npedal dead band vs D5 load floor at 450 Nm, map lift 0\n");
    std::printf("  mph |");
    for (uint16_t lo : {(uint16_t)1500, (uint16_t)1400, (uint16_t)1300, (uint16_t)1200, (uint16_t)1100}) {
        std::printf("  fl%-6u", lo);
    }
    std::printf("\n");
    for (int mph = 42; mph <= 54; mph += 2) {
        std::printf("  %3d |", mph);
        for (uint16_t lo : {(uint16_t)1500, (uint16_t)1400, (uint16_t)1300, (uint16_t)1200, (uint16_t)1100}) {
            Policy p;
            p.floor_min_rpm = lo;
            p.floor_max_rpm = (uint16_t)(lo + 300);
            std::printf("  %-8s", fmt(band_at(p, (int)std::lround(mph * kOutputRpmPerMph), 450)).c_str());
        }
        std::printf("\n");
    }
    return 0;
}
