#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

#include "tcc_direct_slip_controller.h"
#include "tcc_pressure_command.h"
#include "tcc_measured_plant.h"
#include "tcc_measured_replay.h"

namespace {
void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

int clamp_int(int value, int low, int high) {
    return std::max(low, std::min(high, value));
}

int move_toward(int value, int target, int rise_limit, int fall_limit) {
    if (value < target) {
        return value + std::min(target - value, rise_limit);
    }
    if (value > target) {
        return value - std::min(value - target, fall_limit);
    }
    return value;
}

TccDirectSlipInput enabled_input(
    uint32_t now_ms,
    int signed_slip_rpm,
    int torque_direction = 1,
    int target_rpm = 80,
    int feedforward_mbar = 2000,
    bool shift_active = false
) {
    return {
        true,
        true,
        true,
        shift_active,
        true,
        TccDirectSlipReason::None,
        signed_slip_rpm,
        torque_direction,
        target_rpm,
        feedforward_mbar,
        now_ms,
    };
}

// Exact normal-branch structure used by V8 after fill/settle. This is retained
// only to prove that the calibrated hybrid plant can reproduce the rejected
// controller's measured limit cycle.
class LegacyV8StablePolicy {
public:
    explicit LegacyV8StablePolicy(int initial_pressure_mbar)
        : pressure_mbar_(initial_pressure_mbar) {}

    int step(int slip_rpm) {
        constexpr int target_rpm = 80;
        constexpr int feedforward_mbar = 2000;
        const int error_rpm = slip_rpm - target_rpm;
        const int proportional_mbar = clamp_int(2 * error_rpm, -1000, 1500);
        const int proposed_integral_scaled = clamp_int(
            integral_scaled_ + error_rpm,
            -1000 * 50,
            1000 * 50
        );
        const int proposed_mbar = feedforward_mbar + proportional_mbar +
            proposed_integral_scaled / 50;
        if (!((proposed_mbar > 3000 && error_rpm > 0) ||
              (proposed_mbar < 1000 && error_rpm < 0))) {
            integral_scaled_ = proposed_integral_scaled;
        }
        const int requested_mbar = clamp_int(
            feedforward_mbar + proportional_mbar + integral_scaled_ / 50,
            1000,
            3000
        );
        pressure_mbar_ = move_toward(pressure_mbar_, requested_mbar, 25, 50);
        return pressure_mbar_;
    }

private:
    int pressure_mbar_ = 0;
    int integral_scaled_ = 0;
};

void test_vehicle_learned_maps_are_preserved() {
    using tcc_test::MeasuredTccMaps;
    require(std::lround(MeasuredTccMaps::slip_mbar(4, 20)) == 1162,
        "D4 20% learned slip cell must match the vehicle readback");
    require(std::lround(MeasuredTccMaps::slip_mbar(4, 50)) == 1192,
        "D4 50% learned slip cell must match the vehicle readback");
    require(std::lround(MeasuredTccMaps::lock_mbar(4, 20)) == 2986,
        "D4 20% learned lock cell must match the vehicle readback");
    require(std::lround(MeasuredTccMaps::lock_mbar(4, 50)) == 3600,
        "D4 50% learned lock cell must match the vehicle readback");
    require(MeasuredTccMaps::slip_mbar(4, 35) <
            MeasuredTccMaps::lock_mbar(4, 35),
        "controlled-slip and full-lock feed-forward must remain distinct");
}

void test_measured_plant_matches_breakaway_and_regrab_envelope() {
    tcc_test::MeasuredWetTccPlant plant;
    plant.prime_coupled(1900);

    tcc_test::MeasuredPlantOutput sample = {};
    bool released = false;
    uint32_t release_ms = 0;
    double release_effective_mbar = 0;
    for (uint32_t now = 0; now <= 12000; now += 20) {
        const double controller_mbar = 1900.0 - 800.0 * now / 12000.0;
        sample = plant.step(controller_mbar, 150.0);
        if (!sample.coupled) {
            released = true;
            release_ms = now;
            release_effective_mbar = sample.effective_pressure_mbar;
            break;
        }
    }
    require(released, "the measured pressure wind-down must cross breakaway");
    require(release_effective_mbar >= 1000 && release_effective_mbar <= 1220,
        "simulated D4 breakaway must stay inside the measured pressure envelope");

    double peak_slip_rpm = sample.slip_rpm;
    bool regrabbed = false;
    uint32_t regrab_ms = 0;
    for (uint32_t now = release_ms + 20; now <= release_ms + 700; now += 20) {
        sample = plant.step(1500, 150.0);
        peak_slip_rpm = std::max(peak_slip_rpm, sample.slip_rpm);
        if (sample.coupled) {
            regrabbed = true;
            regrab_ms = now - release_ms;
            break;
        }
    }
    require(regrabbed, "the measured re-grab command must recouple the clutch");
    require(regrab_ms >= 150 && regrab_ms <= 500,
        "simulated re-grab timing must cover the measured 246-304 ms response");
    require(peak_slip_rpm >= 200 && peak_slip_rpm <= 360,
        "simulated breakaway peak must remain comparable to the 269-296 RPM trace");
}

void test_measured_plant_reproduces_v8_d4_limit_cycle() {
    tcc_test::MeasuredWetTccPlant plant;
    plant.prime_coupled(1900);
    LegacyV8StablePolicy legacy(1900);

    bool over_threshold = false;
    int excursions = 0;
    double maximum_slip_rpm = 0;
    for (uint32_t now = 0; now <= 25000; now += 20) {
        const int command_mbar = legacy.step(
            static_cast<int>(std::lround(plant.slip_rpm()))
        );
        const auto sample = plant.step(command_mbar, 150.0);
        maximum_slip_rpm = std::max(maximum_slip_rpm, sample.slip_rpm);
        const bool current_over_threshold = sample.slip_rpm > 150;
        if (current_over_threshold && !over_threshold) {
            excursions += 1;
        }
        over_threshold = current_over_threshold;
    }
    require(excursions >= 3,
        "the hybrid plant must reproduce repeated V8 D4 breakaway cycles");
    require(maximum_slip_rpm >= 180 && maximum_slip_rpm <= 420,
        "the legacy-policy failure must remain in the measured order of magnitude");
}

struct ReplayResult {
    double peak_slip_rpm;
    bool released;
    bool regrabbed;
};

struct ClosedLoopReplayResult {
    ReplayResult plant;
    int entry_controller_mbar;
    int minimum_controller_mbar;
    int maximum_controller_mbar;
};

template <std::size_t N, typename CommandSelector>
ReplayResult replay_measured_commands(
    const std::array<tcc_test::ReplayPoint, N>& points,
    CommandSelector command_selector
) {
    tcc_test::MeasuredPlantCalibration calibration;
    calibration.temperature_multiplier = 1.0;
    tcc_test::MeasuredWetTccPlant plant(calibration);
    plant.prime_coupled(command_selector(points.front()));

    ReplayResult result = {0.0, false, false};
    for (const auto& point : points) {
        const auto sample = plant.step(
            command_selector(point),
            point.torque_nm,
            point.dt_ms
        );
        result.peak_slip_rpm = std::max(
            result.peak_slip_rpm,
            sample.slip_rpm
        );
        if (!sample.coupled) {
            result.released = true;
        } else if (result.released) {
            result.regrabbed = true;
        }
    }
    return result;
}

template <std::size_t N>
ClosedLoopReplayResult replay_v9_closed_loop(
    const std::array<tcc_test::ReplayPoint, N>& points,
    bool shift_active
) {
    tcc_test::MeasuredPlantCalibration calibration;
    calibration.temperature_multiplier = 1.0;
    tcc_test::MeasuredWetTccPlant plant(calibration);
    TccDirectSlipController controller;
    TccDirectSlipOutput out = {};
    const float initial_multiplier = static_cast<float>(
        static_cast<double>(points.front().commanded_mbar) /
        points.front().controller_mbar
    );
    plant.prime_coupled(tcc_final_pressure_command_mbar(2000, initial_multiplier));

    uint32_t now_ms = 0;
    for (; now_ms <= 3000; now_ms += 20) {
        out = controller.step(enabled_input(
            now_ms,
            static_cast<int>(std::lround(plant.slip_rpm())),
            1,
            80,
            2000,
            false
        ));
        plant.step(
            tcc_final_pressure_command_mbar(out.pressure_mbar, initial_multiplier),
            points.front().torque_nm,
            20
        );
    }
    require(plant.coupled() && out.pressure_mbar == 2000,
        "closed-loop replay must begin from a coupled measured-map command");

    ClosedLoopReplayResult result = {
        {0.0, false, false},
        out.pressure_mbar,
        out.pressure_mbar,
        out.pressure_mbar,
    };
    for (const auto& point : points) {
        now_ms += point.dt_ms;
        out = controller.step(enabled_input(
            now_ms,
            static_cast<int>(std::lround(plant.slip_rpm())),
            1,
            80,
            2000,
            shift_active
        ));
        const float multiplier = static_cast<float>(
            static_cast<double>(point.commanded_mbar) / point.controller_mbar
        );
        const auto sample = plant.step(
            tcc_final_pressure_command_mbar(out.pressure_mbar, multiplier),
            point.torque_nm,
            point.dt_ms
        );
        result.plant.peak_slip_rpm = std::max(
            result.plant.peak_slip_rpm,
            sample.slip_rpm
        );
        if (!sample.coupled) {
            result.plant.released = true;
        } else if (result.plant.released) {
            result.plant.regrabbed = true;
        }
        result.minimum_controller_mbar = std::min(
            result.minimum_controller_mbar,
            out.pressure_mbar
        );
        result.maximum_controller_mbar = std::max(
            result.maximum_controller_mbar,
            out.pressure_mbar
        );
    }
    return result;
}

void test_actual_d4_command_replay_discriminates_v8_from_v9_hold() {
    int observed_peak_rpm = 0;
    for (const auto& point : tcc_test::kD4BreakawayReplay) {
        observed_peak_rpm = std::max(
            observed_peak_rpm,
            std::abs(point.observed_slip_rpm)
        );
    }
    require(observed_peak_rpm == 270,
        "the frozen D4 replay must preserve the measured V8 peak");

    const auto v8 = replay_measured_commands(
        tcc_test::kD4BreakawayReplay,
        [](const tcc_test::ReplayPoint& point) {
            return static_cast<double>(point.commanded_mbar);
        }
    );
    require(v8.released && v8.regrabbed,
        "the actual D4 V8 command series must release and re-grab the plant");
    require(v8.peak_slip_rpm >= 180.0 && v8.peak_slip_rpm <= 380.0,
        "the D4 replay peak must stay in the measured order of magnitude");

    const auto v9 = replay_v9_closed_loop(
        tcc_test::kD4BreakawayReplay,
        false
    );
    require(!v9.plant.released && v9.plant.peak_slip_rpm == 0.0,
        "the production V9 loop must remove the D4 pressure-dump cycle");
    require(v9.minimum_controller_mbar == v9.entry_controller_mbar,
        "the production V9 loop must not wind down its coupled D4 command");
}

void test_actual_two_to_three_replay_discriminates_v8_from_v9_hold() {
    int observed_peak_rpm = 0;
    for (const auto& point : tcc_test::kTwoToThreeReplay) {
        observed_peak_rpm = std::max(
            observed_peak_rpm,
            std::abs(point.observed_slip_rpm)
        );
    }
    require(observed_peak_rpm == 207,
        "the frozen 2->3 replay must preserve the measured V8 peak");

    const auto v8 = replay_measured_commands(
        tcc_test::kTwoToThreeReplay,
        [](const tcc_test::ReplayPoint& point) {
            return static_cast<double>(point.commanded_mbar);
        }
    );
    require(v8.released,
        "the actual 2->3 V8 command collapse must release the plant");
    require(v8.peak_slip_rpm >= 100.0,
        "the 2->3 replay must cross a material breakaway threshold");

    const auto v9 = replay_v9_closed_loop(
        tcc_test::kTwoToThreeReplay,
        true
    );
    require(!v9.plant.released && v9.plant.peak_slip_rpm == 0.0,
        "the production V9 loop must prevent the 2->3 command-collapse release");
    require(v9.minimum_controller_mbar == v9.entry_controller_mbar &&
            v9.maximum_controller_mbar == v9.entry_controller_mbar,
        "the production V9 loop must freeze, not tighten or unload, during 2->3");
}

void test_v9_stable_d4_does_not_manufacture_slip() {
    TccDirectSlipController controller;
    tcc_test::MeasuredWetTccPlant plant;
    TccDirectSlipOutput out = {};
    bool over_threshold = false;
    int excursions = 0;
    int minimum_coupled_pressure_mbar = 3000;
    bool ever_tracked = false;

    for (uint32_t now = 0; now <= 25000; now += 20) {
        out = controller.step(enabled_input(
            now,
            static_cast<int>(std::lround(plant.slip_rpm())),
            1,
            80,
            2000
        ));
        const auto sample = plant.step(out.pressure_mbar, 150.0);
        if (now >= 5000) {
            minimum_coupled_pressure_mbar = std::min(
                minimum_coupled_pressure_mbar,
                out.pressure_mbar
            );
            const bool current_over_threshold = sample.slip_rpm > 150;
            if (current_over_threshold && !over_threshold) {
                excursions += 1;
            }
            over_threshold = current_over_threshold;
        }
        ever_tracked = ever_tracked || out.tracking_achieved;
    }
    require(ever_tracked, "V9 must recognize a coupled clutch as tracked");
    require(excursions == 0,
        "V9 must not reproduce the stable-D4 breakaway cycle");
    require(minimum_coupled_pressure_mbar == 2000,
        "V9 must retain the proven D4 holding pressure instead of winding down");
    require(out.integral_pressure_mbar >= 0,
        "V9 must never carry the negative integral measured on V8");
}

void test_v9_stable_across_measured_plant_uncertainty() {
    constexpr std::array<double, 3> kTemperatureMultipliers = {0.86, 0.90, 0.93};
    constexpr std::array<double, 3> kInputTorquesNm = {100.0, 150.0, 224.0};
    constexpr std::array<double, 3> kStaticCapacityGains = {0.33, 0.375, 0.42};
    constexpr std::array<std::size_t, 3> kDelayCycles = {2, 3, 5};
    constexpr std::array<double, 3> kRiseTimesMs = {80.0, 110.0, 160.0};

    int cases = 0;
    for (const double temperature_multiplier : kTemperatureMultipliers) {
        for (const double input_torque_nm : kInputTorquesNm) {
            for (const double static_gain : kStaticCapacityGains) {
                for (const std::size_t delay_cycles : kDelayCycles) {
                    for (const double rise_tau_ms : kRiseTimesMs) {
                        tcc_test::MeasuredPlantCalibration calibration;
                        calibration.temperature_multiplier = temperature_multiplier;
                        calibration.static_capacity_nm_per_mbar = static_gain;
                        calibration.command_delay_cycles = delay_cycles;
                        calibration.pressure_rise_tau_ms = rise_tau_ms;
                        tcc_test::MeasuredWetTccPlant plant(calibration);
                        TccDirectSlipController controller;
                        TccDirectSlipOutput out = {};
                        double maximum_settled_slip_rpm = 0;
                        bool tracked = false;
                        for (uint32_t now = 0; now <= 20000; now += 20) {
                            out = controller.step(enabled_input(
                                now,
                                static_cast<int>(std::lround(plant.slip_rpm())),
                                1,
                                80,
                                2000
                            ));
                            const auto sample = plant.step(out.pressure_mbar, input_torque_nm);
                            if (now >= 6000) {
                                maximum_settled_slip_rpm = std::max(
                                    maximum_settled_slip_rpm,
                                    sample.slip_rpm
                                );
                            }
                            tracked = tracked || out.tracking_achieved;
                        }
                        require(tracked,
                            "every measured uncertainty case must acquire coupling");
                        require(maximum_settled_slip_rpm < 150,
                            "no measured uncertainty case may develop a settled limit cycle");
                        cases += 1;
                    }
                }
            }
        }
    }
    require(cases == 243, "the complete measured uncertainty grid must execute");
}

void test_learned_slip_map_is_not_silently_used_as_lock_pressure() {
    using tcc_test::MeasuredTccMaps;
    tcc_test::MeasuredPlantCalibration calibration;
    const double slip_feedforward_final_mbar =
        MeasuredTccMaps::slip_mbar(4, 20) * calibration.temperature_multiplier;
    const double lock_feedforward_final_mbar =
        std::min(2000.0, MeasuredTccMaps::lock_mbar(4, 20)) *
        calibration.temperature_multiplier;
    const double slip_static_capacity_nm = std::max(
        0.0,
        slip_feedforward_final_mbar - calibration.kiss_pressure_mbar
    ) * calibration.static_capacity_nm_per_mbar;
    const double lock_static_capacity_nm = std::max(
        0.0,
        lock_feedforward_final_mbar - calibration.kiss_pressure_mbar
    ) * calibration.static_capacity_nm_per_mbar;
    require(slip_static_capacity_nm < 150.0,
        "the learned D4 slip cell must remain a controlled-slip, near-breakaway input");
    require(lock_static_capacity_nm > 224.0,
        "the bounded learned lock cell must retain the measured loaded-hold margin");
}

void test_v9_coupled_pressure_is_monotonic_under_low_slip() {
    TccDirectSlipController controller;
    TccDirectSlipOutput out = {};
    for (uint32_t now = 0; now <= 1200; now += 20) {
        out = controller.step(enabled_input(now, 300, 1, 80, 1900));
    }
    const int pressure_before_coupling_mbar = out.pressure_mbar;
    for (uint32_t now = 1220; now <= 12000; now += 20) {
        const int previous_pressure_mbar = out.pressure_mbar;
        out = controller.step(enabled_input(now, 0, 1, 80, 1900));
        require(out.pressure_mbar >= previous_pressure_mbar,
            "low slip must never make V9 reduce a proven pressure");
        require(out.proportional_pressure_mbar == 0,
            "slip below the upper bound must not request negative pressure");
        require(out.integral_pressure_mbar >= 0,
            "slip below the upper bound must not wind a negative integral");
    }
    require(out.pressure_mbar >= pressure_before_coupling_mbar,
        "extended coupled operation must preserve its entry pressure");
}

struct LoadedShiftFixture {
    int entry_controller_mbar;
    double temperature_multiplier;
    int input_torque_nm;
    int target_rpm;
    uint32_t pre_ratio_ms;
};

constexpr std::array<LoadedShiftFixture, 4> kMeasuredTwoToThree = {{
    {1903, 1644.0 / 1903.0, 172, 20, 327},
    {1960, 1736.0 / 1960.0, 210, 28, 216},
    {1931, 1737.0 / 1931.0, 183, 20, 448},
    {1893, 1730.0 / 1893.0, 224, 44, 662},
}};

void test_v9_preserves_each_measured_two_to_three_entry_command() {
    for (const auto& fixture : kMeasuredTwoToThree) {
        TccDirectSlipController controller;
        tcc_test::MeasuredPlantCalibration calibration;
        calibration.temperature_multiplier = fixture.temperature_multiplier;
        tcc_test::MeasuredWetTccPlant plant(calibration);
        TccDirectSlipOutput out = {};

        uint32_t now = 0;
        for (; now <= 5000; now += 20) {
            out = controller.step(enabled_input(
                now,
                static_cast<int>(std::lround(plant.slip_rpm())),
                1,
                fixture.target_rpm,
                fixture.entry_controller_mbar
            ));
            plant.step(out.pressure_mbar, fixture.input_torque_nm);
        }
        require(plant.coupled(),
            "precondition: each measured loaded shift must begin coupled");
        const int entry_pressure_mbar = out.pressure_mbar;
        const int entry_final_command_mbar = tcc_final_pressure_command_mbar(
            entry_pressure_mbar,
            static_cast<float>(fixture.temperature_multiplier)
        );
        for (const uint32_t end = now + fixture.pre_ratio_ms; now <= end; now += 20) {
            out = controller.step(enabled_input(
                now,
                static_cast<int>(std::lround(plant.slip_rpm())),
                1,
                fixture.target_rpm,
                fixture.entry_controller_mbar,
                true
            ));
            plant.step(out.pressure_mbar, fixture.input_torque_nm);
            require(out.pressure_mbar >= entry_pressure_mbar,
                "V9 must not relax any measured 2->3 holding pressure");
            require(tcc_final_pressure_command_mbar(
                    out.pressure_mbar,
                    static_cast<float>(fixture.temperature_multiplier)
                ) >= entry_final_command_mbar,
                "the production temperature correction must preserve the final shift-entry command");
        }
    }
}

void test_super_capacity_shift_defers_correction_then_recovers() {
    TccDirectSlipController controller;
    tcc_test::MeasuredWetTccPlant plant;
    TccDirectSlipOutput out = {};
    uint32_t now = 0;
    for (; now <= 5000; now += 20) {
        out = controller.step(enabled_input(
            now,
            static_cast<int>(std::lround(plant.slip_rpm())),
            1,
            80,
            1900
        ));
        plant.step(out.pressure_mbar, 200.0);
    }
    require(plant.coupled(), "precondition: forced breakaway must begin coupled");
    int previous_pressure_mbar = out.pressure_mbar;
    const int entry_pressure_mbar = out.pressure_mbar;
    double maximum_slip_rpm = 0;
    bool released = false;
    for (const uint32_t end = now + 120; now <= end; now += 20) {
        out = controller.step(enabled_input(
            now,
            static_cast<int>(std::lround(plant.slip_rpm())),
            1,
            80,
            1900,
            true
        ));
        const auto sample = plant.step(out.pressure_mbar, 420.0);
        maximum_slip_rpm = std::max(maximum_slip_rpm, sample.slip_rpm);
        released = released || !sample.coupled;
        require(out.pressure_mbar >= previous_pressure_mbar,
            "super-capacity shift must never reduce coupling pressure");
        require(out.pressure_mbar == entry_pressure_mbar,
            "ratio-change slip must not tighten TCC pressure during the shift");
        previous_pressure_mbar = out.pressure_mbar;
    }
    for (const uint32_t end = now + 2000; now <= end; now += 20) {
        out = controller.step(enabled_input(
            now,
            static_cast<int>(std::lround(plant.slip_rpm())),
            1,
            80,
            1900
        ));
        const auto sample = plant.step(out.pressure_mbar, 200.0);
        maximum_slip_rpm = std::max(maximum_slip_rpm, sample.slip_rpm);
        require(out.pressure_mbar >= previous_pressure_mbar,
            "post-disturbance recovery must preserve acquired pressure");
        previous_pressure_mbar = out.pressure_mbar;
    }
    require(maximum_slip_rpm < 1200,
        "an uncalibrated severe disturbance must still remain numerically bounded");
    require(released,
        "the super-capacity disturbance must actually break static coupling");
    require(out.pressure_mbar > entry_pressure_mbar,
        "post-shift feedback must acquire additional holding pressure after breakaway");
    require(plant.coupled(),
        "V9 must re-couple after a short disturbance beyond static capacity");
}

void test_controller_selected_mid_shift_defers_until_shift_finishes() {
    TccDirectSlipController controller;
    auto out = controller.step(enabled_input(0, 1000, 1, 40, 2000, true));
    require(out.state == TccDirectSlipState::Open && out.pressure_mbar == 0 &&
            out.reason == TccDirectSlipReason::ShiftActive,
        "a newly selected controller must remain open through an in-progress shift");
    for (uint32_t now = 20; now <= 800; now += 20) {
        out = controller.step(enabled_input(now, 1000, 1, 40, 2000, true));
        require(out.pressure_mbar == 0,
            "mid-shift deferral must not traverse the V8 breakaway-pressure band");
    }
    out = controller.step(enabled_input(820, 1000, 1, 40, 2000, false));
    require(out.pressure_mbar == TccDirectSlipCalibration::kApplyRisePerCycleMbar,
        "normal bounded fill must begin immediately after the shift clears");
}

void test_partial_fill_is_abandoned_if_shift_starts_next_cycle() {
    TccDirectSlipController controller;
    auto out = controller.step(enabled_input(0, 300, 1, 40, 2000, false));
    require(out.pressure_mbar == TccDirectSlipCalibration::kApplyRisePerCycleMbar,
        "precondition: normal acquisition must begin with a partial fill");
    out = controller.step(enabled_input(20, 300, 1, 40, 2000, true));
    require(out.state == TccDirectSlipState::Open && out.pressure_mbar == 0 &&
            out.reason == TccDirectSlipReason::ShiftActive,
        "a shift beginning during acquisition must abandon the unproven partial fill");
}

void test_high_shift_slip_freezes_proven_pressure() {
    TccDirectSlipController controller;
    TccDirectSlipOutput out = {};
    for (uint32_t now = 0; now <= 1000; now += 20) {
        out = controller.step(enabled_input(now, 0, 1, 40, 1900, false));
    }
    const int entry_pressure_mbar = out.pressure_mbar;
    for (uint32_t now = 1020; now <= 2500; now += 20) {
        out = controller.step(enabled_input(now, 500, 1, 40, 1900, true));
        require(out.pressure_mbar == entry_pressure_mbar &&
                out.pressure_delta_mbar == 0,
            "shift-time slip must never ratchet or unload the proven TCC command");
    }
}

void test_drive_and_coast_remain_symmetric() {
    TccDirectSlipController drive;
    TccDirectSlipController coast;
    for (uint32_t now = 0; now <= 1800; now += 20) {
        const int slip = now < 900 ? 250 : 0;
        const auto drive_out = drive.step(enabled_input(now, slip, 1));
        const auto coast_out = coast.step(enabled_input(now, -slip, -1));
        require(drive_out.oriented_slip_rpm == coast_out.oriented_slip_rpm,
            "equivalent drive/coast slip must remain oriented alike");
        require(drive_out.pressure_mbar == coast_out.pressure_mbar,
            "equivalent drive/coast cases must retain equal pressure");
    }
}

void test_torque_reversal_freezes_feedback() {
    TccDirectSlipController controller;
    TccDirectSlipOutput out = {};
    for (uint32_t now = 0; now <= 900; now += 20) {
        out = controller.step(enabled_input(now, 120, 1));
    }
    const int pressure_before_reversal = out.pressure_mbar;
    for (uint32_t now = 920; now < 1120; now += 20) {
        out = controller.step(enabled_input(now, -120, -1));
        require(out.pressure_mbar == pressure_before_reversal,
            "torque reversal must hold pressure during direction settling");
    }
}

void test_shift_during_torque_reversal_holds_entry_pressure() {
    TccDirectSlipController controller;
    TccDirectSlipOutput out = {};
    for (uint32_t now = 0; now <= 900; now += 20) {
        out = controller.step(enabled_input(now, 120, 1));
    }
    const int pressure_before_reversal = out.pressure_mbar;
    for (uint32_t now = 920; now < 1120; now += 20) {
        out = controller.step(enabled_input(now, 500, -1, 80, 2000, true));
        require(out.pressure_mbar == pressure_before_reversal,
            "a shift during direction settling must hold, not tighten on stale orientation");
    }
}

void test_unresponsive_plant_faults_once_without_retry() {
    TccDirectSlipController controller;
    TccDirectSlipOutput out = {};
    uint32_t fault_time = 0;
    for (uint32_t now = 0; now < 10000; now += 20) {
        out = controller.step(enabled_input(now, 1000));
        require(out.pressure_mbar <= TccDirectSlipCalibration::kMaxCommandPressureMbar,
            "controller pressure must remain bounded");
        if (out.fault_latched) {
            fault_time = now;
            break;
        }
    }
    require(fault_time > 0, "an unresponsive warm plant must eventually fault");
    require(out.state == TccDirectSlipState::FaultOpen && out.pressure_mbar == 0,
        "qualified nontracking must latch open");
    out = controller.step(enabled_input(fault_time + 5000, 1000));
    require(out.state == TccDirectSlipState::FaultOpen && out.pressure_mbar == 0,
        "V9 must never retry a thermal fault automatically");
    controller.clear_fault();
    out = controller.step(enabled_input(fault_time + 5020, 300));
    require(!out.fault_latched && out.pressure_mbar == 100,
        "an explicit Park/key-cycle clear must permit a new fill");
}

void test_cold_pressure_derate_cannot_trigger_full_pressure_fault() {
    TccDirectSlipController controller;
    TccDirectSlipOutput out = {};
    for (uint32_t now = 0; now < 10000; now += 20) {
        auto input = enabled_input(now, 1000);
        input.allow_fault_monitor = false;
        out = controller.step(input);
    }
    require(!out.fault_latched,
        "a cold derated command must not be diagnosed as full-pressure failure");
    require(out.pressure_mbar == TccDirectSlipCalibration::kMaxCommandPressureMbar,
        "cold fault suppression must not change the controller command bound");
}

void test_invalid_speed_opens_without_fault() {
    TccDirectSlipController controller;
    auto input = enabled_input(0, 100);
    input.speed_valid = false;
    const auto out = controller.step(input);
    require(out.state == TccDirectSlipState::Open && out.pressure_mbar == 0,
        "invalid speed must fail open");
    require(out.reason == TccDirectSlipReason::InvalidSpeed && !out.fault_latched,
        "invalid speed must remain distinct from a thermal fault");
}

void test_feedforward_and_target_are_clamped() {
    TccDirectSlipController low;
    TccDirectSlipController high;
    TccDirectSlipOutput low_out = {};
    TccDirectSlipOutput high_out = {};
    for (uint32_t now = 0; now <= 500; now += 20) {
        low_out = low.step(enabled_input(now, 20, 1, 0, 200));
        high_out = high.step(enabled_input(now, 80, 1, 500, 6000));
    }
    require(low_out.feedforward_pressure_mbar == 1500,
        "feed-forward must not fall below the measured same-vehicle bound");
    require(high_out.feedforward_pressure_mbar == 2000,
        "feed-forward must preserve feedback headroom");
    require(low_out.target_slip_rpm >= 20 && high_out.target_slip_rpm <= 80,
        "normal upper slip bound must remain inside the V9 envelope");
}

void test_feedforward_drop_during_fill_cannot_stall_feedback() {
    TccDirectSlipController controller;
    TccDirectSlipOutput out = {};
    for (uint32_t now = 0; now <= 320; now += 20) {
        out = controller.step(enabled_input(now, 0, 1, 80, 2000));
    }
    require(out.pressure_mbar > 1500 && out.pressure_mbar < 2000,
        "precondition: pressure must be between the two feed-forward requests");
    for (uint32_t now = 340; now <= 800; now += 20) {
        out = controller.step(enabled_input(now, 0, 1, 80, 1500));
    }
    require(out.tracking_achieved,
        "a lower load request during fill must still enter upper-bound feedback");
    require(out.pressure_mbar >= 1500,
        "a lower feed-forward request must not erase acquired pressure");
}
}  // namespace

int main() {
    test_vehicle_learned_maps_are_preserved();
    test_measured_plant_matches_breakaway_and_regrab_envelope();
    test_measured_plant_reproduces_v8_d4_limit_cycle();
    test_actual_d4_command_replay_discriminates_v8_from_v9_hold();
    test_actual_two_to_three_replay_discriminates_v8_from_v9_hold();
    test_v9_stable_d4_does_not_manufacture_slip();
    test_v9_stable_across_measured_plant_uncertainty();
    test_learned_slip_map_is_not_silently_used_as_lock_pressure();
    test_v9_coupled_pressure_is_monotonic_under_low_slip();
    test_v9_preserves_each_measured_two_to_three_entry_command();
    test_super_capacity_shift_defers_correction_then_recovers();
    test_controller_selected_mid_shift_defers_until_shift_finishes();
    test_partial_fill_is_abandoned_if_shift_starts_next_cycle();
    test_high_shift_slip_freezes_proven_pressure();
    test_drive_and_coast_remain_symmetric();
    test_torque_reversal_freezes_feedback();
    test_shift_during_torque_reversal_holds_entry_pressure();
    test_unresponsive_plant_faults_once_without_retry();
    test_cold_pressure_derate_cannot_trigger_full_pressure_fault();
    test_invalid_speed_opens_without_fault();
    test_feedforward_and_target_are_clamped();
    test_feedforward_drop_during_fill_cannot_stall_feedback();
    std::cout << "TCC V9 measured-plant host tests passed\n";
    return 0;
}
