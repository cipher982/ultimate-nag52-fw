#ifdef NDEBUG
#undef NDEBUG
#endif

#include "tcc_transient_controller.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

struct PlantProfile {
    const char* name;
    double contact_pressure_mbar;
    double clutch_gain_rpm_per_second_per_mbar;
    double hydraulic_tau_ms;
    int hydraulic_delay_cycles;
    double initial_slip_rpm;
    double free_slip_rpm;
    double free_slip_ramp_rpm_per_second;
    double free_slip_tau_seconds;
    int feedforward_pressure_mbar;
    bool must_settle;
};

struct ScenarioResult {
    std::string name;
    bool contact_detected = false;
    bool settled = false;
    int contact_cycle = -1;
    int settle_cycle = -1;
    int max_pressure_mbar = 0;
    int max_closure_rpm_per_cycle = 0;
    int min_slip_rpm = 10000;
    int final_slip_rpm = 0;
    int max_target_error_last_second_rpm = 0;
    int max_negative_rate_correction_mbar = 0;
    int max_pressure_after_rate_fault_mbar = 0;
    int max_closure_after_rate_fault_rpm = 0;
    int min_slip_after_rate_fault_rpm = 10000;
    double max_hydraulic_pressure_after_rate_fault_mbar = 0;
    bool excessive_rate_fault = false;
    bool contact_timeout_fault = false;
    int rate_fault_cycle = -1;
    int max_pressure_during_rate_cooldown_mbar = 0;
};

struct SweepCell {
    std::string id;
    PlantProfile profile;
    ScenarioResult result;
    std::string classification;
};

struct SweepSummary {
    int scenarios = 0;
    int qualified = 0;
    int controlled_aborts = 0;
    int comfort_misses = 0;
    int tracking_misses = 0;
    int safety_failures = 0;
    bool conservation_passed = false;
    bool regression_passed = false;
    bool machine_passed = false;
    std::vector<SweepCell> cells;
};

constexpr int kSweepCellCount = 216;
constexpr int kBaselineQualifiedMinimum = 15;
constexpr int kBaselineControlledAbortsMaximum = 100;
constexpr int kBaselineComfortMissesMaximum = 93;
constexpr int kBaselineTrackingMissesMaximum = 8;
constexpr int kBaselineSafetyFailuresExact = 0;

std::string sweep_cell_id(
    double contact_pressure,
    double clutch_gain,
    double hydraulic_tau_ms,
    int hydraulic_delay,
    int feedforward_pressure
) {
    std::ostringstream id;
    id << "sweep-cp" << static_cast<int>(std::lround(contact_pressure))
       << "-g" << static_cast<int>(std::lround(clutch_gain * 10.0))
       << "-tau" << static_cast<int>(std::lround(hydraulic_tau_ms))
       << "-d" << hydraulic_delay
       << "-ff" << feedforward_pressure;
    return id.str();
}

void write_scenario_json(std::ostream& output, const ScenarioResult& result) {
    output << "    {\n"
           << "      \"name\": \"" << result.name << "\",\n"
           << "      \"contact_detected\": " <<
                (result.contact_detected ? "true" : "false") << ",\n"
           << "      \"settled\": " << (result.settled ? "true" : "false") << ",\n"
           << "      \"contact_ms\": " <<
                result.contact_cycle * TccTransientCalibration::kCycleMs << ",\n"
           << "      \"settle_ms\": " <<
                result.settle_cycle * TccTransientCalibration::kCycleMs << ",\n"
           << "      \"max_pressure_mbar\": " << result.max_pressure_mbar << ",\n"
           << "      \"max_closure_rpm_per_cycle\": " <<
                result.max_closure_rpm_per_cycle << ",\n"
           << "      \"min_slip_rpm\": " << result.min_slip_rpm << ",\n"
           << "      \"final_slip_rpm\": " << result.final_slip_rpm << ",\n"
           << "      \"max_target_error_last_second_rpm\": " <<
                result.max_target_error_last_second_rpm << ",\n"
           << "      \"max_negative_rate_correction_mbar\": " <<
                result.max_negative_rate_correction_mbar << ",\n"
           << "      \"max_pressure_after_rate_fault_mbar\": " <<
                result.max_pressure_after_rate_fault_mbar << ",\n"
           << "      \"max_closure_after_rate_fault_rpm\": " <<
                result.max_closure_after_rate_fault_rpm << ",\n"
           << "      \"min_slip_after_rate_fault_rpm\": " <<
                result.min_slip_after_rate_fault_rpm << ",\n"
           << "      \"max_hydraulic_pressure_after_rate_fault_mbar\": " <<
                std::fixed << std::setprecision(3) <<
                result.max_hydraulic_pressure_after_rate_fault_mbar << ",\n"
           << "      \"excessive_rate_fault\": " <<
                (result.excessive_rate_fault ? "true" : "false") << ",\n"
           << "      \"contact_timeout_fault\": " <<
                (result.contact_timeout_fault ? "true" : "false") << ",\n"
           << "      \"rate_fault_ms\": ";
    if (result.rate_fault_cycle >= 0) {
        output << result.rate_fault_cycle * TccTransientCalibration::kCycleMs;
    } else {
        output << "null";
    }
    output << ",\n"
           << "      \"max_pressure_during_rate_cooldown_mbar\": " <<
                result.max_pressure_during_rate_cooldown_mbar << "\n"
           << "    }";
}

bool write_json_summary(
    const char* path,
    const std::vector<ScenarioResult>& results,
    const SweepSummary& summary
) {
    std::ofstream output(path);
    if (!output) return false;

    output << "{\n"
           << "  \"schema\": \"tcc-closed-loop-v2\",\n"
           << "  \"passed\": " <<
                (summary.machine_passed ? "true" : "false") << ",\n"
           << "  \"hard_controller_verdict\": \"" <<
                (summary.safety_failures == kBaselineSafetyFailuresExact ? "PASS" : "FAIL")
           << "\",\n"
           << "  \"baseline_regression_verdict\": \"" <<
                (summary.regression_passed ? "PASS" : "FAIL") << "\",\n"
           << "  \"summary\": {\n"
           << "    \"scenarios\": " << summary.scenarios << ",\n"
           << "    \"sweep_cells\": " << summary.cells.size() << ",\n"
           << "    \"qualified\": " << summary.qualified << ",\n"
           << "    \"controlled_aborts\": " << summary.controlled_aborts << ",\n"
           << "    \"comfort_misses\": " << summary.comfort_misses << ",\n"
           << "    \"tracking_misses\": " << summary.tracking_misses << ",\n"
           << "    \"safety_failures\": " << summary.safety_failures << ",\n"
           << "    \"baseline_regression_budget\": {\n"
           << "      \"provenance\": \"baseline_regression_budget\",\n"
           << "      \"qualified_minimum\": " << kBaselineQualifiedMinimum << ",\n"
           << "      \"controlled_aborts_maximum\": " <<
                kBaselineControlledAbortsMaximum << ",\n"
           << "      \"comfort_misses_maximum\": " <<
                kBaselineComfortMissesMaximum << ",\n"
           << "      \"tracking_misses_maximum\": " <<
                kBaselineTrackingMissesMaximum << ",\n"
           << "      \"safety_failures_exact\": " <<
                kBaselineSafetyFailuresExact << ",\n"
           << "      \"passed\": " <<
                (summary.regression_passed ? "true" : "false") << "\n"
           << "    }\n"
           << "  },\n"
           << "  \"scenarios\": [\n";
    for (size_t index = 0; index < results.size(); ++index) {
        write_scenario_json(output, results[index]);
        output << (index + 1 == results.size() ? "\n" : ",\n");
    }
    output << "  ],\n"
           << "  \"sweep_cells\": [\n";
    for (size_t index = 0; index < summary.cells.size(); ++index) {
        const auto& cell = summary.cells[index];
        output << "    {\n"
               << "      \"id\": \"" << cell.id << "\",\n"
               << "      \"plant\": {\n"
               << "        \"contact_pressure_mbar\": " << std::fixed <<
                    std::setprecision(3) << cell.profile.contact_pressure_mbar << ",\n"
               << "        \"clutch_gain_rpm_per_second_per_mbar\": " <<
                    cell.profile.clutch_gain_rpm_per_second_per_mbar << ",\n"
               << "        \"hydraulic_tau_ms\": " << cell.profile.hydraulic_tau_ms << ",\n"
               << "        \"hydraulic_delay_cycles\": " <<
                    cell.profile.hydraulic_delay_cycles << ",\n"
               << "        \"initial_slip_rpm\": " << cell.profile.initial_slip_rpm << ",\n"
               << "        \"free_slip_rpm\": " << cell.profile.free_slip_rpm << ",\n"
               << "        \"free_slip_ramp_rpm_per_second\": " <<
                    cell.profile.free_slip_ramp_rpm_per_second << ",\n"
               << "        \"free_slip_tau_seconds\": " <<
                    cell.profile.free_slip_tau_seconds << ",\n"
               << "        \"feedforward_pressure_mbar\": " <<
                    cell.profile.feedforward_pressure_mbar << "\n"
               << "      },\n"
               << "      \"metrics\": {\n"
               << "        \"contact_detected\": " <<
                    (cell.result.contact_detected ? "true" : "false") << ",\n"
               << "        \"settled\": " <<
                    (cell.result.settled ? "true" : "false") << ",\n"
               << "        \"contact_ms\": " <<
                    cell.result.contact_cycle * TccTransientCalibration::kCycleMs << ",\n"
               << "        \"settle_ms\": " <<
                    cell.result.settle_cycle * TccTransientCalibration::kCycleMs << ",\n"
               << "        \"max_pressure_mbar\": " << cell.result.max_pressure_mbar << ",\n"
               << "        \"max_closure_rpm_per_cycle\": " <<
                    cell.result.max_closure_rpm_per_cycle << ",\n"
               << "        \"min_slip_rpm\": " << cell.result.min_slip_rpm << ",\n"
               << "        \"final_slip_rpm\": " << cell.result.final_slip_rpm << ",\n"
               << "        \"max_target_error_last_second_rpm\": " <<
                    cell.result.max_target_error_last_second_rpm << ",\n"
               << "        \"max_negative_rate_correction_mbar\": " <<
                    cell.result.max_negative_rate_correction_mbar << ",\n"
               << "        \"max_pressure_after_rate_fault_mbar\": " <<
                    cell.result.max_pressure_after_rate_fault_mbar << ",\n"
               << "        \"max_closure_after_rate_fault_rpm\": " <<
                    cell.result.max_closure_after_rate_fault_rpm << ",\n"
               << "        \"min_slip_after_rate_fault_rpm\": " <<
                    cell.result.min_slip_after_rate_fault_rpm << ",\n"
               << "        \"max_hydraulic_pressure_after_rate_fault_mbar\": " <<
                    cell.result.max_hydraulic_pressure_after_rate_fault_mbar << ",\n"
               << "        \"excessive_rate_fault\": " <<
                    (cell.result.excessive_rate_fault ? "true" : "false") << ",\n"
               << "        \"contact_timeout_fault\": " <<
                    (cell.result.contact_timeout_fault ? "true" : "false") << ",\n"
               << "        \"rate_fault_ms\": ";
        if (cell.result.rate_fault_cycle >= 0) {
            output << cell.result.rate_fault_cycle * TccTransientCalibration::kCycleMs;
        } else {
            output << "null";
        }
        output << ",\n"
               << "        \"max_pressure_during_rate_cooldown_mbar\": " <<
                    cell.result.max_pressure_during_rate_cooldown_mbar << "\n"
               << "      },\n"
               << "      \"classification\": \"" << cell.classification << "\"\n"
               << "    }" << (index + 1 == summary.cells.size() ? "\n" : ",\n");
    }
    output << "  ]\n"
           << "}\n";
    output.close();
    return output.good();
}

class SimpleTccPlant {
public:
    explicit SimpleTccPlant(const PlantProfile& profile)
        : profile_(profile), slip_rpm_(profile.initial_slip_rpm),
          delayed_commands_(profile.hydraulic_delay_cycles + 1, 0) {}

    int measured_slip_rpm() const { return (int)std::lround(slip_rpm_); }
    double hydraulic_pressure_mbar() const { return hydraulic_pressure_mbar_; }

    void step(int commanded_pressure_mbar) {
        const double free_slip_rpm = profile_.free_slip_rpm +
            profile_.free_slip_ramp_rpm_per_second * elapsed_seconds_;
        step_toward(commanded_pressure_mbar, free_slip_rpm);
    }

    void step_toward(int commanded_pressure_mbar, double free_slip_rpm) {
        delayed_commands_.push_back(commanded_pressure_mbar);
        const int delayed_command = delayed_commands_.front();
        delayed_commands_.pop_front();

        const double hydraulic_fraction = std::min(1.0,
            TccTransientCalibration::kCycleMs / profile_.hydraulic_tau_ms);
        hydraulic_pressure_mbar_ += (delayed_command - hydraulic_pressure_mbar_) * hydraulic_fraction;

        const double open_converter_rate =
            (free_slip_rpm - slip_rpm_) / profile_.free_slip_tau_seconds;
        const double clutch_pressure = std::max(0.0,
            hydraulic_pressure_mbar_ - profile_.contact_pressure_mbar);
        const double clutch_rate = clutch_pressure * profile_.clutch_gain_rpm_per_second_per_mbar;
        // Friction torque opposes relative motion. It cannot continue pushing
        // engine and turbine apart after their speed ordering crosses zero.
        const double clutch_direction = slip_rpm_ >= 0.0 ? 1.0 : -1.0;
        const double slip_rate = open_converter_rate - clutch_direction * clutch_rate;
        slip_rpm_ += slip_rate * (TccTransientCalibration::kCycleMs / 1000.0);
        slip_rpm_ = std::max(-500.0, std::min(1000.0, slip_rpm_));
        elapsed_seconds_ += TccTransientCalibration::kCycleMs / 1000.0;
    }

private:
    PlantProfile profile_;
    double slip_rpm_;
    double hydraulic_pressure_mbar_ = 0.0;
    double elapsed_seconds_ = 0.0;
    std::deque<int> delayed_commands_;
};

ScenarioResult run_apply(const PlantProfile& profile) {
    TccTransientController controller;
    controller.select_gear(2);
    SimpleTccPlant plant(profile);
    ScenarioResult result{profile.name};
    int previous_slip = plant.measured_slip_rpm();
    int previous_pressure = 0;
    int in_band_cycles = 0;

    for (int cycle = 0; cycle < 250; ++cycle) {
        const int slip = plant.measured_slip_rpm();
        const auto output = controller.step({
            true,
            false,
            true,
            slip,
            50,
            profile.feedforward_pressure_mbar,
            (uint32_t)(cycle * TccTransientCalibration::kCycleMs),
        });

        const int pressure_delta = output.pressure - previous_pressure;
        assert(output.pressure >= 0);
        assert(output.pressure <= TccTransientCalibration::kMaxCommandPressure);
        assert(output.pressure <= std::min(
            profile.feedforward_pressure_mbar + TccTransientCalibration::kFeedbackHeadroom,
            TccTransientCalibration::kMaxCommandPressure));
        assert(pressure_delta <= TccTransientCalibration::kApplySlewPerCycle);
        if (output.state != TccTransientState::ReleaseFault) {
            assert(pressure_delta >= -TccTransientCalibration::kFeedbackReliefSlewPerCycle);
        }

        if (output.reason == TccTransientReason::ExcessiveSlipRate) {
            result.excessive_rate_fault = true;
            if (result.rate_fault_cycle < 0) result.rate_fault_cycle = cycle;
        }
        if (output.reason == TccTransientReason::ContactNotDetected) {
            result.contact_timeout_fault = true;
        }
        if (result.excessive_rate_fault) {
            result.max_pressure_after_rate_fault_mbar = std::max(
                result.max_pressure_after_rate_fault_mbar, output.pressure);
        }

        if (output.contact_detected && !result.contact_detected) {
            result.contact_detected = true;
            result.contact_cycle = cycle;
        }
        result.max_pressure_mbar = std::max(result.max_pressure_mbar, output.pressure);
        result.max_negative_rate_correction_mbar = std::min(
            result.max_negative_rate_correction_mbar, output.slip_rate_correction);

        plant.step(output.pressure);
        const int next_slip = plant.measured_slip_rpm();
        if (result.excessive_rate_fault) {
            result.max_closure_after_rate_fault_rpm = std::max(
                result.max_closure_after_rate_fault_rpm, previous_slip - next_slip);
            result.min_slip_after_rate_fault_rpm = std::min(
                result.min_slip_after_rate_fault_rpm, next_slip);
            result.max_hydraulic_pressure_after_rate_fault_mbar = std::max(
                result.max_hydraulic_pressure_after_rate_fault_mbar,
                plant.hydraulic_pressure_mbar());
        }
        if (result.rate_fault_cycle >= 0 &&
            cycle - result.rate_fault_cycle <
                TccTransientCalibration::kFaultRetryCooldownMs /
                    TccTransientCalibration::kCycleMs) {
            result.max_pressure_during_rate_cooldown_mbar = std::max(
                result.max_pressure_during_rate_cooldown_mbar, output.pressure);
        }
        result.max_closure_rpm_per_cycle = std::max(
            result.max_closure_rpm_per_cycle, previous_slip - next_slip);
        result.min_slip_rpm = std::min(result.min_slip_rpm, next_slip);
        if (cycle >= 200) {
            result.max_target_error_last_second_rpm = std::max(
                result.max_target_error_last_second_rpm, std::abs(next_slip - 50));
        }

        if (std::abs(next_slip - 50) <= 25) {
            in_band_cycles += 1;
            if (in_band_cycles >= 5 && !result.settled) {
                result.settled = true;
                result.settle_cycle = cycle;
            }
        } else {
            in_band_cycles = 0;
        }
        previous_slip = next_slip;
        previous_pressure = output.pressure;
        result.final_slip_rpm = next_slip;
    }
    return result;
}

void assert_fault_release() {
    TccTransientController controller;
    controller.select_gear(2);
    TccTransientOutput output{};
    for (int cycle = 0; cycle < 20; ++cycle) {
        const int slip = 260 - cycle * 3;
        output = controller.step({true, false, true, slip, 50, 1200,
            (uint32_t)(cycle * TccTransientCalibration::kCycleMs)});
    }
    assert(output.pressure > 0);
    output = controller.step({true, false, false, 150, 50, 1200, 420});
    assert(output.state == TccTransientState::ReleaseFault);
    assert(output.reason == TccTransientReason::InvalidSpeed);
    assert(output.pressure == 0);

    controller.reset();
    controller.select_gear(2);
    controller.step({true, false, true, 260, 50, 1200, 0});
    controller.step({true, false, true, 250, 50, 1200, 20});
    controller.step({true, false, true, 240, 50, 1200, 40});
    controller.step({true, false, true, 230, 50, 1200, 60});
    output = controller.step({true, false, true, 190, 50, 1200, 80});
    assert(output.reason != TccTransientReason::ExcessiveSlipRate);
    output = controller.step({true, false, true, 150, 50, 1200, 100});
    assert(output.state == TccTransientState::ReleaseFault);
    assert(output.reason == TccTransientReason::ExcessiveSlipRate);
    assert(output.pressure == 0);
    output = controller.step({true, false, true, 190, 50, 1200, 100});
    assert(output.state == TccTransientState::ReleaseFault);
    assert(output.pressure == 0);
    output = controller.step({false, false, true, 190, 120, 1200, 120});
    assert(output.state == TccTransientState::Open);
}

void assert_coast_overrun_closed_loop() {
    const PlantProfile profile = {
        "coast-overrun", 700, 2.0, 80, 2, 80, 80, 0, 0.18, 1200, false,
    };
    TccTransientController controller;
    controller.select_gear(2);
    SimpleTccPlant plant(profile);
    TccTransientOutput output{};

    // Establish a pressured application before introducing the logged coast
    // disturbance. This is a plant/controller loop, not a canned slip trace.
    for (int cycle = 0; cycle < 30; ++cycle) {
        output = controller.step({true, false, true, plant.measured_slip_rpm(),
            10, 1200, (uint32_t)(cycle * TccTransientCalibration::kCycleMs), false});
        plant.step_toward(output.pressure, 80.0);
    }
    assert(output.pressure > 0);

    bool coast_latched = false;
    int min_coast_slip = plant.measured_slip_rpm();
    for (int cycle = 30; cycle < 130; ++cycle) {
        output = controller.step({true, false, true, plant.measured_slip_rpm(),
            10, 1200, (uint32_t)(cycle * TccTransientCalibration::kCycleMs), true});
        if (output.reason == TccTransientReason::CoastOverrun) {
            coast_latched = true;
            assert(output.state == TccTransientState::Open);
            assert(output.pressure == 0);
        }
        plant.step_toward(output.pressure, -360.0);
        min_coast_slip = std::min(min_coast_slip, plant.measured_slip_rpm());
    }
    assert(coast_latched);
    // Do not hide the measured -327 RPM regime behind the old -200 RPM plant
    // clamp. The open converter is allowed to follow the negative disturbance.
    assert(min_coast_slip <= -327);

    output = controller.step({true, false, true, plant.measured_slip_rpm(),
        50, 1200, 130 * TccTransientCalibration::kCycleMs, false});
    assert(output.state == TccTransientState::Fill);
    assert(output.reason == TccTransientReason::None);
    assert(output.pressure == TccTransientCalibration::kApplySlewPerCycle);

    int previous_pressure = output.pressure;
    for (int cycle = 131; cycle < 151; ++cycle) {
        plant.step_toward(output.pressure, 80.0);
        output = controller.step({true, false, true, plant.measured_slip_rpm(),
            50, 1200, (uint32_t)(cycle * TccTransientCalibration::kCycleMs), false});
        assert(output.reason != TccTransientReason::CoastOverrun);
        assert(output.state != TccTransientState::ReleaseFault);
        assert(output.pressure - previous_pressure <= TccTransientCalibration::kApplySlewPerCycle);
        previous_pressure = output.pressure;
    }
}

void assert_low_slip_cruise() {
    TccTransientController controller;
    controller.select_gear(5);
    bool contact_detected = false;
    for (int cycle = 0; cycle < 500; ++cycle) {
        const int slip = 10 + (cycle % 6) * 2;
        const auto output = controller.step({true, false, true, slip, 10, 1100,
            (uint32_t)(cycle * TccTransientCalibration::kCycleMs)});
        assert(output.reason != TccTransientReason::ContactNotDetected);
        assert(output.state != TccTransientState::ReleaseFault);
        assert(output.pressure <= 1100 + TccTransientCalibration::kFeedbackHeadroom);
        contact_detected = contact_detected || output.contact_detected;
    }
    assert(contact_detected);
}

void assert_positive_feedback_headroom() {
    TccTransientController controller;
    controller.select_gear(5);
    const int feedforward = 1100;
    controller.step({true, false, true, 30, 10, feedforward, 0});
    controller.step({true, false, true, 25, 10, feedforward, 20});
    auto output = controller.step({true, false, true, 20, 10, feedforward, 40});
    assert(output.contact_detected);

    bool added_pressure = false;
    int maximum_pressure = output.pressure;
    for (int cycle = 3; cycle < 180; ++cycle) {
        output = controller.step({true, false, true, 40, 10, feedforward,
            (uint32_t)(cycle * TccTransientCalibration::kCycleMs)});
        maximum_pressure = std::max(maximum_pressure, output.pressure);
        added_pressure = added_pressure || output.pressure > feedforward;
        assert(output.pressure <= feedforward + TccTransientCalibration::kFeedbackHeadroom);
    }
    assert(added_pressure);
    assert(maximum_pressure == feedforward + TccTransientCalibration::kFeedbackHeadroom);
}

void assert_integrator_deadband() {
    TccTransientController controller;
    controller.select_gear(5);
    const int feedforward = 1100;
    auto output = controller.step({true, false, true, 10, 10, feedforward, 0});
    assert(output.contact_detected);
    const int initial_integral = output.integral_correction;
    for (int cycle = 1; cycle <= 250; ++cycle) {
        const int slip = (cycle % 2 == 0) ? 5 : 15;
        output = controller.step({true, false, true, slip, 10, feedforward,
            (uint32_t)(cycle * TccTransientCalibration::kCycleMs)});
        if (cycle >= 35) {
            assert(output.state == TccTransientState::Locked);
        }
        assert(output.integral_correction == initial_integral);
        assert(output.pressure <= feedforward + TccTransientCalibration::kFeedbackHeadroom);
    }
}

void assert_quantization_noise_and_retry() {
    TccTransientController controller;
    controller.select_gear(5);
    controller.step({true, false, true, 100, 50, 1200, 0});
    auto output = controller.step({true, false, true, 74, 50, 1200, 20});
    assert(output.reason != TccTransientReason::ExcessiveSlipRate);
    output = controller.step({true, false, true, 74, 50, 1200, 40});
    assert(output.reason != TccTransientReason::ExcessiveSlipRate);

    controller.reset();
    controller.select_gear(5);
    for (int cycle = 0; cycle < 150; ++cycle) {
        output = controller.step({true, false, true, 331, 89, 1200,
            (uint32_t)(cycle * TccTransientCalibration::kCycleMs)});
    }
    output = controller.step({true, false, true, 331, 89, 1200, 3000});
    assert(output.reason == TccTransientReason::ContactNotDetected);
    for (uint32_t now = 3020; now < 5000; now += TccTransientCalibration::kCycleMs) {
        output = controller.step({true, false, true, 331, 89, 1200, now});
        assert(output.pressure == 0);
        assert(output.reason == TccTransientReason::ContactNotDetected);
    }
    output = controller.step({true, false, true, 331, 89, 1200, 5000});
    assert(output.state == TccTransientState::Fill);
    assert(output.pressure == TccTransientCalibration::kApplySlewPerCycle);
}

void assert_pedal_flutter_gate() {
    bool coast_mode = false;
    const uint16_t raw_pedal[] = {38, 55, 40, 52, 42, 50};
    for (const uint16_t raw : raw_pedal) {
        coast_mode = tcc_transient_update_coast_mode(
            coast_mode, tcc_transient_pedal_percent(raw), false);
        assert(!coast_mode);
    }
}

SweepSummary assert_parameter_sweep() {
    constexpr int kComfortClosureRpmPerCycle = 12;
    const double contact_pressures[] = {600.0, 700.0, 750.0};
    const double clutch_gains[] = {1.5, 3.0, 5.0};
    const double hydraulic_taus_ms[] = {40.0, 80.0, 120.0, 160.0};
    const int hydraulic_delays[] = {1, 2, 4};
    const int feedforward_pressures[] = {1100, 1300};
    int scenarios = 0;
    int qualified = 0;
    int controlled_aborts = 0;
    int comfort_misses = 0;
    int tracking_misses = 0;
    int safety_failures = 0;
    int printed_safety_failures = 0;
    int printed_comfort_misses = 0;
    std::vector<SweepCell> cells;
    cells.reserve(kSweepCellCount);

    for (const double contact_pressure : contact_pressures) {
        for (const double clutch_gain : clutch_gains) {
            for (const double hydraulic_tau_ms : hydraulic_taus_ms) {
                for (const int hydraulic_delay : hydraulic_delays) {
                    for (const int feedforward_pressure : feedforward_pressures) {
                        const PlantProfile profile = {
                            "sweep",
                            contact_pressure,
                            clutch_gain,
                            hydraulic_tau_ms,
                            hydraulic_delay,
                            260.0,
                            300.0,
                            0.0,
                            0.50,
                            feedforward_pressure,
                            true,
                        };
                        const ScenarioResult result = run_apply(profile);
                        // A reactive guard observes one over-rate cycle before
                        // it can release. Safety after that release is judged
                        // on the modeled physical slip, including residual
                        // hydraulic pressure, rather than command pressure alone.
                        const bool closure_safe = result.excessive_rate_fault
                            ? result.min_slip_after_rate_fault_rpm >= -30
                            : result.max_closure_rpm_per_cycle <=
                                TccTransientCalibration::kHardSlipClosurePerCycle + 10;
                        const bool hard_safe = result.min_slip_rpm >= -30 &&
                            closure_safe &&
                            result.max_pressure_during_rate_cooldown_mbar == 0;
                        const bool functionally_settled = !result.excessive_rate_fault &&
                            result.contact_detected && result.settled &&
                            result.settle_cycle * TccTransientCalibration::kCycleMs <= 4000 &&
                            result.max_target_error_last_second_rpm <= 30 &&
                            result.max_closure_rpm_per_cycle <=
                                TccTransientCalibration::kHardSlipClosurePerCycle &&
                            result.min_slip_rpm >= -30;
                        const bool passed = functionally_settled &&
                            result.max_closure_rpm_per_cycle <= kComfortClosureRpmPerCycle;
                        scenarios += 1;
                        std::string classification;
                        if (!hard_safe) {
                            classification = "safety_failure";
                            if (printed_safety_failures < 8) {
                                std::cout << "safety failure contact=" << contact_pressure
                                          << " gain=" << clutch_gain
                                          << " tau_ms=" << hydraulic_tau_ms
                                          << " delay=" << hydraulic_delay
                                          << " feedforward=" << feedforward_pressure
                                          << " max_closure=" << result.max_closure_rpm_per_cycle
                                          << " post_fault_max_closure=" <<
                                                result.max_closure_after_rate_fault_rpm
                                          << " min_slip=" << result.min_slip_rpm
                                          << " post_fault_min_slip=" <<
                                                result.min_slip_after_rate_fault_rpm
                                          << '\n';
                                printed_safety_failures += 1;
                            }
                            safety_failures += 1;
                        } else if (passed) {
                            classification = "qualified";
                            qualified += 1;
                        } else if (result.excessive_rate_fault || result.contact_timeout_fault) {
                            classification = "controlled_abort";
                            controlled_aborts += 1;
                        } else if (functionally_settled) {
                            classification = "comfort_miss";
                            if (printed_comfort_misses < 4) {
                                std::cout << "comfort miss contact=" << contact_pressure
                                          << " gain=" << clutch_gain
                                          << " tau_ms=" << hydraulic_tau_ms
                                          << " delay=" << hydraulic_delay
                                          << " feedforward=" << feedforward_pressure
                                          << " max_closure=" << result.max_closure_rpm_per_cycle
                                          << '\n';
                                printed_comfort_misses += 1;
                            }
                            comfort_misses += 1;
                        } else {
                            classification = "tracking_miss";
                            if (tracking_misses < 4) {
                                std::cout << "tracking miss contact=" << contact_pressure
                                          << " gain=" << clutch_gain
                                          << " tau_ms=" << hydraulic_tau_ms
                                          << " delay=" << hydraulic_delay
                                          << " feedforward=" << feedforward_pressure
                                          << " settle_ms=" << result.settle_cycle *
                                                TccTransientCalibration::kCycleMs
                                          << " final_window_error=" <<
                                                result.max_target_error_last_second_rpm
                                          << '\n';
                            }
                            tracking_misses += 1;
                        }
                        cells.push_back({
                            sweep_cell_id(
                                contact_pressure, clutch_gain, hydraulic_tau_ms,
                                hydraulic_delay, feedforward_pressure),
                            profile,
                            result,
                            classification,
                        });
                    }
                }
            }
        }
    }

    std::cout << "parameter sweep scenarios=" << scenarios
              << " qualified=" << qualified
              << " controlled_aborts=" << controlled_aborts
              << " comfort_misses=" << comfort_misses
               << " tracking_misses=" << tracking_misses
               << " safety_failures=" << safety_failures << '\n';
    const bool conservation_passed = scenarios == kSweepCellCount &&
        cells.size() == static_cast<size_t>(kSweepCellCount) &&
        qualified + controlled_aborts + comfort_misses + tracking_misses +
            safety_failures == kSweepCellCount;
    const bool regression_passed = qualified >= kBaselineQualifiedMinimum &&
        controlled_aborts <= kBaselineControlledAbortsMaximum &&
        comfort_misses <= kBaselineComfortMissesMaximum &&
        tracking_misses <= kBaselineTrackingMissesMaximum &&
        safety_failures == kBaselineSafetyFailuresExact;
    const bool machine_passed = conservation_passed && regression_passed;
    if (!conservation_passed) {
        std::cerr << "parameter sweep failed cell conservation\n";
    }
    if (!regression_passed) {
        std::cerr << "parameter sweep failed baseline regression budget\n";
    }
    return {
        scenarios,
        qualified,
        controlled_aborts,
        comfort_misses,
        tracking_misses,
        safety_failures,
        conservation_passed,
        regression_passed,
        machine_passed,
        std::move(cells),
    };
}

}

int main(int argc, char** argv) {
    const char* json_summary_path = nullptr;
    if (argc == 3 && std::string(argv[1]) == "--json-summary" && argv[2][0] != '\0') {
        json_summary_path = argv[2];
    } else if (argc != 1) {
        std::cerr << "usage: " << argv[0] << " [--json-summary PATH]\n";
        return 2;
    }
    const std::vector<PlantProfile> profiles = {
        {"nominal", 700, 3.0, 80, 2, 260, 300, 0, 0.50, 1200, true},
        {"sharp-controlled-abort", 750, 5.0, 40, 1, 260, 300, 0, 0.45, 1300, false},
        {"soft", 600, 1.5, 120, 3, 260, 300, 0, 0.60, 1100, true},
        {"cold-slow-above-effective-cap", 780, 2.0, 160, 4, 260, 320, 0, 0.70, 1400, false},
        {"contact-above-search-cap", 850, 3.0, 80, 2, 260, 300, 0, 0.50, 1400, false},
        {"falling-slip-contact-above-cap", 850, 3.0, 80, 2, 243, 243, -35, 0.50, 1400, false},
        {"true-no-contact", 1500, 3.0, 80, 2, 260, 300, 0, 0.50, 1400, false},
        {"falling-slip-true-no-contact", 1500, 3.0, 80, 2, 243, 243, -35, 0.50, 1400, false},
    };

    std::vector<ScenarioResult> results;
    results.reserve(profiles.size());

    for (const auto& profile : profiles) {
        const ScenarioResult result = run_apply(profile);
        std::cout << result.name
                  << " contact=" << result.contact_detected
                  << " settled=" << result.settled
                  << " contact_ms=" << result.contact_cycle * TccTransientCalibration::kCycleMs
                  << " settle_ms=" << result.settle_cycle * TccTransientCalibration::kCycleMs
                  << " max_pressure=" << result.max_pressure_mbar
                  << " max_closure_per_cycle=" << result.max_closure_rpm_per_cycle
                  << " min_slip=" << result.min_slip_rpm
                  << " final_slip=" << result.final_slip_rpm
                  << " final_window_error=" << result.max_target_error_last_second_rpm
                  << " max_rate_relief=" << result.max_negative_rate_correction_mbar
                  << " post_fault_max_closure=" << result.max_closure_after_rate_fault_rpm
                  << " post_fault_min_slip=" << result.min_slip_after_rate_fault_rpm
                  << " post_fault_hydraulic_pressure=" <<
                        result.max_hydraulic_pressure_after_rate_fault_mbar
                  << " rate_fault=" << result.excessive_rate_fault
                  << " contact_timeout=" << result.contact_timeout_fault
                  << '\n';

        assert(result.min_slip_rpm >= -30);
        if (profile.must_settle) {
            assert(!result.excessive_rate_fault);
            assert(result.max_closure_rpm_per_cycle <=
                TccTransientCalibration::kHardSlipClosurePerCycle);
            assert(result.contact_detected);
            assert(result.settled);
            assert(result.settle_cycle * TccTransientCalibration::kCycleMs <= 4000);
            assert(result.max_target_error_last_second_rpm <= 30);
            if (result.max_closure_rpm_per_cycle >
                TccTransientCalibration::kMaxDesiredSlipClosurePerCycle) {
                assert(result.max_negative_rate_correction_mbar < 0);
            }
        } else {
            assert(result.excessive_rate_fault || result.contact_timeout_fault);
            assert(result.max_pressure_during_rate_cooldown_mbar == 0);
            if (result.excessive_rate_fault) {
                assert(result.min_slip_after_rate_fault_rpm >= -30);
            }
            if (result.contact_timeout_fault) {
                assert(!result.contact_detected);
                assert(result.max_pressure_mbar == std::min(
                    profile.feedforward_pressure_mbar,
                    TccTransientCalibration::kContactSearchPressure));
            }
        }
        results.push_back(result);
    }

    assert_fault_release();
    assert_coast_overrun_closed_loop();
    assert_low_slip_cruise();
    assert_positive_feedback_headroom();
    assert_integrator_deadband();
    assert_quantization_noise_and_retry();
    assert_pedal_flutter_gate();
    const SweepSummary summary = assert_parameter_sweep();
    if (json_summary_path != nullptr &&
        !write_json_summary(json_summary_path, results, summary)) {
        std::cerr << "failed to write JSON summary: " << json_summary_path << '\n';
        return 2;
    }
    if (!summary.machine_passed) {
        std::cerr << "host TCC closed-loop simulation failed machine verdict\n";
        return 1;
    }
    std::cout << "host TCC closed-loop simulation passed\n";
    return 0;
}
