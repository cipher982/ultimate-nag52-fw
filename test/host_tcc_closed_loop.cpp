#include "tcc_transient_controller.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <deque>
#include <iostream>
#include <string>
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
    bool excessive_rate_fault = false;
    bool contact_timeout_fault = false;
};

class SimpleTccPlant {
public:
    explicit SimpleTccPlant(const PlantProfile& profile)
        : profile_(profile), slip_rpm_(profile.initial_slip_rpm),
          delayed_commands_(profile.hydraulic_delay_cycles + 1, 0) {}

    int measured_slip_rpm() const { return (int)std::lround(slip_rpm_); }

    void step(int commanded_pressure_mbar) {
        delayed_commands_.push_back(commanded_pressure_mbar);
        const int delayed_command = delayed_commands_.front();
        delayed_commands_.pop_front();

        const double hydraulic_fraction = std::min(1.0,
            TccTransientCalibration::kCycleMs / profile_.hydraulic_tau_ms);
        hydraulic_pressure_mbar_ += (delayed_command - hydraulic_pressure_mbar_) * hydraulic_fraction;

        const double free_slip_rpm = profile_.free_slip_rpm +
            profile_.free_slip_ramp_rpm_per_second * elapsed_seconds_;
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
        slip_rpm_ = std::max(-200.0, std::min(1000.0, slip_rpm_));
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
        assert(output.pressure <= profile.feedforward_pressure_mbar);
        assert(pressure_delta <= TccTransientCalibration::kApplySlewPerCycle);
        if (output.state != TccTransientState::ReleaseFault) {
            assert(pressure_delta >= -TccTransientCalibration::kFeedbackReliefSlewPerCycle);
        }

        if (output.reason == TccTransientReason::ExcessiveSlipRate) {
            result.excessive_rate_fault = true;
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
    assert(output.state == TccTransientState::ReleaseFault);
    assert(output.reason == TccTransientReason::ExcessiveSlipRate);
    assert(output.pressure == 0);
    output = controller.step({true, false, true, 190, 50, 1200, 100});
    assert(output.state == TccTransientState::ReleaseFault);
    assert(output.pressure == 0);
    output = controller.step({false, false, true, 190, 120, 1200, 120});
    assert(output.state == TccTransientState::Open);
}

void assert_parameter_sweep() {
    const double contact_pressures[] = {600.0, 700.0, 750.0};
    const double clutch_gains[] = {1.5, 3.0, 5.0};
    const double hydraulic_taus_ms[] = {40.0, 80.0, 120.0, 160.0};
    const int hydraulic_delays[] = {1, 2, 4};
    const int feedforward_pressures[] = {1100, 1300};
    int scenarios = 0;
    int qualified = 0;
    int controlled_aborts = 0;
    int tracking_misses = 0;
    int safety_failures = 0;

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
                        const bool hard_safe = result.min_slip_rpm >= -50 &&
                            (result.max_closure_rpm_per_cycle <=
                                TccTransientCalibration::kHardSlipClosurePerCycle ||
                                result.excessive_rate_fault) &&
                            result.max_pressure_after_rate_fault_mbar == 0;
                        const bool passed = !result.excessive_rate_fault &&
                            result.contact_detected && result.settled &&
                            result.settle_cycle * TccTransientCalibration::kCycleMs <= 4000 &&
                            result.max_target_error_last_second_rpm <= 30 &&
                            result.max_closure_rpm_per_cycle <= 30 &&
                            result.min_slip_rpm >= -30;
                        scenarios += 1;
                        if (!hard_safe) {
                            safety_failures += 1;
                        } else if (passed) {
                            qualified += 1;
                        } else if (result.excessive_rate_fault) {
                            controlled_aborts += 1;
                        } else {
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
                    }
                }
            }
        }
    }

    std::cout << "parameter sweep scenarios=" << scenarios
              << " qualified=" << qualified
              << " controlled_aborts=" << controlled_aborts
              << " tracking_misses=" << tracking_misses
              << " safety_failures=" << safety_failures << '\n';
    assert(qualified + controlled_aborts + tracking_misses + safety_failures == scenarios);
    assert(safety_failures == 0);
}

}

int main() {
    const std::vector<PlantProfile> profiles = {
        {"nominal", 700, 3.0, 80, 2, 260, 300, 0, 0.50, 1200, true},
        {"sharp", 750, 5.0, 40, 1, 260, 300, 0, 0.45, 1300, true},
        {"soft", 600, 1.5, 120, 3, 260, 300, 0, 0.60, 1100, true},
        {"cold-slow", 780, 2.0, 160, 4, 260, 320, 0, 0.70, 1400, true},
        {"contact-above-initial-search", 850, 3.0, 80, 2, 260, 300, 0, 0.50, 1400, true},
        {"falling-slip-before-contact", 850, 3.0, 80, 2, 243, 243, -35, 0.50, 1400, true},
        {"true-no-contact", 1500, 3.0, 80, 2, 260, 300, 0, 0.50, 1400, false},
        {"falling-slip-true-no-contact", 1500, 3.0, 80, 2, 243, 243, -35, 0.50, 1400, false},
    };

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
                  << " rate_fault=" << result.excessive_rate_fault
                  << " contact_timeout=" << result.contact_timeout_fault
                  << '\n';

        assert(result.max_closure_rpm_per_cycle <= 30);
        assert(result.min_slip_rpm >= -30);
        if (profile.must_settle) {
            assert(result.contact_detected);
            assert(result.settled);
            assert(result.settle_cycle * TccTransientCalibration::kCycleMs <= 4000);
            assert(result.max_target_error_last_second_rpm <= 30);
            if (result.max_closure_rpm_per_cycle >
                TccTransientCalibration::kMaxDesiredSlipClosurePerCycle) {
                assert(result.max_negative_rate_correction_mbar < 0);
            }
        } else {
            assert(!result.contact_detected);
            assert(result.contact_timeout_fault);
            assert(result.max_pressure_mbar == profile.feedforward_pressure_mbar);
        }
    }

    assert_fault_release();
    assert_parameter_sweep();
    std::cout << "host TCC closed-loop simulation passed\n";
}
