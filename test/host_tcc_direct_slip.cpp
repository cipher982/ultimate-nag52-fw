#include <cstdlib>
#include <iostream>

#include "tcc_direct_slip_controller.h"

namespace {
void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

TccDirectSlipInput enabled_input(
    uint32_t now_ms,
    int slip_rpm,
    int target_rpm = 0,
    int feedforward_mbar = 1500
) {
    return {
        true,
        true,
        true,
        TccDirectSlipReason::None,
        slip_rpm,
        target_rpm,
        feedforward_mbar,
        now_ms,
    };
}

void test_reaches_and_retains_holding_floor_at_zero_slip() {
    TccDirectSlipController controller;
    TccDirectSlipOutput out = {};
    for (uint32_t now = 0; now <= 400; now += 20) {
        out = controller.step(enabled_input(now, 0));
    }
    require(out.state == TccDirectSlipState::Tracking, "zero slip should confirm tracking");
    require(out.pressure_mbar == 1500, "tracking must retain the holding-pressure floor");
    require(!out.fault_latched, "normal tracking must never latch a fault");
}

void test_positive_and_negative_slip_have_identical_control() {
    TccDirectSlipController positive;
    TccDirectSlipController negative;
    for (uint32_t now = 0; now <= 1200; now += 20) {
        const auto pos = positive.step(enabled_input(now, 120));
        const auto neg = negative.step(enabled_input(now, -120));
        require(pos.pressure_mbar == neg.pressure_mbar,
            "equal slip magnitudes must produce equal pressure");
        require(pos.pressure_delta_mbar == neg.pressure_delta_mbar,
            "torque direction must not change controller polarity");
        require(pos.state == neg.state, "torque direction must not change controller state");
    }
}

void test_torque_reversal_never_releases_a_tracking_clutch() {
    TccDirectSlipController controller;
    TccDirectSlipOutput out = {};
    uint32_t now = 0;
    for (; now <= 400; now += 20) {
        out = controller.step(enabled_input(now, 0));
    }
    require(out.tracking_achieved, "precondition: clutch should be tracking");

    const int pressure_before = out.pressure_mbar;
    out = controller.step(enabled_input(now, -90));
    require(out.pressure_mbar >= pressure_before,
        "coast torque must not release holding pressure");
    now += 20;
    out = controller.step(enabled_input(now, 90));
    require(out.pressure_mbar >= pressure_before,
        "return to drive torque must not reacquire from zero");
}

void test_v6_coast_fault_window_applies_instead_of_latching_open() {
    TccDirectSlipController controller;
    TccDirectSlipOutput out = {};
    for (uint32_t now = 0; now <= 4200; now += 20) {
        out = controller.step(enabled_input(now, -85));
        require(out.state != TccDirectSlipState::RetryCooldown,
            "the captured negative-slip coast window is not a qualified failure");
        require(!out.fault_latched, "coast must never create a drive-long fault");
    }
    require(out.pressure_mbar >= TccDirectSlipCalibration::kMinimumHoldPressureMbar,
        "negative-slip coast must actively apply the clutch");
}

void test_feedback_waits_for_hydraulic_response() {
    TccDirectSlipController controller;
    TccDirectSlipOutput out = {};
    for (uint32_t now = 0; now <= 280; now += 20) {
        out = controller.step(enabled_input(now, 300));
    }
    require(out.pressure_mbar == 1500, "controller should stop at the feed-forward floor");

    for (uint32_t now = 300; now < 380; now += 20) {
        out = controller.step(enabled_input(now, 300));
        require(out.pressure_mbar == 1500,
            "feedback must wait for the measured hydraulic response interval");
    }
    out = controller.step(enabled_input(380, 300));
    require(out.pressure_mbar == 1550,
        "feedback should add one bounded step after the response interval");
}

void test_delayed_plants_track_in_both_torque_directions() {
    for (const int direction : {1, -1}) {
        TccDirectSlipController controller;
        TccDirectSlipOutput out = {};
        double applied_pressure = 0;
        double slip_magnitude = 300;
        uint32_t now = 0;
        for (; now < 4000 && !out.tracking_achieved; now += 20) {
            out = controller.step(enabled_input(
                now,
                direction * static_cast<int>(slip_magnitude),
                0,
                1500
            ));
            applied_pressure += (out.pressure_mbar - applied_pressure) / 5.0;
            const double coupling = applied_pressure > 1350
                ? applied_pressure - 1350 : 0;
            const double equilibrium_slip = 300 - coupling > 0
                ? 300 - coupling : 0;
            slip_magnitude += (equilibrium_slip - slip_magnitude) / 4.0;
        }
        require(out.tracking_achieved, "reachable delayed plant should track");
        require(now < 4000, "reachable delayed plant should track before timeout");
        require(!out.fault_latched, "tracking must not latch a fault");
    }
}

void test_shift_and_mismatch_open_then_reapply_immediately() {
    TccDirectSlipController controller;
    auto out = controller.step(enabled_input(0, 300));
    require(out.pressure_mbar == 100, "precondition: application should begin immediately");

    auto input = enabled_input(20, 300);
    input.inhibit_reason = TccDirectSlipReason::ShiftActive;
    out = controller.step(input);
    require(out.state == TccDirectSlipState::ShiftInhibit && out.pressure_mbar == 0,
        "shift activity should open immediately");

    input = enabled_input(40, 300);
    input.inhibit_reason = TccDirectSlipReason::GearMismatch;
    out = controller.step(input);
    require(out.state == TccDirectSlipState::ShiftInhibit && out.pressure_mbar == 0,
        "actual/target mismatch should remain open");

    out = controller.step(enabled_input(60, 300));
    require(out.state == TccDirectSlipState::Applying && out.pressure_mbar == 100,
        "stable post-shift gear should reapply without a torque gate");
}

void test_unresponsive_plant_cools_down_and_retries() {
    TccDirectSlipController controller;
    TccDirectSlipOutput out = {};
    uint32_t cooldown_started_ms = 0;
    int maximum_pressure_mbar = 0;
    for (uint32_t now = 0; now < 8000; now += 20) {
        out = controller.step(enabled_input(now, 300));
        if (out.pressure_mbar > maximum_pressure_mbar) {
            maximum_pressure_mbar = out.pressure_mbar;
        }
        require(out.pressure_mbar <= TccDirectSlipCalibration::kMaxCommandPressureMbar,
            "feedback must never exceed the command-pressure cap");
        if (out.state == TccDirectSlipState::RetryCooldown) {
            cooldown_started_ms = now;
            break;
        }
    }
    require(cooldown_started_ms > 0, "qualified nontracking should enter cooldown");
    require(maximum_pressure_mbar == TccDirectSlipCalibration::kMaxCommandPressureMbar,
        "an unresponsive plant should reach the bounded pressure cap before retry");
    require(out.pressure_mbar == 0, "cooldown must release the clutch");
    require(!out.fault_latched, "ordinary nontracking must not latch for the drive");

    out = controller.step(enabled_input(cooldown_started_ms + 500, 300));
    require(out.state == TccDirectSlipState::RetryCooldown && out.pressure_mbar == 0,
        "controller should honor its bounded cooldown");
    out = controller.step(enabled_input(cooldown_started_ms + 1020, 300));
    require(out.state == TccDirectSlipState::Applying && out.pressure_mbar == 100,
        "controller should retry automatically after cooldown");
}

void test_invalid_speed_opens_without_latching() {
    TccDirectSlipController controller;
    auto invalid = enabled_input(0, -100);
    invalid.speed_valid = false;
    const auto out = controller.step(invalid);
    require(out.state == TccDirectSlipState::Open && out.pressure_mbar == 0,
        "invalid speed must fail open");
    require(out.reason == TccDirectSlipReason::InvalidSpeed && !out.fault_latched,
        "invalid speed must remain a non-latching inhibit");
}

void test_feedforward_is_clamped_to_validated_envelope() {
    TccDirectSlipController low;
    TccDirectSlipController high;
    TccDirectSlipOutput low_out = {};
    TccDirectSlipOutput high_out = {};
    for (uint32_t now = 0; now <= 500; now += 20) {
        low_out = low.step(enabled_input(now, 0, 0, 200));
        high_out = high.step(enabled_input(now, 0, 0, 6000));
    }
    require(low_out.pressure_mbar == 1500, "feed-forward must not fall below hold pressure");
    require(high_out.pressure_mbar == 2000, "feed-forward must stay below feedback headroom");
}
}

int main() {
    test_reaches_and_retains_holding_floor_at_zero_slip();
    test_positive_and_negative_slip_have_identical_control();
    test_torque_reversal_never_releases_a_tracking_clutch();
    test_v6_coast_fault_window_applies_instead_of_latching_open();
    test_feedback_waits_for_hydraulic_response();
    test_delayed_plants_track_in_both_torque_directions();
    test_shift_and_mismatch_open_then_reapply_immediately();
    test_unresponsive_plant_cools_down_and_retries();
    test_invalid_speed_opens_without_latching();
    test_feedforward_is_clamped_to_validated_envelope();
    std::cout << "TCC V7 stable-lock host tests passed\n";
    return 0;
}
