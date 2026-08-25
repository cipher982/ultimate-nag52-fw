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
    int signed_slip_rpm,
    int torque_direction = 1,
    int target_rpm = 40,
    int feedforward_mbar = 1500,
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

void test_fast_fill_then_bounded_feedback() {
    TccDirectSlipController controller;
    TccDirectSlipOutput out = {};
    for (uint32_t now = 0; now <= 280; now += 20) {
        out = controller.step(enabled_input(now, 300));
    }
    require(out.pressure_mbar == 1500,
        "V8 should cross the measured dead zone in 300 ms");
    require(out.state == TccDirectSlipState::Applying,
        "fill and settle must precede tracking");

    for (uint32_t now = 300; now < 480; now += 20) {
        out = controller.step(enabled_input(now, 300));
        require(out.pressure_mbar == 1500,
            "PI must wait for the hydraulic settle window");
    }
    out = controller.step(enabled_input(480, 300));
    require(out.pressure_delta_mbar == 25,
        "PI pressure rise must use the V8 regulation slew limit");
    require(out.proportional_pressure_mbar > 0,
        "positive excess slip must request more pressure");
}

void test_drive_and_coast_use_the_same_oriented_error() {
    TccDirectSlipController drive;
    TccDirectSlipController coast;
    for (uint32_t now = 0; now <= 1200; now += 20) {
        const auto drive_out = drive.step(enabled_input(now, 140, 1));
        const auto coast_out = coast.step(enabled_input(now, -140, -1));
        require(drive_out.oriented_slip_rpm == coast_out.oriented_slip_rpm,
            "drive and coast must orient physically equivalent slip alike");
        require(drive_out.slip_error_rpm == coast_out.slip_error_rpm,
            "drive and coast must produce the same control error");
        require(drive_out.pressure_mbar == coast_out.pressure_mbar,
            "equivalent signed-slip cases must produce equal pressure");
    }
}

void test_delayed_drive_and_coast_plants_track() {
    for (const int direction : {1, -1}) {
        TccDirectSlipController controller;
        TccDirectSlipOutput out = {};
        double hydraulic_pressure_mbar = 0;
        double slip_magnitude_rpm = 300;
        uint32_t now = 0;
        for (; now < 5000 && !out.tracking_achieved; now += 20) {
            out = controller.step(enabled_input(
                now,
                direction * static_cast<int>(slip_magnitude_rpm),
                direction
            ));
            hydraulic_pressure_mbar +=
                (out.pressure_mbar - hydraulic_pressure_mbar) / 5.0;
            const double coupling_rpm = hydraulic_pressure_mbar > 1350
                ? hydraulic_pressure_mbar - 1350 : 0;
            const double equilibrium_slip_rpm = 300 - coupling_rpm > 0
                ? 300 - coupling_rpm : 0;
            slip_magnitude_rpm +=
                (equilibrium_slip_rpm - slip_magnitude_rpm) / 4.0;
        }
        require(out.tracking_achieved,
            "a reachable delayed plant must follow the V8 slip trajectory");
        require(now < 5000,
            "a reachable delayed plant must track before the fault window");
        require(!out.fault_latched,
            "normal delayed hydraulic response must not latch a fault");
    }
}

void test_negative_error_relaxes_pressure() {
    TccDirectSlipController controller;
    TccDirectSlipOutput out = {};
    for (uint32_t now = 0; now <= 1200; now += 20) {
        out = controller.step(enabled_input(now, 400));
    }
    const int high_pressure_mbar = out.pressure_mbar;
    require(high_pressure_mbar > 1500,
        "precondition: excess slip should raise pressure above feed-forward");

    out = controller.step(enabled_input(1220, 0));
    require(out.proportional_pressure_mbar < 0,
        "slip below the trajectory must produce negative proportional correction");
    require(out.pressure_mbar < high_pressure_mbar,
        "V8 must be able to remove pressure as well as add it");
    require(out.pressure_delta_mbar >= -50,
        "pressure relief must respect its per-cycle slew limit");
}

void test_shift_holds_pressure_and_raises_reference() {
    TccDirectSlipController controller;
    TccDirectSlipOutput out = {};
    for (uint32_t now = 0; now <= 1000; now += 20) {
        out = controller.step(enabled_input(now, 40));
    }
    const int pressure_before_shift = out.pressure_mbar;
    const int target_before_shift = out.target_slip_rpm;
    require(pressure_before_shift > 0, "precondition: clutch must be applied");

    for (uint32_t now = 1020; now <= 1320; now += 20) {
        out = controller.step(enabled_input(now, 500, 1, 40, 1500, true));
        require(out.state == TccDirectSlipState::ShiftHold,
            "an active shift must report shift-hold state");
        require(out.pressure_mbar == pressure_before_shift,
            "an active shift must not dump or increase TCC pressure");
        require(out.pressure_delta_mbar == 0,
            "held shift pressure must have zero command step");
        require(out.shift_hold, "shift hold must be explicit in telemetry");
    }
    require(out.target_slip_rpm > target_before_shift,
        "the shift must relax the slip trajectory instead of opening the clutch");
    require(out.target_slip_rpm <= TccDirectSlipCalibration::kShiftTargetRpm,
        "the relaxed shift target must remain bounded");

    out = controller.step(enabled_input(1340, 500));
    require(out.pressure_mbar >= pressure_before_shift,
        "post-shift control must continue from the held pressure");
    require(out.pressure_delta_mbar <= 25,
        "post-shift pressure must resume without a step change");
    require(out.target_slip_rpm < TccDirectSlipCalibration::kShiftTargetRpm,
        "post-shift trajectory must immediately begin returning to normal");
}

void test_torque_reversal_holds_before_changing_pressure() {
    TccDirectSlipController controller;
    TccDirectSlipOutput out = {};
    for (uint32_t now = 0; now <= 900; now += 20) {
        out = controller.step(enabled_input(now, 120, 1));
    }
    const int pressure_before_reversal = out.pressure_mbar;

    for (uint32_t now = 920; now < 1120; now += 20) {
        out = controller.step(enabled_input(now, -120, -1));
        require(out.pressure_mbar == pressure_before_reversal,
            "torque reversal must hold pressure during its settle window");
        require(out.torque_direction == -1,
            "telemetry must expose the new coast direction");
    }
    out = controller.step(enabled_input(1120, -120, -1));
    require(out.pressure_delta_mbar <= 25 && out.pressure_delta_mbar >= -50,
        "control after torque reversal must resume through normal slew limits");
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
    require(fault_time > 0, "an unresponsive plant must eventually fault");
    require(out.state == TccDirectSlipState::FaultOpen,
        "qualified nontracking must latch open");
    require(out.pressure_mbar == 0, "fault-open must command zero pressure");

    out = controller.step(enabled_input(fault_time + 5000, 1000));
    require(out.state == TccDirectSlipState::FaultOpen && out.pressure_mbar == 0,
        "V8 must never retry a thermal fault automatically");
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
        "a derated cold command must not be diagnosed as a full-pressure failure");
    require(out.pressure_mbar == TccDirectSlipCalibration::kMaxCommandPressureMbar,
        "cold fault suppression must not change the bounded controller command");
}

void test_indeterminate_torque_freezes_feedback() {
    TccDirectSlipController controller;
    TccDirectSlipOutput out = {};
    for (uint32_t now = 0; now <= 900; now += 20) {
        out = controller.step(enabled_input(now, 200, 1));
    }
    const int pressure_before_deadband = out.pressure_mbar;
    for (uint32_t now = 920; now <= 1100; now += 20) {
        out = controller.step(enabled_input(now, -300, 0));
        require(out.pressure_mbar == pressure_before_deadband,
            "uncertain torque direction must freeze feedback pressure");
    }
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
        "feed-forward must not fall below the measured same-truck floor");
    require(high_out.feedforward_pressure_mbar == 2000,
        "feed-forward must preserve feedback headroom");
    require(low_out.target_slip_rpm >= 20,
        "normal target must not request a hard zero-slip lock");
    require(high_out.target_slip_rpm <= 80,
        "normal target must stay inside the V8 envelope");
}
}

int main() {
    test_fast_fill_then_bounded_feedback();
    test_drive_and_coast_use_the_same_oriented_error();
    test_delayed_drive_and_coast_plants_track();
    test_negative_error_relaxes_pressure();
    test_shift_holds_pressure_and_raises_reference();
    test_torque_reversal_holds_before_changing_pressure();
    test_unresponsive_plant_faults_once_without_retry();
    test_cold_pressure_derate_cannot_trigger_full_pressure_fault();
    test_indeterminate_torque_freezes_feedback();
    test_invalid_speed_opens_without_fault();
    test_feedforward_and_target_are_clamped();
    std::cout << "TCC V8 host tests passed\n";
    return 0;
}
