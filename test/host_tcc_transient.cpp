#include "tcc_transient_controller.h"
#include <cassert>
#include <cstdlib>
#include <iostream>

static TccTransientInput in(bool apply, bool force_open, bool shifting, bool match, bool valid,
    int slip, int target, int base = 1344, uint32_t now = 0) {
    return {apply, force_open, shifting, match, valid, slip, target, base, now};
}

int main() {
    assert(tcc_transient_applies_to_gear(2, true));
    assert(!tcc_transient_applies_to_gear(1, true));
    assert(!tcc_transient_applies_to_gear(3, true));
    assert(!tcc_transient_applies_to_gear(4, true));
    assert(!tcc_transient_applies_to_gear(5, true));

    TccTransientController c;

    // Open/apply target hysteresis: 95 RPM cannot start an application, but
    // once started the controller remains active until the map exceeds 110.
    auto held_open = c.step(in(true, false, false, true, true, 331, 95, 1344, 0));
    assert(held_open.state == TccTransientState::Open && held_open.pressure == 0);
    auto started = c.step(in(true, false, false, true, true, 331, 89, 1344, 20));
    assert(started.state == TccTransientState::Fill && started.pressure == 100);
    auto held_apply = c.step(in(false, false, false, true, true, 320, 105, 1344, 40));
    assert(held_apply.state == TccTransientState::Fill && held_apply.pressure == 200);
    auto opened = c.step(in(false, false, false, true, true, 320, 111, 1344, 60));
    assert(opened.state == TccTransientState::Open && opened.pressure < held_apply.pressure);

    // August 15 stable-D2 fixture: no 10,000 mbar pulse, command is bounded by
    // 100 mbar/cycle and by the existing 1,344 mbar learned map value.
    c.reset();
    auto first = c.step(in(true, false, false, true, true, 331, 89, 1344, 0));
    assert(first.state == TccTransientState::Fill && first.pressure == 100);
    auto contact = c.step(in(true, false, false, true, true, 285, 89, 1344, 20));
    assert(contact.state == TccTransientState::SlipControl);
    assert(contact.contact_detected && contact.pressure == 200);
    assert(contact.trajectory_slip_rpm == 285);
    int previous_trajectory = contact.trajectory_slip_rpm;
    int previous_pressure = contact.pressure;
    for (int i = 2; i < 30; ++i) {
        const int observed_slip = 285 - (i - 1) * 8;
        auto out = c.step(in(true, false, false, true, true, observed_slip, 89, 1344, i * 20));
        assert(out.pressure >= 0 && out.pressure <= 1344);
        assert(std::abs(out.pressure - previous_pressure) <= TccTransientCalibration::kApplySlewPerCycle);
        assert(out.trajectory_slip_rpm <= previous_trajectory);
        assert(previous_trajectory - out.trajectory_slip_rpm <= TccTransientCalibration::kTargetSlipSlewPerCycle);
        previous_trajectory = out.trajectory_slip_rpm;
        previous_pressure = out.pressure;
    }

    // Slip is a magnitude: negative signed slip cannot masquerade as contact
    // or an already-locked converter.
    c.reset();
    auto negative_slip = c.step(in(true, false, false, true, true, -331, 89, 1344, 0));
    assert(negative_slip.slip_rpm == 331);
    assert(negative_slip.state == TccTransientState::Fill);

    // A pressured application interrupted by either shift signal or actual /
    // target mismatch commands exactly zero in the same cycle.
    c.reset();
    c.step(in(true, false, false, true, true, 331, 89, 1344, 0));
    auto pressured = c.step(in(true, false, false, true, true, 280, 89, 1344, 20));
    assert(pressured.pressure > 0);
    auto shift = c.step(in(true, false, true, true, true, 250, 89, 1344, 40));
    assert(shift.pressure == 0 && shift.reason == TccTransientReason::ShiftInhibit);
    auto mismatch = c.step(in(true, false, false, false, true, 250, 89, 1344, 60));
    assert(mismatch.pressure == 0 && mismatch.reason == TccTransientReason::GearMismatch);
    auto dwell = c.step(in(true, false, false, true, true, 250, 89, 1344, 80));
    assert(dwell.pressure == 0 && dwell.reason == TccTransientReason::PostShiftDwell);
    auto after_dwell = c.step(in(true, false, false, true, true, 250, 89, 1344, 400));
    assert(after_dwell.state == TccTransientState::Fill && after_dwell.pressure == 100);

    // Invalid speed and hard-open requests are fail-open. Invalid map pressure
    // also cannot energize the clutch.
    auto invalid = c.step(in(true, false, false, true, false, 200, 50, 1344, 420));
    assert(invalid.pressure == 0 && invalid.reason == TccTransientReason::InvalidSpeed);
    c.reset();
    c.step(in(true, false, false, true, true, 300, 50, 1344, 0));
    auto before_forced = c.step(in(true, false, false, true, true, 260, 50, 1344, 20));
    assert(before_forced.pressure > 0);
    auto forced = c.step(in(true, true, false, true, true, 200, 50, 1344, 440));
    assert(forced.pressure == 0 && forced.reason == TccTransientReason::DemandOpen);
    c.reset();
    auto invalid_pressure = c.step(in(true, false, false, true, true, 200, 50, 0, 0));
    assert(invalid_pressure.pressure == 0 && invalid_pressure.reason == TccTransientReason::InvalidPressure);

    // No command can exceed the learned feed-forward map value, even with a
    // very large slip error, and every normal increase is slew bounded.
    c.reset();
    int previous = 0;
    for (int i = 0; i < 40; ++i) {
        auto out = c.step(in(true, false, false, true, true, i == 0 ? 1000 : 900, 0, 900, i * 20));
        assert(out.pressure <= 900);
        assert(out.pressure - previous <= TccTransientCalibration::kApplySlewPerCycle);
        previous = out.pressure;
    }

    std::cout << "host TCC transient tests passed\n";
}
