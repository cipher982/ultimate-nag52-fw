#include "tcc_transient_controller.h"
#include <cassert>
#include <cstdlib>
#include <iostream>

static TccTransientInput in(bool apply, bool force_open, bool valid,
    int slip, int target, int base = 1344, uint32_t now = 0) {
    return {apply, force_open, valid, slip, target, base, now};
}

int main() {
    assert(tcc_transient_applies_to_gear(2, true));
    assert(!tcc_transient_applies_to_gear(1, true));
    assert(!tcc_transient_applies_to_gear(3, true));
    assert(!tcc_transient_applies_to_gear(4, true));
    assert(!tcc_transient_applies_to_gear(5, true));

    // One authoritative shift lifetime: active shift, actual/target mismatch,
    // then a 300 ms post-commit dwell. Initial deadline zero never inhibits.
    TccShiftGuard guard;
    assert(!guard.step(0x90000000U, false));
    assert(guard.step(0, true));
    assert(guard.step(20, true));
    assert(guard.step(40, false));
    assert(guard.step(320, false));
    assert(!guard.step(340, false));
    guard.reset();
    assert(!guard.step(0, false));

    TccTransientController c;

    // Open/apply target hysteresis: 95 RPM cannot start an application, but
    // once started the controller remains active until the map exceeds 110.
    auto held_open = c.step(in(true, false, true, 331, 95, 1344, 0));
    assert(held_open.state == TccTransientState::Open && held_open.pressure == 0);
    auto started = c.step(in(true, false, true, 331, 89, 1344, 20));
    assert(started.state == TccTransientState::Fill && started.pressure == 100);
    auto held_apply = c.step(in(false, false, true, 320, 105, 1344, 40));
    assert(held_apply.state == TccTransientState::Fill && held_apply.pressure == 200);
    auto opened = c.step(in(false, false, true, 320, 111, 1344, 60));
    assert(opened.state == TccTransientState::Open && opened.pressure < held_apply.pressure);

    // No contact means no command above the conservative 800 mbar search cap,
    // even when the learned map is the logged 1,344 mbar.
    c.reset();
    for (int i = 0; i < 30; ++i) {
        auto out = c.step(in(true, false, true, 331, 89, 1344, i * 20));
        assert(out.state == TccTransientState::Fill);
        assert(!out.contact_detected);
        assert(out.pressure <= TccTransientCalibration::kContactSearchPressure);
    }

    // August 15 stable-D2 fixture: the original one-sample 331->285 change is
    // not enough; contact requires three confirming samples, then a bounded
    // trajectory starts without a 10,000 mbar pulse.
    c.reset();
    auto first = c.step(in(true, false, true, 331, 89, 1344, 0));
    assert(first.state == TccTransientState::Fill && first.pressure == 100);
    auto drop1 = c.step(in(true, false, true, 285, 89, 1344, 20));
    auto drop2 = c.step(in(true, false, true, 280, 89, 1344, 40));
    auto contact = c.step(in(true, false, true, 275, 89, 1344, 60));
    assert(!drop1.contact_detected && !drop2.contact_detected);
    assert(contact.state == TccTransientState::SlipControl && contact.contact_detected);
    assert(contact.pressure == 400 && contact.trajectory_slip_rpm == 275);
    int previous_trajectory = contact.trajectory_slip_rpm;
    int previous_pressure = contact.pressure;
    bool saw_negative_feedback = false;
    for (int i = 4; i < 30; ++i) {
        // Deliberately cross below the trajectory to prove pressure backs off.
        const int observed_slip = i < 12 ? 275 - (i - 3) * 12 : 80;
        auto out = c.step(in(true, false, true, observed_slip, 89, 1344, i * 20));
        assert(out.pressure >= 0 && out.pressure <= 1344);
        assert(std::abs(out.pressure - previous_pressure) <= TccTransientCalibration::kApplySlewPerCycle);
        assert(std::abs(out.trajectory_slip_rpm - previous_trajectory) <= TccTransientCalibration::kTargetSlipSlewPerCycle);
        if (out.feedback_correction < 0) saw_negative_feedback = true;
        previous_trajectory = out.trajectory_slip_rpm;
        previous_pressure = out.pressure;
    }
    assert(saw_negative_feedback);

    // Being inside the target band is valid contact evidence, but one noisy
    // sample may not promote Fill to SlipControl. It needs the same three
    // confirming cycles as cumulative slip drop.
    c.reset();
    auto band_glitch = c.step(in(true, false, true, 95, 89, 1344, 0));
    assert(band_glitch.state == TccTransientState::Fill && !band_glitch.contact_detected);
    auto band_second = c.step(in(true, false, true, 94, 89, 1344, 20));
    assert(band_second.state == TccTransientState::Fill && !band_second.contact_detected);
    auto band_contact = c.step(in(true, false, true, 93, 89, 1344, 40));
    assert(band_contact.state == TccTransientState::SlipControl && band_contact.contact_detected);

    // Slip is a magnitude, including through contact detection.
    c.reset();
    auto negative_slip = c.step(in(true, false, true, -331, 89, 1344, 0));
    assert(negative_slip.slip_rpm == 331 && negative_slip.state == TccTransientState::Fill);
    c.step(in(true, false, true, -285, 89, 1344, 20));
    c.step(in(true, false, true, -280, 89, 1344, 40));
    assert(c.step(in(true, false, true, -275, 89, 1344, 60)).contact_detected);

    // A pressured application interrupted by the shared shift guard, an engine
    // hard-open request, or invalid speed commands exactly zero in one cycle.
    c.reset();
    c.step(in(true, false, true, 331, 89, 1344, 0));
    auto pressured = c.step(in(true, false, true, 300, 89, 1344, 20));
    assert(pressured.pressure > 0);
    assert(c.step(in(true, true, true, 300, 89, 1344, 40)).pressure == 0);
    c.reset();
    c.step(in(true, false, true, 331, 89, 1344, 0));
    assert(c.step(in(true, false, false, 300, 89, 1344, 20)).pressure == 0);

    // Reset models every gearbox bypass: Park/Neutral, invalid-speed, and
    // diagnostic and engine-stopped bypasses cannot restore the old D2
    // pressure on re-entry.
    c.reset();
    for (int i = 0; i < 8; ++i) c.step(in(true, false, true, 331, 50, 1344, i * 20));
    c.reset();
    auto garage_reentry = c.step(in(true, false, true, 331, 50, 1344, 200));
    assert(garage_reentry.state == TccTransientState::Fill);
    assert(garage_reentry.pressure == TccTransientCalibration::kApplySlewPerCycle);

    c.reset();
    auto invalid_pressure = c.step(in(true, false, true, 200, 50, 0, 0));
    assert(invalid_pressure.pressure == 0 && invalid_pressure.reason == TccTransientReason::InvalidPressure);

    std::cout << "host TCC transient tests passed\n";
}
