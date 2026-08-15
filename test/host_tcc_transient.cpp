#include "tcc_transient_controller.h"
#include <cassert>
#include <cstdlib>
#include <iostream>

static TccTransientInput in(bool apply, bool force_open, bool valid,
    int slip, int target, int base = 1344, uint32_t now = 0) {
    return {apply, force_open, valid, slip, target, base, now};
}

static void assert_shift_trace(uint8_t source_gear, uint8_t destination_gear) {
    TccShiftGuard guard;
    TccTransientController controller;
    uint32_t now = 0;
    uint32_t ratio_epoch = 0;

    controller.select_gear(source_gear);
    if (source_gear >= 2) {
        for (int i = 0; i < 4; ++i) {
            auto out = controller.step(in(true, false, true, 330, 50, 1344, now));
            assert(out.pressure > 0 && out.pressure <= TccTransientCalibration::kContactSearchPressure);
            now += TccTransientCalibration::kCycleMs;
        }
    }

    // Internal shift active: command must be exactly zero regardless of the
    // prior gear's controller pressure.
    auto reason = guard.step(now, true, true, false, 1000, ++ratio_epoch);
    assert(reason == TccTransientReason::ShiftInhibit);
    assert(controller.step(in(true, true, true, 300, 50, 1344, now)).pressure == 0);
    now += TccTransientCalibration::kCycleMs;

    // The local shift algorithm may have ended, but actual/target mismatch
    // continues to hold the converter open.
    reason = guard.step(now, false, true, true, 300, ++ratio_epoch);
    assert(reason == TccTransientReason::GearMismatch);
    assert(controller.step(in(true, true, true, 300, 50, 1344, now)).pressure == 0);
    now += TccTransientCalibration::kCycleMs;

    // Gear commit resets the common controller. A wrong-ratio sample after
    // the minimum time resets stability confirmation rather than releasing.
    controller.select_gear(destination_gear);
    const uint32_t commit_ms = now;
    reason = guard.step(now, false, false, true, 300, ++ratio_epoch);
    assert(reason == TccTransientReason::PostShiftSettling);
    assert(controller.step(in(true, true, true, 300, 50, 1344, now)).pressure == 0);
    while (now - commit_ms < (uint32_t)TccTransientCalibration::kPostShiftMinSettleMs) {
        now += TccTransientCalibration::kCycleMs;
        reason = guard.step(now, false, false, true, 300, ++ratio_epoch);
        assert(reason == TccTransientReason::PostShiftSettling);
        assert(controller.step(in(true, true, true, 300, 50, 1344, now)).pressure == 0);
    }

    // Five consecutive plausible ratio samples are required. The first four
    // remain hard-open; the fifth releases into a 100 mbar Fill step.
    for (int stable = 1; stable <= TccTransientCalibration::kPostShiftStableCycles; ++stable) {
        now += TccTransientCalibration::kCycleMs;
        reason = guard.step(now, false, false, true, 20, ++ratio_epoch);
        if (stable < TccTransientCalibration::kPostShiftStableCycles) {
            assert(reason == TccTransientReason::PostShiftSettling);
            assert(controller.step(in(true, true, true, 300, 50, 1344, now)).pressure == 0);
        } else {
            assert(reason == TccTransientReason::None);
            const auto reacquire = controller.step(in(true, false, true, 300, 50, 1344, now));
            assert(reacquire.state == TccTransientState::Fill);
            assert(reacquire.pressure == TccTransientCalibration::kApplySlewPerCycle);
            assert(reacquire.pressure <= TccTransientCalibration::kContactSearchPressure);
        }
    }
}

int main() {
    assert(tcc_transient_applies_to_gear(2, true));
    assert(!tcc_transient_applies_to_gear(1, true));
    assert(tcc_transient_applies_to_gear(3, true));
    assert(tcc_transient_applies_to_gear(4, true));
    assert(tcc_transient_applies_to_gear(5, true));
    assert(!tcc_transient_applies_to_gear(2, false));

    // With no observed shift lifecycle, normal same-gear control is released.
    TccShiftGuard guard;
    uint32_t ratio_epoch = 0;
    assert(guard.step(0x90000000U, false, false, true, 0, ++ratio_epoch) == TccTransientReason::None);
    guard.reset();

    // Settling is state-based, not a timer-only release. Missing, implausible,
    // or interrupted ratio samples hold zero indefinitely. Unsigned elapsed
    // arithmetic also remains correct through the millisecond counter wrap.
    const uint32_t near_wrap = UINT32_MAX - 40U;
    assert(guard.step(near_wrap, true, true, false, 1000, ++ratio_epoch) == TccTransientReason::ShiftInhibit);
    assert(guard.step(near_wrap + 20U, false, false, false, 0, ++ratio_epoch) == TccTransientReason::PostShiftSettling);
    uint32_t wrapped_now = near_wrap + 20U;
    for (int i = 0; i < 20; ++i) {
        wrapped_now += TccTransientCalibration::kCycleMs;
        assert(guard.step(wrapped_now, false, false, false, 0, ++ratio_epoch) == TccTransientReason::PostShiftSettling);
    }
    for (int i = 0; i < 3; ++i) {
        wrapped_now += TccTransientCalibration::kCycleMs;
        assert(guard.step(wrapped_now, false, false, true, 80, ++ratio_epoch) == TccTransientReason::PostShiftSettling);
    }
    // Re-reading the same source epoch cannot finish confirmation even when
    // the cached value remains plausible.
    const uint32_t frozen_epoch = ratio_epoch;
    for (int i = 0; i < 10; ++i) {
        wrapped_now += TccTransientCalibration::kCycleMs;
        assert(guard.step(wrapped_now, false, false, true, 80, frozen_epoch) ==
            TccTransientReason::PostShiftSettling);
    }
    wrapped_now += TccTransientCalibration::kCycleMs;
    assert(guard.step(wrapped_now, false, false, true, 81, ++ratio_epoch) == TccTransientReason::PostShiftSettling);
    for (int i = 1; i <= TccTransientCalibration::kPostShiftStableCycles; ++i) {
        wrapped_now += TccTransientCalibration::kCycleMs;
        const auto settle_reason = guard.step(wrapped_now, false, false, true, 80, ++ratio_epoch);
        assert(settle_reason == (i == TccTransientCalibration::kPostShiftStableCycles
            ? TccTransientReason::None : TccTransientReason::PostShiftSettling));
    }
    guard.reset();

    // Exercise every forward shift destination controlled by V1, including
    // the formerly split D2->legacy-D3 handoff and all useful downshifts.
    const uint8_t traces[][2] = {
        {1, 2}, {2, 3}, {3, 4}, {4, 5},
        {5, 4}, {4, 3}, {3, 2},
    };
    for (const auto& trace : traces) {
        assert_shift_trace(trace[0], trace[1]);
    }

    TccTransientController c;

    // Selecting the same gear preserves a ramp; selecting another gear resets
    // it, preventing D2 pressure/contact state from leaking into D3.
    c.select_gear(2);
    assert(c.step(in(true, false, true, 331, 50, 1344, 0)).pressure == 100);
    c.select_gear(2);
    assert(c.step(in(true, false, true, 331, 50, 1344, 20)).pressure == 200);
    c.select_gear(3);
    assert(c.step(in(true, false, true, 331, 50, 1344, 40)).pressure == 100);
    c.reset();

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

    // Merely starting inside the target band is not evidence that commanded
    // pressure caused clutch contact. With no pressure-correlated slip drop,
    // the controller remains at the conservative Fill cap indefinitely.
    c.reset();
    auto band_glitch = c.step(in(true, false, true, 95, 89, 1344, 0));
    assert(band_glitch.state == TccTransientState::Fill && !band_glitch.contact_detected);
    for (int i = 1; i < 20; ++i) {
        auto band_only = c.step(in(true, false, true, 95, 89, 1344, i * 20));
        assert(band_only.state == TccTransientState::Fill);
        assert(!band_only.contact_detected);
        assert(band_only.pressure <= TccTransientCalibration::kContactSearchPressure);
    }

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
