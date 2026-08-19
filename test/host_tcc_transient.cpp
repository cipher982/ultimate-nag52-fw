#include "tcc_transient_controller.h"
#include <cassert>
#include <cstdlib>
#include <iostream>

static TccTransientInput in(bool apply, bool force_open, bool valid,
    int slip, int target, int base = 1344, uint32_t now = 0,
    bool coast_mode = false) {
    return {apply, force_open, valid, slip, target, base, now, coast_mode};
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
    // remain hard-open; the fifth releases into one bounded Fill step.
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

    // Coast hysteresis is expressed in normalized pedal percent and requires
    // negative torque. Raw pedal counts are converted before this helper.
    bool coast_mode = false;
    assert(tcc_transient_pedal_percent(20) == 8);
    coast_mode = tcc_transient_update_coast_mode(coast_mode, 8, true);
    assert(coast_mode);
    coast_mode = tcc_transient_update_coast_mode(coast_mode, 9, true);
    assert(coast_mode);
    coast_mode = tcc_transient_update_coast_mode(coast_mode, 14, true);
    assert(coast_mode);
    coast_mode = tcc_transient_update_coast_mode(coast_mode, 15, true);
    assert(!coast_mode);
    coast_mode = tcc_transient_update_coast_mode(coast_mode, 7, false);
    assert(!coast_mode);

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
    assert(c.step(in(true, false, true, 331, 50, 1344, 0)).pressure ==
        TccTransientCalibration::kApplySlewPerCycle);
    c.select_gear(2);
    assert(c.step(in(true, false, true, 331, 50, 1344, 20)).pressure ==
        2 * TccTransientCalibration::kApplySlewPerCycle);
    c.select_gear(3);
    assert(c.step(in(true, false, true, 331, 50, 1344, 40)).pressure ==
        TccTransientCalibration::kApplySlewPerCycle);
    c.reset();

    // Open/apply target hysteresis: 95 RPM cannot start an application, but
    // once started the controller remains active until the map exceeds 110.
    auto held_open = c.step(in(true, false, true, 331, 95, 1344, 0));
    assert(held_open.state == TccTransientState::Open && held_open.pressure == 0);
    auto started = c.step(in(true, false, true, 331, 89, 1344, 20));
    assert(started.state == TccTransientState::Fill &&
        started.pressure == TccTransientCalibration::kApplySlewPerCycle);
    auto held_apply = c.step(in(false, false, true, 320, 105, 1344, 40));
    assert(held_apply.state == TccTransientState::Fill &&
        held_apply.pressure == 2 * TccTransientCalibration::kApplySlewPerCycle);
    auto opened = c.step(in(false, false, true, 320, 111, 1344, 60));
    assert(opened.state == TccTransientState::Open && opened.pressure < held_apply.pressure);

    // Without rate-confirmed contact, search holds at the provisional 800 mbar
    // ceiling rather than creeping toward an unvalidated learned-map pressure.
    c.reset();
    for (int i = 0; i < 150; ++i) {
        auto out = c.step(in(true, false, true, 331, 89, 1344, i * 20));
        assert(out.state == TccTransientState::Fill);
        assert(!out.contact_detected);
        assert(out.pressure <= TccTransientCalibration::kContactSearchPressure);
    }
    auto contact_timeout = c.step(in(true, false, true, 331, 89, 1344, 3000));
    assert(contact_timeout.state == TccTransientState::ReleaseFault);
    assert(contact_timeout.reason == TccTransientReason::ContactNotDetected);
    assert(contact_timeout.pressure == 0);
    assert(c.step(in(true, false, true, 331, 89, 1344, 3020)).pressure == 0);
    auto retry = c.step(in(true, false, true, 331, 89, 1344,
        TccTransientCalibration::kFaultRetryCooldownMs + 3000));
    assert(retry.state == TccTransientState::Fill);
    assert(retry.pressure == TccTransientCalibration::kApplySlewPerCycle);

    // Signed slip is preserved for the emergency guard. Equal magnitudes on
    // opposite sides of zero must still expose a 40 RPM pull-through.
    c.reset();
    auto positive_slip = c.step(in(true, false, true, 20, 89, 1344, 0));
    assert(positive_slip.state == TccTransientState::SlipControl);
    auto zero_crossing = c.step(in(true, false, true, -20, 89, 1344, 20));
    assert(zero_crossing.reason != TccTransientReason::ExcessiveSlipRate);
    assert(zero_crossing.slip_delta_rpm == -40);
    auto second_zero_crossing = c.step(in(true, false, true, -60, 89, 1344, 40));
    assert(second_zero_crossing.state == TccTransientState::ReleaseFault);
    assert(second_zero_crossing.reason == TccTransientReason::ExcessiveSlipRate);
    assert(second_zero_crossing.pressure == 0);

    // The provisional boundary is strict: 25 RPM is accepted, 26 RPM aborts.
    c.reset();
    c.step(in(true, false, true, 100, 89, 1344, 0));
    auto at_boundary = c.step(in(true, false, true, 75, 89, 1344, 20));
    assert(at_boundary.reason != TccTransientReason::ExcessiveSlipRate);
    c.reset();
    c.step(in(true, false, true, 100, 89, 1344, 0));
    auto over_boundary = c.step(in(true, false, true, 74, 89, 1344, 20));
    assert(over_boundary.reason != TccTransientReason::ExcessiveSlipRate);
    auto over_boundary_confirmed = c.step(in(true, false, true, 48, 89, 1344, 40));
    assert(over_boundary_confirmed.reason == TccTransientReason::ExcessiveSlipRate);
    assert(over_boundary_confirmed.pressure == 0);

    // The August 15 coast event reached large negative slip while the TCC kept
    // applying. Once coast overrun exceeds 40 RPM, hold hard-open until throttle
    // returns, then reacquire from one bounded Fill step.
    c.reset();
    auto coast_boundary = c.step(in(true, false, true, -40, 10, 1344, 0, true));
    assert(coast_boundary.reason != TccTransientReason::CoastOverrun);
    auto coast_open = c.step(in(true, false, true, -41, 10, 1344, 20, true));
    assert(coast_open.state == TccTransientState::Open);
    assert(coast_open.reason == TccTransientReason::CoastOverrun);
    assert(coast_open.pressure == 0);
    auto coast_latched = c.step(in(true, false, true, -10, 10, 1344, 40, true));
    assert(coast_latched.state == TccTransientState::Open);
    assert(coast_latched.reason == TccTransientReason::CoastOverrun);
    auto stale_while_latched = c.step(in(true, false, false, -10, 10, 1344, 60, true));
    assert(stale_while_latched.state == TccTransientState::ReleaseFault);
    assert(stale_while_latched.reason == TccTransientReason::InvalidSpeed);
    auto coast_resumed = c.step(in(true, false, true, -10, 10, 1344, 80, true));
    assert(coast_resumed.state == TccTransientState::Open);
    assert(coast_resumed.reason == TccTransientReason::CoastOverrun);
    auto tip_in_reacquire = c.step(in(true, false, true, 80, 50, 1344, 100, false));
    assert(tip_in_reacquire.state == TccTransientState::Fill);
    assert(tip_in_reacquire.pressure == TccTransientCalibration::kApplySlewPerCycle);

    // A stale shaft sample fails open as invalid data; it cannot accidentally
    // arm or clear the coast-overrun latch.
    c.reset();
    auto stale_coast = c.step(in(true, false, false, -100, 10, 1344, 0, true));
    assert(stale_coast.state == TccTransientState::ReleaseFault);
    assert(stale_coast.reason == TccTransientReason::InvalidSpeed);
    assert(stale_coast.pressure == 0);

    // Persisted map data is untrusted. Even a 10,000 mbar cell cannot command
    // above the independent V1 ceiling after contact.
    c.reset();
    c.step(in(true, false, true, 331, 89, 10000, 0));
    c.step(in(true, false, true, 316, 89, 10000, 20));
    c.step(in(true, false, true, 301, 89, 10000, 40));
    auto capped_contact = c.step(in(true, false, true, 286, 89, 10000, 60));
    assert(capped_contact.contact_detected);
    assert(capped_contact.feedforward_pressure == TccTransientCalibration::kMaxCommandPressure);
    bool reached_hard_cap = false;
    for (int i = 4; i < 80; ++i) {
        auto out = c.step(in(true, false, true, 300, 89, 10000, i * 20));
        assert(out.pressure <= TccTransientCalibration::kMaxCommandPressure);
        if (out.pressure == TccTransientCalibration::kMaxCommandPressure) {
            reached_hard_cap = true;
        }
    }
    assert(reached_hard_cap);

    // The August 15 logger observed 331->285 across a roughly 100 ms sample.
    // Interpolate that shape into safe 20 ms controller steps; contact still
    // requires three confirming samples before the bounded trajectory starts.
    c.reset();
    auto first = c.step(in(true, false, true, 331, 89, 1344, 0));
    assert(first.state == TccTransientState::Fill &&
        first.pressure == TccTransientCalibration::kApplySlewPerCycle);
    auto drop1 = c.step(in(true, false, true, 316, 89, 1344, 20));
    auto drop2 = c.step(in(true, false, true, 301, 89, 1344, 40));
    auto contact = c.step(in(true, false, true, 286, 89, 1344, 60));
    assert(!drop1.contact_detected && !drop2.contact_detected);
    assert(contact.state == TccTransientState::SlipControl && contact.contact_detected);
    assert(contact.pressure == 4 * TccTransientCalibration::kApplySlewPerCycle &&
        contact.trajectory_slip_rpm == 286);
    int previous_trajectory = contact.trajectory_slip_rpm;
    int previous_pressure = contact.pressure;
    bool saw_negative_feedback = false;
    for (int i = 4; i < 30; ++i) {
        // Deliberately cross below the trajectory to prove pressure backs off.
        const int observed_slip = std::max(80, 286 - (i - 3) * 12);
        auto out = c.step(in(true, false, true, observed_slip, 89, 1344, i * 20));
        assert(out.pressure >= 0 && out.pressure <= 1344);
        const int pressure_delta = out.pressure - previous_pressure;
        assert(pressure_delta <= TccTransientCalibration::kApplySlewPerCycle);
        assert(pressure_delta >= -TccTransientCalibration::kFeedbackReliefSlewPerCycle);
        assert(std::abs(out.trajectory_slip_rpm - previous_trajectory) <= TccTransientCalibration::kTargetSlipSlewPerCycle);
        if (out.feedback_correction < 0) saw_negative_feedback = true;
        previous_trajectory = out.trajectory_slip_rpm;
        previous_pressure = out.pressure;
    }
    assert(saw_negative_feedback);

    // Merely starting inside the target band is not evidence that commanded
    // pressure caused clutch contact. With no rate-confirmed response, the
    // controller remains in bounded Fill and ultimately times out fail-open.
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
    c.step(in(true, false, true, -316, 89, 1344, 20));
    c.step(in(true, false, true, -301, 89, 1344, 40));
    assert(c.step(in(true, false, true, -286, 89, 1344, 60)).contact_detected);

    // A pressured application interrupted by the shared shift guard, an engine
    // hard-open request, or invalid speed commands exactly zero in one cycle.
    c.reset();
    c.step(in(true, false, true, 331, 89, 1344, 0));
    auto pressured = c.step(in(true, false, true, 306, 89, 1344, 20));
    assert(pressured.pressure > 0);
    assert(c.step(in(true, true, true, 306, 89, 1344, 40)).pressure == 0);
    c.reset();
    c.step(in(true, false, true, 331, 89, 1344, 0));
    assert(c.step(in(true, false, false, 306, 89, 1344, 20)).pressure == 0);

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

    // Low-slip Fill entry is valid contact evidence for steady cruise and
    // never enters the old three-second contact timeout path.
    c.reset();
    auto low_slip = c.step(in(true, false, true, 10, 10, 1100, 0));
    assert(low_slip.contact_detected && low_slip.state == TccTransientState::SlipControl);
    for (int cycle = 1; cycle < 500; ++cycle) {
        const auto cruise = c.step(in(true, false, true, 10 + (cycle % 6) * 2,
            10, 1100, cycle * 20));
        assert(cruise.reason != TccTransientReason::ContactNotDetected);
        assert(cruise.state != TccTransientState::ReleaseFault);
        if (cycle >= 35) {
            assert(cruise.state == TccTransientState::Locked);
        }
    }

    // PI pressure may use the explicit 400 mbar headroom and must settle at
    // the independent command ceiling rather than the map value.
    c.reset();
    c.step(in(true, false, true, 30, 10, 1100, 0));
    c.step(in(true, false, true, 25, 10, 1100, 20));
    c.step(in(true, false, true, 20, 10, 1100, 40));
    int maximum_headroom_pressure = 0;
    for (int cycle = 3; cycle < 180; ++cycle) {
        const auto disturbance = c.step(in(true, false, true, 40, 10, 1100, cycle * 20));
        maximum_headroom_pressure = std::max(maximum_headroom_pressure, disturbance.pressure);
        assert(disturbance.pressure <= 1500);
    }
    assert(maximum_headroom_pressure == 1500);

    // +/-5 RPM noise around the target is inside the integrator deadband.
    c.reset();
    auto deadband_start = c.step(in(true, false, true, 10, 10, 1100, 0));
    for (int cycle = 1; cycle < 180; ++cycle) {
        const auto noise = c.step(in(true, false, true, cycle % 2 ? 15 : 5,
            10, 1100, cycle * 20));
        assert(noise.integral_correction == deadband_start.integral_correction);
    }

    // Bumpless feedforward step: stepping feedforward modifies integrator
    // baseline without a sudden jump in feedback error.
    c.reset();
    c.step(in(true, false, true, 10, 10, 1000, 0));
    for (int i = 1; i < 40; ++i) c.step(in(true, false, true, 10, 10, 1000, i * 20));
    auto before_step = c.step(in(true, false, true, 10, 10, 1000, 800));
    auto after_step = c.step(in(true, false, true, 10, 10, 1200, 820));
    assert(std::abs(after_step.integral_correction - before_step.integral_correction) <= 50);

    // Retry limit: persistent fault retries up to 2 times, then latches open.
    c.reset();
    c.step(in(true, false, true, 200, 50, 1100, 0));
    c.step(in(true, false, true, 150, 50, 1100, 20));
    auto fault1 = c.step(in(true, false, true, 100, 50, 1100, 40));
    assert(fault1.state == TccTransientState::ReleaseFault && fault1.reason == TccTransientReason::ExcessiveSlipRate);

    auto cooldown1 = c.step(in(true, false, true, 100, 50, 1100, 2050));
    assert(cooldown1.state == TccTransientState::Fill);
    c.step(in(true, false, true, 150, 50, 1100, 2070));

    auto cooldown2 = c.step(in(true, false, true, 100, 50, 1100, 4100));
    assert(cooldown2.state == TccTransientState::Fill);
    c.step(in(true, false, true, 150, 50, 1100, 4120));
    c.step(in(true, false, true, 100, 50, 1100, 4140));

    auto latched = c.step(in(true, false, true, 100, 50, 1100, 7000));
    assert(latched.state == TccTransientState::ReleaseFault && latched.pressure == 0);

    auto cleared = c.step(in(false, false, true, 100, 50, 1100, 7020));
    assert(cleared.reason == TccTransientReason::DemandOpen);
    std::cout << "host TCC transient tests passed\n";
}
