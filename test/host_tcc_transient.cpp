#include "tcc_transient_controller.h"
#include <cassert>
#include <iostream>

static TccTransientInput in(bool apply, bool shifting, bool match, bool valid, bool fresh,
    int slip, int target, int base = 1400, uint32_t now = 0) {
    return {apply, shifting, match, valid, fresh, slip, target, base, 10000, now};
}

int main() {
    assert(tcc_transient_applies_to_gear(2, true));
    assert(!tcc_transient_applies_to_gear(1, true));
    assert(!tcc_transient_applies_to_gear(3, true));
    assert(!tcc_transient_applies_to_gear(4, true));
    assert(!tcc_transient_applies_to_gear(5, true));

    TccTransientController c;

    // August 15 stable-D2 shape: 331 RPM slip at the 1,146 RPM transition.
    auto first = c.step(in(true, false, true, true, true, 331, 95, 1344, 0));
    assert(first.state == TccTransientState::Fill && first.pressure <= 300);
    auto second = c.step(in(true, false, true, true, true, 285, 95, 1344, 20));
    assert(second.pressure > first.pressure && second.pressure <= 600);
    for (int i = 2; i < 20; ++i) c.step(in(true, false, true, true, true, 180, 95, 1344, i * 20));
    auto controlled = c.step(in(true, false, true, true, true, 120, 95, 1344, 400));
    assert(controlled.pressure <= 2500);

    // Hysteresis: noisy values around the open boundary do not repeatedly start fill.
    c.reset();
    c.step(in(false, false, true, true, true, 111, 100, 1400, 0));
    auto noise = c.step(in(false, false, true, true, true, 95, 100, 1400, 20));
    assert(noise.state == TccTransientState::Open);
    c.step(in(true, false, true, true, true, 95, 100, 1400, 40));
    assert(c.step(in(false, false, true, true, true, 95, 100, 1400, 60)).pressure == 0);

    // Shift callback ending early and actual != target both keep the TCC open.
    c.reset();
    c.step(in(true, true, true, true, true, 200, 50, 1400, 0));
    auto mismatch = c.step(in(true, false, false, true, true, 200, 50, 1400, 20));
    assert(mismatch.pressure == 0 && mismatch.reason == TccTransientReason::GearMismatch);
    auto dwell = c.step(in(true, false, true, true, true, 200, 50, 1400, 40));
    assert(dwell.pressure == 0 && dwell.reason == TccTransientReason::PostShiftDwell);
    auto after_dwell = c.step(in(true, false, true, true, true, 200, 50, 1400, 400));
    assert(after_dwell.state == TccTransientState::Fill);

    // Invalid and stale speed inputs fail open.
    auto invalid = c.step(in(true, false, true, false, true, 200, 50, 1400, 420));
    assert(invalid.pressure < after_dwell.pressure && invalid.reason == TccTransientReason::InvalidSpeed);
    auto stale = c.step(in(true, false, true, true, false, 200, 50, 1400, 440));
    assert(stale.reason == TccTransientReason::StaleSpeed);

    // Pressure ceiling and per-cycle slew are invariant.
    c.reset();
    int previous = 0;
    for (int i = 0; i < 40; ++i) {
        auto out = c.step(in(true, false, true, true, true, 1000, 0, 9000, i * 20));
        assert(out.pressure <= TccTransientCalibration::kPressureCeiling);
        assert(out.pressure - previous <= TccTransientCalibration::kApplySlewPerCycle);
        previous = out.pressure;
    }

    std::cout << "host TCC transient tests passed\n";
}
