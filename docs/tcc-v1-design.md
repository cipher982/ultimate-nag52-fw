# TCC V1 Offline Candidate

## Scope and requirements

This candidate starts at installed commit `f9ea309473d3ec6ebc130c529eaf050162119137`.
It changes stable D2 Open-to-apply behavior and adds a gearbox-lifetime TCC
inhibit with post-shift dwell. D1 configuration semantics remain unchanged;
D3-D5 stable map behavior remains unchanged except for the shared safety
interlock. This is an offline engineering prototype, not a vehicle-flash
authorization.

## Architecture and invariants

`TccTransientController` has Open, Fill, SlipControl, Locked, and
ReleaseFault states. Fill uses at most one quarter of the configured prefill
pressure, then SlipControl combines the existing map pressure with bounded
proportional slip feedback; every pressure increase/decrease is slew limited.
Any active shift, actual/target mismatch, invalid speed, or post-shift dwell
forces Open/release. The controller has no integral term and transient cycles
never write adaptation maps. The existing diagnostic packet is unchanged;
`get_transient_snapshot()` is an internal trace seam for a later safe protocol
extension.

Current `SensorData` carries no source timestamp or age. Integration therefore
passes `speed_fresh=true` and can only reject zero/`UINT16_MAX` RPM values;
the host seam supports stale-data rejection and the missing firmware age signal
is an explicit follow-up limitation.

## State transitions

Open -> Fill occurs only on an apply request after hysteresis. Fill ->
SlipControl occurs at bounded fill pressure or near target slip. SlipControl ->
Locked occurs within the lock band. Any open request, shift/mismatch, invalid
or stale speed, or dwell enters Open/ReleaseFault and pressure returns toward
zero. A successful shift starts a 300 ms post-shift dwell before reapplication.

## Calibration table

| Constant | Value | Units / bounds | Rationale |
|---|---:|---|---|
| `kPostShiftDwellMs` | 300 | ms, >= 0 | Covers the measured early reapply window; provisional |
| `kFillPressure` | 2500 | mbar, <= ceiling | Quarter of installed 10,000 mbar burst; not vehicle validated |
| `kPressureCeiling` | 6000 | mbar, positive | Conservative prototype ceiling below the configured burst |
| `kApplySlewPerCycle` | 300 | mbar / 20 ms | Prevents a discontinuous bite |
| `kReleaseSlewPerCycle` | 800 | mbar / 20 ms | Faster fail-open release while still bounded |
| `kFeedbackGain` | 4 | mbar / RPM | Small proportional correction, no integral windup |
| `kFeedbackLimit` | 600 | mbar | Bounds feedback contribution |
| hysteresis | 90 / 110 | RPM | Avoids target-boundary chatter |

These values are starting limits selected for deterministic offline testing,
not claimed vehicle calibration.

## Test mapping

`test/host_tcc_transient.cpp` covers delayed D2 apply and the August 15
331->285 RPM fixture shape, noisy threshold behavior, shift start during apply,
early callback end with actual/target mismatch, post-shift dwell, invalid and
stale speed, pressure ceiling and slew. The source-level integration keeps D3-D5
on the existing path; the full firmware build is the regression check for that
unchanged code path.

## Integration, rollback, and risks

The unchanged KWP diagnostics packet is intentional for V1. The internal
snapshot exposes state, slip, target, feedforward, feedback, final pressure,
and reason without breaking wire compatibility. Rollback is the archived
official `f9ea309` image and preserved settings/maps. Remaining risks are
unknown hydraulic response to the provisional pressure law, lack of source-age
data in the firmware sensor architecture, and possible interaction between
the new D2 path and learned maps or temperature scaling. If a gentle offline
or later controlled command still causes a shaft snap, inspect TCC hydraulic,
damper, solenoid, and driveline causes rather than increasing these constants.
