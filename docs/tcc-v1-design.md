# TCC V1 Offline Candidate

## Scope and requirements

This candidate starts at installed commit `f9ea309473d3ec6ebc130c529eaf050162119137`.
It replaces legacy TCC application in every enabled D2-D5 gear with one bounded
controller and adds a gearbox-lifetime TCC inhibit followed by measured
post-shift ratio settling. D1 configuration semantics and its legacy path remain
unchanged. This is an offline engineering candidate, not a vehicle-flash
authorization.

## Architecture and invariants

`TccTransientController` has Open, Fill, SlipControl, Locked, and
ReleaseFault states. Fill rises by at most 100 mbar per 20 ms to a conservative
800 mbar contact-search cap, never directly to the learned per-gear map value;
the configured 10,000 mbar prefill is not used. SlipControl starts only after
measured slip falls at least 25 RPM from the fill-entry value for three
confirming cycles while pressure is being applied. Merely being inside the
target band is not contact evidence. The controller then walks a
target-slip trajectory in either direction by at most 10 RPM per cycle and
uses bounded proportional feedback without exceeding the existing learned-map
ceiling.

Any active shift, actual/target mismatch, invalid speed, hard-open request, or
post-shift settling period commands zero immediately. After gear commit, the
guard requires at least 100 ms plus five consecutive fresh turbine/output-ratio
samples within 80 RPM of the destination ratio. A monotonic epoch from the
underlying rear-wheel CAN frame means repeated reads of one cached frame do not
count as new samples. There is no timer-only release; an unstable or missing
ratio remains fail-open. A normal map-driven release is slew limited. Slip is
always treated as a magnitude. The controller has no integral term and
controlled D2-D5 cycles never enter the legacy adaptation-writing branch. The
existing diagnostic packet is unchanged;
`get_transient_snapshot()` is an internal trace seam for a later safe protocol
extension.

The gearbox now requires engine RPM no older than 100 ms instead of treating
its prior 1,000 ms cache allowance as live TCC feedback. Invalid physical shaft
data bypasses `update()` and clears apply pressure/contact state, but preserves
and advances the shift guard so a one-tick dropout cannot erase post-shift
settling.
Park/Neutral/Reverse, engine stopped or stalled, slave mode,
diagnostic TCC disable, and map-initialization failure fully reset state.
Diagnostic-control takeover also latches a pressure-manager inhibit and writes
zero TCC duty immediately, so a later pressure update cannot restore an old
hardware command. Selecting a different controlled gear resets pressure,
contact, and trajectory state, so every D2-D5 reacquisition starts at the first
100 mbar step rather than inheriting the source gear's state.

## State transitions

Open -> Fill occurs only when the map requests at most 90 RPM slip. Once an
application is active it remains eligible until the map exceeds 110 RPM,
providing Open/Slipping hysteresis. Fill -> SlipControl requires the measured
contact evidence above. SlipControl -> Locked occurs within the lock band. Any
shift/mismatch, invalid speed, hard-open request, or ratio-settling period enters
Open or ReleaseFault at zero command. One shared `TccShiftGuard` owns the
complete gearbox lifecycle and stable-ratio release for every gear; the common
D2-D5 controller does not duplicate that state.

## Calibration table

| Constant | Value | Units / bounds | Rationale |
|---|---:|---|---|
| `kPostShiftMinSettleMs` | 100 | ms minimum | Rejects immediate release on the first apparently committed sample |
| `kPostShiftStableCycles` | 5 | consecutive new source samples | Requires five plausible measurements; cached rereads do not count |
| `kPostShiftRatioErrorRpm` | 80 | turbine RPM maximum error | Directly checks turbine/output synchronization to the committed gear |
| `kApplySlewPerCycle` | 100 | mbar / 20 ms | Reaches the logged 1,344 mbar ceiling in about 280 ms rather than one tick |
| `kContactSearchPressure` | 800 | mbar maximum | Below the observed ~1,000 mbar breakaway region; no contact means no further rise |
| `kDemandReleaseSlewPerCycle` | 400 | mbar / 20 ms | Normal map release only; safety inhibits are immediate zero |
| `kFeedbackGain` | 4 | mbar / RPM | Small proportional correction, no integral windup |
| `kFeedbackLimit` | 600 | mbar | Bounds feedback contribution |
| apply/open hysteresis | 90 / 110 | target RPM | Avoids map-boundary chatter |
| `kContactSlipDropRpm` | 25 | RPM | Requires observable shaft response before trajectory control |
| `kContactConfirmCycles` | 3 | 20 ms cycles | Rejects one-sample slip noise as contact |
| `kTargetSlipSlewPerCycle` | 10 | RPM / 20 ms | A 300->100 RPM synchronization takes at least 400 ms |

These values are conservative starting limits selected for deterministic
offline testing, not claimed final vehicle calibration.

## Test mapping

`test/host_tcc_transient.cpp` covers the August 15 331->285 RPM fixture shape,
target hysteresis, magnitude handling, measured-contact transition, target
trajectory and undershoot feedback, a pressured shift interruption, early
callback end with actual/target mismatch, post-shift ratio settling, invalid,
frozen, and unstable ratio samples, invalid speed, hard-open behavior,
no-contact pressure cap, target-band-without-contact rejection,
negative-slip contact, state reset and garage/stall re-entry, map-pressure
ceiling, and slew. Scripted traces cover 1->2, 2->3, 3->4, 4->5, 5->4, 4->3,
and 3->2. They prove source-gear pressure goes immediately to zero, gear
mismatch remains inhibited, five stable destination-ratio samples are required,
and destination reacquisition starts at 100 mbar without a legacy 10,000 mbar
fallthrough. The full firmware build checks the real integration signatures.

Run the mandatory host gate with `sh scripts/test_tcc_transient.sh` before the
full `unified` PlatformIO build. This test is intentionally host-native so the
pure controller and shift guard run without ESP32 hardware.

## Integration, rollback, and risks

The unchanged KWP diagnostics packet is intentional for V1. D2-D5 adaptation is
intentionally frozen for the complete first A/B candidate: the preserved
learned per-gear maps are read-only feed-forward ceilings. This isolates the
new controller and prevents a short validation drive from changing its own
baseline. Restoring adaptation is a separate post-validation change, not part
of this initial flash gate. The internal snapshot exposes state, slip, map
target, trajectory target, feed-forward,
feedback, final pressure, contact, and reason without breaking wire
compatibility. Rollback is the archived official `f9ea309` image and preserved
settings/maps. Remaining risks are unknown hydraulic response to the
provisional pressure law, provisional ratio/contact thresholds, and possible
interaction between the common path and temperature scaling. If a bounded command still
causes a shaft snap, restore `f9ea309` and inspect TCC hydraulics,
damper/regulator hydraulics, solenoid, converter, and driveline causes rather
than increasing pressure constants.
