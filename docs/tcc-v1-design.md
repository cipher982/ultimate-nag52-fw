# TCC V1 Offline Candidate

## Scope and requirements

This candidate starts at installed commit `f9ea309473d3ec6ebc130c529eaf050162119137`.
It changes stable D2 Open-to-apply behavior and adds a gearbox-lifetime TCC
inhibit with post-shift dwell. D1 configuration semantics remain unchanged;
D3-D5 stable map behavior remains unchanged except for the shared shift safety
interlock. This is an offline engineering candidate, not a vehicle-flash
authorization.

## Architecture and invariants

`TccTransientController` has Open, Fill, SlipControl, Locked, and
ReleaseFault states. Fill rises by at most 100 mbar per 20 ms to a conservative
800 mbar contact-search cap, never directly to the learned D2 map value; the
configured 10,000 mbar prefill is not used. SlipControl starts only after
measured slip falls at least 25 RPM from the fill-entry value for three
confirming cycles (or is already in the target band). It then walks a
target-slip trajectory in either direction by at most 10 RPM per cycle and
uses bounded proportional feedback without exceeding the existing learned-map
ceiling.

Any active shift, actual/target mismatch, invalid speed, hard-open request, or
post-shift dwell commands zero immediately. A normal map-driven release is
slew limited. Slip is always treated as a magnitude. The controller has no
integral term and transient cycles never enter the legacy adaptation-writing
branch. The existing diagnostic packet is unchanged; `get_transient_snapshot()`
is an internal trace seam for a later safe protocol extension.

The gearbox now passes whether the current engine-RPM read was fresh instead of
silently treating its cached fallback as live TCC feedback. Invalid physical
shaft data bypasses `update()` but explicitly resets transient state first.
Park/Neutral/Reverse, slave mode, diagnostic-control takeover, diagnostic TCC
disable, and map-initialization failure do the same. Re-entering D2 therefore
starts at the first 100 mbar step rather than restoring a frozen command.

## State transitions

Open -> Fill occurs only when the map requests at most 90 RPM slip. Once an
application is active it remains eligible until the map exceeds 110 RPM,
providing Open/Slipping hysteresis. Fill -> SlipControl requires the measured
contact evidence above. SlipControl -> Locked occurs within the lock band. Any
shift/mismatch, invalid speed, hard-open request, or dwell enters Open or
ReleaseFault at zero command. One shared `TccShiftGuard` owns the complete
gearbox lifecycle and 300 ms post-commit dwell for every gear; the D2 controller
does not duplicate that timing state.

## Calibration table

| Constant | Value | Units / bounds | Rationale |
|---|---:|---|---|
| `kPostShiftDwellMs` | 300 | ms, >= 0 | Covers the measured early reapply window; provisional |
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
callback end with actual/target mismatch, post-shift dwell, invalid speed,
hard-open behavior, no-contact pressure cap, negative-slip contact, state reset
and garage D2 re-entry, map-pressure ceiling, and slew. The source-level
integration keeps D3-D5 on the existing path; the full firmware build is the
regression check for that unchanged code path.

Run the mandatory host gate with `sh scripts/test_tcc_transient.sh` before the
full `unified` PlatformIO build. This test is intentionally host-native so the
pure controller and shift guard run without ESP32 hardware.

## Integration, rollback, and risks

The unchanged KWP diagnostics packet is intentional for V1. The internal
snapshot exposes state, slip, map target, trajectory target, feed-forward,
feedback, final pressure, contact, and reason without breaking wire
compatibility. Rollback is the archived official `f9ea309` image and preserved
settings/maps. Remaining risks are unknown hydraulic response to the
provisional pressure law, lack of source-age data in the shared sensor
structure, and possible interaction between the new D2 path and temperature
scaling. If a bounded command still causes a shaft snap, restore `f9ea309` and
inspect TCC hydraulics, damper, solenoid, converter, and driveline causes rather
than increasing pressure constants.
