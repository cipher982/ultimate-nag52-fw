# TCC V2 Phase 2: Log-Constrained Contact Search

## Decision

Phase 2 remains **offline-only**. It uses the installed `482b299` V1 road log
and independent review to add one missing disturbance class, preserve signed
slip for the safety guard, and bound ambiguous contact search. The existing
100 ms diagnostic cadence cannot validate the new 20 ms rate thresholds. Do not
flash this branch until one control-rate V1 baseline confirms those thresholds.

## What the V1 log establishes

Source:
`g55/logs/tcu-tests/20260815-482b299-first-road-test/first-loop.jsonl`
(1,820 samples over 188.1 seconds, zero recorded diagnostic errors).

The clearest stable-D2 loaded application begins at 86.935 seconds. Pedal stays
at 84-86 raw and reported engine torque at 177-190 Nm while:

- slip is already falling from 243 RPM before decisive clutch influence;
- the controller holds an 800 mbar target while its software pressure estimate
  rises from 80 to 422 mbar and slip drifts from 235 to about 203 RPM;
- the target then rises to about 1,130 mbar; and
- slip later falls from 195 to 42 RPM in 108 ms, equivalent to an average
  28.3 RPM per 20 ms, then crosses to -12 RPM on the next sample.

A second loaded D2 application at 65.809 seconds is much slower: with pedal 71
and torque near 133 Nm, slip falls from 211 to 71 RPM over roughly 2.5 seconds
while the target reaches 1,104 mbar. The large difference proves that a single
fixed clutch-gain value is not identified by this capture.

The log can constrain initial slip (roughly 200-250 RPM), target slip (roughly
40-50 RPM), feed-forward pressure (roughly 1,100-1,150 mbar), and average
closure rate. It cannot identify physical contact pressure, hydraulic transport
delay, or instantaneous 20 ms closure because:

- RLI samples are about 100 ms apart; and
- `current_pressure_mbar` is the firmware's first-order command estimate, not a
  hydraulic pressure sensor.

## Defect exposed by the added disturbance

Phase 1 modeled open-converter slip as moving toward a fixed equilibrium. The
road event shows that open-converter slip can already be falling as road and
turbine speed change. A new no-contact plant therefore ramps free slip downward
while clutch contact remains above the initial 800 mbar search level.

The Phase-1 cumulative 25 RPM detector fails this case: it declares contact at
1.2 seconds solely from background slip motion, raises pressure from 800 to
1,400 mbar, and closes the clutch. That is a direct software explanation for
why a bounded V1 fill can still produce a later clamp.

## Phase-2 controller change

The independently reviewed revision:

1. approaches 800 mbar at the normal 30 mbar per 20 ms limit;
2. requires at least 3 RPM closure per cycle for three consecutive cycles,
   replacing cumulative motion from Fill entry;
3. holds at 800 mbar when contact remains ambiguous instead of creeping toward
   an unvalidated learned-map pressure;
4. preserves signed engine-minus-input slip for the emergency guard while PI
   and contact control continue to use slip magnitude;
5. transitions to PI slip control only after the rate-confirmed response; and
6. commands zero and latches `ContactNotDetected` after 3 seconds without a
   confirmed response.

The independent 2,000 mbar absolute ceiling, learned-map ceiling, shift guard,
invalid-speed release, and all V1 reset behavior remain. The provisional hard
closure latch is now 25 RPM per 20 ms. Damping begins above 10 RPM per cycle so
it has useful authority before that hard cutoff. Both values remain measurement
gates, not vehicle-validated calibration.

## Offline result

The falling-slip case no longer declares contact from background drift. Plants
whose modeled contact lies above the 800 mbar search cap latch open at timeout
rather than exploring higher pressure without vehicle evidence. A signed
zero-crossing fixture proves that `+20 -> -20 RPM` commands zero immediately.

The safety classification now follows modeled physical slip after a release,
including residual hydraulic pressure. It no longer treats zero commanded
pressure as proof that clutch pressure disappeared, and it enforces the same
`-30 RPM` physical floor documented by the gate.

```text
parameter sweep scenarios=216 qualified=105 controlled_aborts=110 tracking_misses=1 safety_failures=0
```

The one tracking miss is the 750 mbar-contact, gain-5.0, 160 ms hydraulic-lag,
80 ms command-delay, 1,100 mbar feed-forward corner. It remains bounded but
leaves the final target band. Most controlled aborts are deliberate no-contact
timeouts created by the conservative 800 mbar cap; they are not comfort passes.
The existing V1 log cannot determine which modeled plant resembles the truck.

Host tests and the full `unified` firmware build pass.

## Required next measurement

Use installed V1 `482b299`, not this branch. First run a 30-second parked capture
to prove the diagnostic transport actually sustains the requested cadence. Then
capture RLI `0x24` every 20 ms and RLI `0x27` every fifth cycle during a short
flat-road baseline. The G55 logger's `--fast-tcc-data` profile is read-only and
emits a cadence summary specifically for this gate.

The road measurement must establish the distribution of per-cycle signed and
absolute slip closure, whether three consecutive 3 RPM closure samples occur
before the visible clamp, and whether a 25 RPM abort threshold has false-positive
risk during normal applications. Only then can the contact and abort thresholds
be accepted or revised for a parked/crawl flash candidate.
