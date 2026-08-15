# TCC V2 Phase 2: Log-Constrained Contact Search

## Decision

Phase 2 remains **offline-only**. It uses the installed `482b299` V1 road log
to add one missing disturbance class and revise contact search, but the existing
100 ms diagnostic cadence cannot validate the new 20 ms rate thresholds. Do not
flash this branch until one control-rate V1 baseline confirms those thresholds.

## What the V1 log establishes

Source:
`g55/logs/tcu-tests/20260815-482b299-first-road-test/first-loop.jsonl`
(1,820 samples over 188.1 seconds, zero recorded diagnostic errors).

The clearest stable-D2 loaded application begins at 86.935 seconds. Pedal stays
at 84-86 raw and reported engine torque at 177-190 Nm while:

- slip is already falling from 243 RPM before decisive clutch influence;
- the controller holds an 800 mbar target while slip drifts from 235 to about
  203 RPM;
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

The revised Fill state:

1. approaches 800 mbar at the normal 30 mbar per 20 ms limit;
2. requires at least 3 RPM closure per cycle for three consecutive cycles,
   replacing cumulative motion from Fill entry;
3. if contact is still ambiguous, creeps toward the learned feed-forward ceiling
   at only 5 mbar per cycle;
4. transitions to PI slip control only after the rate-confirmed response; and
5. commands zero and latches `ContactNotDetected` after 3 seconds without a
   confirmed response.

The independent 2,000 mbar absolute ceiling, learned-map ceiling, shift guard,
invalid-speed release, 30 RPM-per-cycle excessive-closure latch, and all V1
reset behavior remain.

## Offline result

The falling-slip case now waits for actual modeled clutch influence rather than
background drift. True no-contact plants reach only their bounded feed-forward
ceiling and then latch open at timeout.

```text
parameter sweep scenarios=216 qualified=188 controlled_aborts=26 tracking_misses=2 safety_failures=0
```

The two tracking misses are the same extreme corner: clutch gain 5.0,
160 ms hydraulic lag, 80 ms command delay, 1,100 mbar feed-forward, at contact
pressures 700 and 750 mbar. Both remain bounded but leave the final target band.
The existing V1 log cannot determine whether that 240 ms combined-delay corner
is physically applicable, so Phase 2 does not add another control layer for it.

Host tests and the full `unified` firmware build pass.

## Required next measurement

Use installed V1 `482b299`, not this branch. Capture RLI `0x24` every 20 ms and
RLI `0x27` every fifth cycle during a short flat-road baseline. The G55 logger's
`--fast-tcc-data` profile is read-only and exists specifically for this gate.
The measurement must establish the distribution of per-cycle slip closure and
whether three consecutive 3 RPM closure samples occur before the visible clamp.
Only then can the 3 RPM contact threshold and 30 RPM abort threshold be accepted
or revised for a parked/crawl flash candidate.
