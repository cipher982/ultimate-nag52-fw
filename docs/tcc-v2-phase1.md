# TCC V2 Phase 1: Offline Closed-Loop Prototype

> Historical baseline. Phase 2 supersedes the 30 RPM guard and no-contact
> behavior described here; see `docs/tcc-v2-phase2-log-envelope.md`.

## Decision

This branch is an offline control prototype based on installed commit
`482b299658f41ab971c19e62e21e1a3fdfd1dca4`. It is **not approved for a
vehicle flash**. The installed V1 remains the rollback and road baseline.

Phase 1 deliberately stops at a small host-native controller, nonlinear plant
ensemble, and executable safety/performance gates. It does not add persistent
adaptation, a detailed 722.6 hydraulic model, HIL hardware, or new vehicle
calibration.

## Controller

The V1 Open/Fill/SlipControl/Locked/ReleaseFault state machine and all existing
shift inhibits remain. Post-contact control adds:

- bounded proportional and integral slip-error correction;
- damping-only slip-rate feedback when closure exceeds 20 RPM per 20 ms;
- 30 mbar-per-cycle pressure application and 60 mbar-per-cycle normal relief;
- anti-windup at zero pressure and the learned feed-forward ceiling; and
- an outer guard that commands zero and latches `ReleaseFault` when measured
  slip magnitude closes by more than 30 RPM in one 20 ms control cycle. The
  latch clears only after apply demand clears or controller state is reset.

The rate term never adds pressure. P/I owns tracking; rate feedback only backs
pressure off during an unexpectedly fast closure. This avoids adding pressure
in response to hydraulic delay.

## Lightweight plant model

`test/host_tcc_closed_loop.cpp` runs the production controller against a
deliberately simple nonlinear model with:

- an unknown contact/dead-zone pressure;
- first-order hydraulic lag and command transport delay;
- variable clutch authority;
- converter slip disturbance toward an open-converter equilibrium; and
- friction torque that opposes relative shaft motion on either side of zero.

This is a robustness test, not a validated digital twin. The software-reported
TCC pressure in vehicle logs is a command-following estimate, not a physical
hydraulic measurement, so the plant parameters are intentionally swept rather
than claimed as identified values.

## Executable gates

The targeted nominal, sharp, and soft plants must:

- detect clutch contact and settle within 4 seconds;
- remain within 30 RPM of the 50 RPM target for the final simulated second;
- avoid more than 30 RPM closure in one cycle and avoid slip below -30 RPM;
- stay within zero, the learned feed-forward ceiling, and the independent
  2,000 mbar command ceiling; and
- preserve all V1 shift, invalid-speed, stale-pressure, and reset behavior.

No-contact plants must remain at the 800 mbar search cap. Invalid speed must
release immediately. Excessive closure must command zero and remain latched
open while apply demand remains.

The broad deterministic sweep covers 216 combinations of contact pressure,
clutch authority, hydraulic time constant, command delay, and feed-forward
pressure. It distinguishes comfort qualification from a controlled safety
abort; it does not pretend every arbitrary plant should be drivable.

## Current result

Host gate result:

```text
parameter sweep scenarios=216 qualified=146 controlled_aborts=36 tracking_misses=34 safety_failures=0
```

The targeted nominal, sharp, soft, no-contact, invalid-speed, and latched-rate
fault cases pass. The full `unified` firmware build also succeeds.

The result is useful but not flash-worthy: 34 modeled plants remain bounded yet
miss the comfort/tracking gate, while 36 deliberately strong plants fall back
to open rather than continue clamping. More controller complexity is not
justified until real V1 logs constrain hydraulic delay and clutch authority.

## Next gate

Keep `482b299` installed. The next bounded phase is to identify a realistic
plant envelope from the existing V1 road capture or one deliberately flat,
instrumented baseline, then rerun this unchanged suite against that envelope.
Only a candidate that passes the realistic envelope, the host invariants, the
full build, and an independent pre-flash review should become a parked/crawl
vehicle candidate.
