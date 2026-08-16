# TCC V2 Phase 3: Coast Overrun and Honest Comfort Evaluation

## Decision

Phase 3 remains offline-only. It does not change the installed V1 `482b299` and
is not approved for flashing. It adds one concrete behavior from the independent
clean-sheet reviews and tightens what the simulator is allowed to call
comfortable.

## Newly elevated road event

The V1 road log contains a stable-D2, zero-pedal event at 45.95-47.10 seconds.
Actual and target gear remain D2 while the TCC target alternates slipping/closed,
the target pressure climbs from roughly 1,004 to 1,611 mbar, and signed slip
moves from -18 to -327 RPM before rebounding through +67 RPM. Engine speed falls
from 1,355 to 1,079 RPM while input speed remains near 1,400 RPM.

This proves a separate coast-overrun regime exists in the complaint data. It
does not prove TCC pressure is the only cause, because pressure is estimated and
engine drag/road load are not independently identified. It does prove that one
absolute-slip controller is an incomplete behavioral model.

## Controller change

In D2-D5 controlled operation:

1. `pedal_pos <= 15` identifies the existing firmware's low-pedal/coast region.
2. If signed engine-minus-input slip falls below -40 RPM in that region, the
   controller commands zero and latches `CoastOverrun` open.
3. Small recovery or measurement noise cannot restart the clutch while the
   pedal remains low.
4. When pedal rises above 15, the latch clears and the controller reacquires
   from a fresh bounded Fill step, not the previous pressure.

The -40 RPM and pedal-15 thresholds are provisional. The strategy intentionally
prefers reduced coast lockup/engine braking over another uncontrolled clutch
capture. Long-descent heat and engine-braking behavior must be evaluated before
this can become a vehicle candidate.

## Evaluation change

Safety and comfort are now separate gates. A controlled abort can remain safe
without being drivable, and a settled trajectory can remain functional without
being comfortable.

The hard safety abort remains 25 RPM per 20 ms. Comfort qualification now
requires modeled closure no greater than 12 RPM per 20 ms, close to the
controller's 10 RPM desired closure rate. Results:

```text
parameter sweep scenarios=216 qualified=21 controlled_aborts=110 comfort_misses=84 tracking_misses=1 safety_failures=0
```

This replaces the misleading Phase-2 implication that all 105 functionally
settled cases were comfortable. The model remains a deterministic robustness
harness, not a validated hydraulic twin.

## Required measurement

After the parked cadence gate passes, one flat-road V1 capture must include:

- at least five stable-D2 loaded applications;
- at least three zero-pedal D2 coast events through tip-in; and
- clean 1->2 and 2->3 shifts with stable comparison segments.

The capture decides whether contact develops gradually enough for 20 ms feedback,
whether non-contact motion can false-trigger the provisional thresholds, and
whether negative-slip coast behavior requires the open-first strategy. No Phase-3
binary should be flashed before those questions are answered.
