# TCC V2 Phase 4 QA Instrumentation

This revision closes the offline testability gaps identified after Phase 3. It
does not authorize a vehicle flash.

## Controller and simulation changes

- The closed-loop plant can now represent signed slip down to -500 RPM instead
  of clipping at -200 RPM.
- A closed-loop coast disturbance establishes a pressured TCC application,
  drives open-converter slip toward -360 RPM, verifies the `CoastOverrun`
  fail-open latch through the measured -327 RPM regime, and verifies that
  throttle return starts a fresh bounded Fill step.
- Coast-mode pedal detection now has provisional hysteresis: enter at pedal
  8 or below and exit at 15 or above. This
  prevents one-count pedal noise from alternating open and reacquire commands.
- Unit tests cover the exact -40/-41 RPM overrun boundary and prove that an
  invalid speed sample fails open as `InvalidSpeed` rather than mutating the
  overrun latch.

The closed-loop artifact is schema `tcc-closed-loop-v2` and emits all 216
parameter cells. Each cell carries plant parameters, result metrics, and one
of five legal classifications; summary totals must conserve exactly 216. The
baseline regression budget is qualified >= 15, controlled aborts <= 100,
comfort misses <= 93, tracking misses <= 8, and safety failures == 0. The
latest committed dogfood artifact, `logs/tcu-sim/20260819-dogfood-08/`,
reports 15 qualified cells, 100 controlled aborts, 93 comfort misses, 8
tracking misses, and zero safety failures. These are synthetic-plant
regression results without vehicle-measured provenance, not vehicle acceptance
limits. The machine verdict remains explicit and enforced under `NDEBUG`.

The host replay binary emits `tcc-command-limits-v1` from the compiled
`TccTransientCalibration`, including cycle time, slew rates, pressure limits,
controlled gears, and coast thresholds. Replay hard gates consume this
artifact rather than copied calibration values.

Historical FFT and jerk remain raw exogenous observations with status `INFO`
or `NOT_EVALUATED`; they are not controller or comfort gates. A hard replay
PASS on exogenous traces is `APPROXIMATE_EXOGENOUS`, which is not controller
or vehicle approval. Closed-loop safety, baseline regression, replay hard
gates, and evidence sufficiency remain separate machine fields.

## Read-only controller trace

RLI `0x2D` is a new, read-only 23-byte diagnostic record. RLI `0x24` is
unchanged for config-app compatibility. The new record exports:

- state and reason;
- contact-detected and coast-mode flags;
- pedal position and signed slip;
- per-control-cycle slip delta;
- target and trajectory slip;
- final and feed-forward pressure commands; and
- proportional/integral/rate corrections.

The G55 logger owns the matching decoder and a V2-only 20 ms capture profile.

## Flash-size warning resolution

The project partition table extends to `0x3C6000`, the selected
`esp-wrover-kit` board manifest declares 4 MB flash, and live firmware readback
has reported OTA partitions at `0x110000` and `0x210000`. The inherited
`sdkconfig.unified` still declared a 2 MB image header, which caused the prior
"expected 4 MB, found 2 MB" build warning even though the partition layout
cannot fit in 2 MB.

This Phase 4 revision changed only the ESP-IDF image flash-size declaration
from 2 MB to 4 MB; it did not alter partition offsets or sizes. A full
`unified` build succeeded without the warning at that time. The final
simulator-v2 session passed the firmware host suite, but could not rerun the
full PlatformIO target build, so host results do not establish target
validation.

## Current vehicle evidence gate

V2 commit `a619e59` is installed and was driven home. The owner's highway
oscillation observation is subjective evidence; there is still no
authoritative 20 ms RLI `0x2D` road capture. V3 branch `bf4e3fb` is
implemented and pushed but remains unflashed.

The next action is exact, staged V2 evidence collection: Park, then a 20-foot
crawl, then a short loop captured with `--v2-transient-data`. Review that
trace before revisiting any V3 flash decision. The old Phase 4 gate requiring
a V1 baseline is complete and no longer governs the next step.
