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
  15 or below, retain the mode through 16-19, and exit at 20 or above. This
  prevents one-count pedal noise from alternating open and reacquire commands.
- Unit tests cover the exact -40/-41 RPM overrun boundary and prove that an
  invalid speed sample fails open as `InvalidSpeed` rather than mutating the
  overrun latch.

The 216-case plant sweep remains 21 comfort-qualified, 110 controlled aborts,
84 functional-but-too-fast comfort misses, one tracking miss, and zero modeled
safety failures. These are software-model results, not vehicle validation.

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

This revision changes only the ESP-IDF image flash-size declaration from 2 MB
to 4 MB. It does not alter partition offsets or sizes. A full `unified` build
succeeds without the warning.

## Remaining gate

Keep installed V1 `482b299`. First collect one valid 20 ms V1 baseline and run
the G55 analyzer. The existing 100 ms road log is useful for event discovery
but is explicitly rejected for per-cycle threshold evaluation. Only after the
baseline constrains the provisional thresholds should a clean V2 artifact be
reviewed for flash.
