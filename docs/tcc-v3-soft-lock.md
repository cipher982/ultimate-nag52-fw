# V3 TCC Soft-Lock Controller

## Scope

V3 controls the transient TCC apply path for gears 2 through 5. The legacy
gear-1 path remains unchanged. The controller runs once per 20 ms cycle and
uses mbar for pressure and RPM for slip.

## Inputs and Coast Gate

The pedal sensor is a 0-250 raw count. It is normalized before use:

```
pedal_percent = min(100, raw_pedal * 100 / 250)
```

Coast hysteresis is entered at 8% or below and exited at 15% or above. Coast
is also gated by negative converted engine torque. Low pedal with positive
road-load torque is therefore normal cruise, not overrun. A coast overrun
opens the clutch only for the current coast event. When torque direction
returns, coast mode clears and the next apply starts at a bounded Fill.

## State Machine

* `Open`: zero commanded pressure, or target hysteresis/explicit release.
* `Fill`: pressure slews toward `min(feedforward, 800)` while contact is
  confirmed by three consecutive closure samples of at least 3 RPM/cycle.
* `SlipControl`: PI and slip-rate control track the soft-lock trajectory while
  pressure slews toward target.
* `Locked`: the target is in the lock range, measured slip is in its lock
  band, and commanded pressure has reached at least 80% of feedforward.
* `ReleaseFault`: invalid speed/pressure faults release immediately. The
  retryable `ExcessiveSlipRate` and `ContactNotDetected` faults hold zero for
  2000 ms, then allow up to 2 retries per apply request before latching open.

If Fill begins with slip at or below 20 RPM, the controller suppresses the
three-second timeout and transitions to `SlipControl` so pressure slews up
smoothly before declaring `Locked`.

## Pressure Control

The positive feedback ceiling is independent of the learned map:

```
command_ceiling = min(feedforward + 400, 2000)
requested = clamp(feedforward + feedback, 0, command_ceiling)
```

The PI integrator is stored as an absolute pressure bias and is clamped to
`[0, command_ceiling]`, rather than to the feedforward pressure. Its initial
value is the current feedforward pressure. When feedforward steps on load
changes, $\Delta p_\text{ff}$ is added to the integrator to ensure bumpless
transfer without step discontinuities in feedback error.

```
abs(actual_slip - trajectory_slip) <= 12 RPM
```

This deadband prevents RPM quantization and sensor noise from ratcheting the
integrator while locked.

The hard closure guard requires two consecutive samples with signed slip
closure greater than 25 RPM/cycle. A single quantization spike is ignored.

## Slip Trajectory

The reference is held in Q8 fixed point and approaches the target
exponentially:

```
delta = target_q8 - trajectory_q8
step = max(1 Q8, abs(delta) * 20 ms / 400 ms)
trajectory_q8 += sign(delta) * min(step, 10 RPM/cycle)
```

The proportional step therefore falls with the remaining error. The fixed
point state avoids a discontinuous final integer-RPM step; the reported
integer trajectory becomes stationary as the reference reaches the target.
Negative slip-rate feedback is retained as a relief term and is slew-limited
to protect the hydraulic plant.

## Calibration Values

| Parameter | Value |
| --- | ---: |
| Cycle period | 20 ms |
| Feedback headroom | 400 mbar |
| Maximum command | 2000 mbar |
| Fill contact search ceiling | 800 mbar |
| Contact confirmation | 3 samples, 3 RPM/cycle |
| Contact timeout | 3000 ms |
| Retry cooldown / limit | 2000 ms / 2 attempts |
| Low-slip Fill entry | 20 RPM |
| Hard closure guard | 25 RPM/cycle, 2 samples |
| Trajectory time constant | 400 ms |
| Coast entry/exit | 8% / 15% pedal |

## Verification Matrix

The host script `scripts/test_tcc_transient.sh` builds both native binaries
with C++17, warnings, and `-Werror`:

* shift settling, gear reset, target hysteresis, invalid inputs, and pressure
  bounds;
* 10-second 5th-gear low-slip cruise without ContactNotDetected;
* positive road-load disturbance with feedforward plus 400 mbar feedback;
* coast overrun, scoped coast hold, and smooth tip-in reacquisition;
* normalized pedal flutter from 15% through 22% without coast chattering;
* integrator deadband under +/-5 RPM target noise;
* single-sample closure quantization noise and two-sample fault confirmation;
* retry after the bounded 2000 ms fault cooldown;
* closed-loop plant parameter sweep for comfort, tracking, and safety.

### Machine-Readable Closed-Loop Result

The script leaves the closed-loop simulator at
`$BUILD_DIR/host_tcc_closed_loop` (`BUILD_DIR` defaults to `build`). Its
ordinary no-argument invocation retains the assertion-backed human output.
Automation can additionally request a deterministic JSON artifact:

```
BUILD_DIR="${BUILD_DIR:-build}"
"$BUILD_DIR/host_tcc_closed_loop" --json-summary report.json
```

The artifact uses schema `tcc-closed-loop-v1`. Its `summary` object contains
the sweep counts (`scenarios`, `qualified`, `controlled_aborts`,
`comfort_misses`, `tracking_misses`, and `safety_failures`), while the
top-level `scenarios` array contains the metrics for every named plant
scenario. Within each scenario, `rate_fault_ms` is the fault time in
milliseconds or JSON `null` when no rate fault occurred. Invalid command-line
arguments or an unwritable output path return status 2; controller safety
failures remain hard assertions.

### Closed-Loop and Shadow-Replay Boundary

This executable drives controller commands into a deterministic synthetic
hydraulic and clutch plant, so it can exercise modeled feedback dynamics,
including modeled oscillation and jerk. A shadow replay instead feeds the
controller exogenous slip from a historical capture: its command invariants
are useful, but it cannot validate the physical oscillation or jerk that those
commands would produce. Captures carrying `tcc_transient` (RLI `0x2D`) provide
exact controller-state telemetry for replay comparison. Legacy captures
without it are approximate and must not be presented as exact controller
validation.

Verification result: `./scripts/test_tcc_transient.sh` passes both host test
executables with zero assertion failures.
