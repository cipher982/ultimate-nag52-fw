#ifndef TCC_DIRECT_SLIP_CONTROLLER_H
#define TCC_DIRECT_SLIP_CONTROLLER_H

#include <stdint.h>

// V8 follows one signed slip target in D2-D5. During a gear shift it keeps the
// clutch partly applied by relaxing pressure toward the feed-forward holding
// pressure without venting the circuit. Otherwise a bounded PI correction may
// add or remove pressure around the feed-forward command.
namespace TccDirectSlipCalibration {
constexpr int kCycleMs = 20;
constexpr int kMinimumControlPressureMbar = 1000;
constexpr int kMinimumFeedForwardPressureMbar = 1500;
constexpr int kMaximumFeedForwardPressureMbar = 2000;
constexpr int kMaxCommandPressureMbar = 3000;
constexpr int kApplyRisePerCycleMbar = 100;
constexpr int kRegulateRisePerCycleMbar = 25;
constexpr int kRegulateFallPerCycleMbar = 50;
constexpr int kShiftRelaxPerCycleMbar = 50;
constexpr int kProportionalMbarPerRpm = 2;
constexpr int kProportionalMinimumMbar = -1000;
constexpr int kProportionalMaximumMbar = 1500;
constexpr int kIntegralScale = 50; // 20 ms cycles per second
constexpr int kIntegralMinimumMbar = -1000;
constexpr int kIntegralMaximumMbar = 1000;
constexpr int kRequestedTargetMinimumRpm = 20;
constexpr int kRequestedTargetMaximumRpm = 80;
constexpr int kInitialTargetMaximumRpm = 300;
constexpr int kTargetSlewRpmPerCycle = 4;
constexpr int kShiftTargetRpm = 150;
constexpr int kShiftTargetRiseRpmPerCycle = 8;
constexpr uint32_t kFillSettleMs = 200;
constexpr uint32_t kDirectionSettleMs = 200;
constexpr int kTrackingBandRpm = 30;
constexpr int kTrackingConfirmCycles = 3;
constexpr int kFailureExcessSlipRpm = 100;
constexpr uint32_t kNonTrackingTimeoutMs = 4000;
}

enum class TccDirectSlipState : uint8_t {
    Open = 0,
    ShiftHold = 1,
    Applying = 2,
    Tracking = 3,
    FaultOpen = 4,
};

enum class TccDirectSlipReason : uint8_t {
    None = 0,
    NotSelected = 1,
    DemandOpen = 2,
    ShiftActive = 3,
    GearMismatch = 4,
    InvalidSpeed = 5,
    NonPositiveTorque = 6, // Retained for diagnostic compatibility.
    TrackingTimeout = 7,
    ControllerDisabled = 8,
    LowInputSpeed = 9,
    Temperature = 10,
    EngineRequest = 11,
};

struct TccDirectSlipInput {
    bool selected;
    bool request_control;
    bool speed_valid;
    bool shift_active;
    bool allow_fault_monitor;
    TccDirectSlipReason inhibit_reason;
    int signed_slip_rpm;
    int torque_direction; // +1 drive, -1 coast, 0 indeterminate
    int target_slip_rpm;
    int feedforward_pressure_mbar;
    uint32_t now_ms;
};

struct TccDirectSlipOutput {
    TccDirectSlipState state;
    TccDirectSlipReason reason;
    int pressure_mbar;
    int pressure_delta_mbar;
    int signed_slip_rpm;
    int oriented_slip_rpm;
    int slip_error_rpm;
    int target_slip_rpm;
    int feedforward_pressure_mbar;
    int proportional_pressure_mbar;
    int integral_pressure_mbar;
    int torque_direction;
    uint32_t nontracking_ms;
    bool tracking_achieved;
    bool fault_latched;
    bool shift_hold;
};

class TccDirectSlipController {
public:
    TccDirectSlipOutput step(const TccDirectSlipInput& input);
    void clear_fault();
    void reset_nonfault_state();
    TccDirectSlipOutput snapshot() const { return output_; }

private:
    void open(TccDirectSlipReason reason);
    void latch_fault();
    void reset_control_state();

    TccDirectSlipOutput output_ = {};
    uint32_t feedforward_reached_ms_ = 0;
    uint32_t direction_changed_ms_ = 0;
    uint32_t nontracking_started_ms_ = 0;
    int integral_scaled_ = 0;
    int trajectory_target_rpm_ = TccDirectSlipCalibration::kRequestedTargetMaximumRpm;
    int active_direction_ = 0;
    int tracking_cycles_ = 0;
    bool active_ = false;
    bool feedforward_reached_ = false;
    bool feedback_ready_ = false;
    bool direction_settling_ = false;
    bool nontracking_timer_active_ = false;
    bool fault_latched_ = false;
};

#endif
