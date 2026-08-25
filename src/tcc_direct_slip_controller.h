#ifndef TCC_DIRECT_SLIP_CONTROLLER_H
#define TCC_DIRECT_SLIP_CONTROLLER_H

#include <stdint.h>

// V7 keeps the converter applied in every stable enabled forward gear. The
// pressure floor preserves clutch capacity through zero-torque crossings;
// absolute-slip feedback may only add pressure above that floor. This avoids
// both the V6 coast release and its signed-slip polarity failure.
namespace TccDirectSlipCalibration {
constexpr int kCycleMs = 20;
constexpr int kMinimumHoldPressureMbar = 1500;
constexpr int kMaximumFeedForwardPressureMbar = 2000;
constexpr int kMaxCommandPressureMbar = 3000;
constexpr int kPressureRisePerCycleMbar = 100;
constexpr int kPressureFallPerCycleMbar = 50;
constexpr int kFeedbackRiseStepMbar = 50;
constexpr uint32_t kFeedbackStepIntervalMs = 100;
constexpr int kTrackingBandRpm = 30;
constexpr int kTrackingConfirmCycles = 3;
constexpr int kFailureSlipRpm = 100;
constexpr uint32_t kNonTrackingTimeoutMs = 4000;
constexpr uint32_t kRetryCooldownMs = 1000;
}

enum class TccDirectSlipState : uint8_t {
    Open = 0,
    ShiftInhibit = 1,
    Applying = 2,
    Tracking = 3,
    RetryCooldown = 4,
};

enum class TccDirectSlipReason : uint8_t {
    None = 0,
    NotSelected = 1,
    DemandOpen = 2,
    ShiftActive = 3,
    GearMismatch = 4,
    InvalidSpeed = 5,
    NonPositiveTorque = 6, // Retained for diagnostic compatibility; unused by V7.
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
    TccDirectSlipReason inhibit_reason;
    int signed_slip_rpm;
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
    int slip_error_rpm;
    int target_slip_rpm;
    uint32_t nontracking_ms;
    bool tracking_achieved;
    bool fault_latched;
};

class TccDirectSlipController {
public:
    TccDirectSlipOutput step(const TccDirectSlipInput& input);
    void clear_fault();
    void reset_nonfault_state();
    TccDirectSlipOutput snapshot() const { return output_; }

private:
    void open(TccDirectSlipReason reason, TccDirectSlipState state);
    void start_retry_cooldown(uint32_t now_ms);

    TccDirectSlipOutput output_ = {
        TccDirectSlipState::Open,
        TccDirectSlipReason::NotSelected,
        0, 0, 0, 0, 0, 0, false, false
    };
    uint32_t last_feedback_step_ms_ = 0;
    uint32_t nontracking_started_ms_ = 0;
    uint32_t retry_until_ms_ = 0;
    int feedback_trim_mbar_ = 0;
    int tracking_cycles_ = 0;
    bool applying_ = false;
    bool base_pressure_reached_ = false;
    bool nontracking_timer_active_ = false;
};

#endif
