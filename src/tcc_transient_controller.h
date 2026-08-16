#ifndef TCC_TRANSIENT_CONTROLLER_H
#define TCC_TRANSIENT_CONTROLLER_H

#include <stdint.h>

// Provisional V2 Phase-1 limits. Units are mbar, RPM, and milliseconds.
namespace TccTransientCalibration {
constexpr int kCycleMs = 20;
constexpr int kPostShiftMinSettleMs = 100;
constexpr int kPostShiftStableCycles = 5;
constexpr int kPostShiftRatioErrorRpm = 80;
constexpr int kApplySlewPerCycle = 30;
constexpr int kDemandReleaseSlewPerCycle = 400;
constexpr int kFeedbackGain = 2;
constexpr int kFeedbackLimit = 800;
constexpr int kIntegralErrorDivisor = 8;
constexpr int kIntegralFeedbackLimit = 600;
constexpr int kSlipRateFeedbackGain = 10;
constexpr int kSlipRateFeedbackLimit = 600;
constexpr int kMaxDesiredSlipClosurePerCycle = 10;
constexpr int kHardSlipClosurePerCycle = 25;
constexpr int kCombinedFeedbackLimit = 1400;
constexpr int kFeedbackReliefSlewPerCycle = 60;
constexpr int kApplyTargetSlipRpm = 90;
constexpr int kOpenTargetSlipRpm = 110;
constexpr int kContactSearchPressure = 800;
constexpr int kContactSeekTimeoutMs = 3000;
constexpr int kMaxCommandPressure = 2000;
constexpr int kContactClosurePerCycle = 3;
constexpr int kContactConfirmCycles = 3;
constexpr int kTargetSlipSlewPerCycle = 10;
constexpr int kLockSlipBand = 12;
constexpr int kCoastOverrunOpenSlipRpm = 40;
}

inline bool tcc_transient_applies_to_gear(uint8_t gear, bool gear_enabled) {
    return gear_enabled && gear >= 2 && gear <= 5;
}

enum class TccTransientState : uint8_t {
    Open = 0,
    Fill = 1,
    SlipControl = 2,
    Locked = 3,
    ReleaseFault = 4,
};

enum class TccTransientReason : uint8_t {
    None = 0,
    DemandOpen = 1,
    ShiftInhibit = 2,
    GearMismatch = 3,
    PostShiftSettling = 4,
    InvalidSpeed = 5,
    TargetHysteresis = 6,
    InvalidPressure = 7,
    ExcessiveSlipRate = 8,
    ContactNotDetected = 9,
    CoastOverrun = 10,
};

struct TccTransientInput {
    bool request_apply;
    bool force_open;
    bool speed_valid;
    // Signed engine RPM minus transmission-input RPM. Normal control uses the
    // magnitude; the hard-closure guard preserves the sign so a pull-through
    // from positive to negative slip cannot disappear behind abs().
    int signed_actual_slip_rpm;
    int target_slip_rpm;
    int feedforward_pressure;
    uint32_t now_ms;
    bool coast_mode = false;
};

class TccShiftGuard {
public:
    TccTransientReason step(uint32_t now_ms, bool gearbox_shift_active,
        bool gear_mismatch, bool ratio_sample_valid, int ratio_error_rpm,
        uint32_t ratio_sample_epoch);
    void reset();

private:
    bool lifecycle_was_active_ = false;
    bool settling_active_ = false;
    uint32_t settle_started_ms_ = 0;
    int stable_ratio_cycles_ = 0;
    uint32_t last_ratio_sample_epoch_ = 0;
};

struct TccTransientOutput {
    TccTransientState state;
    TccTransientReason reason;
    int pressure;
    int feedforward_pressure;
    int feedback_correction;
    int integral_correction;
    int slip_rate_correction;
    int slip_rpm;
    int slip_delta_rpm;
    int target_slip_rpm;
    int trajectory_slip_rpm;
    bool contact_detected;
};

class TccTransientController {
public:
    TccTransientOutput step(const TccTransientInput& input);
    bool active() const { return state_ != TccTransientState::Open; }
    void select_gear(uint8_t gear);
    void reset();

private:
    void reset_control_state();
    TccTransientState state_ = TccTransientState::Open;
    TccTransientReason reason_ = TccTransientReason::None;
    int pressure_ = 0;
    uint32_t fill_started_ms_ = 0;
    int previous_slip_rpm_ = 0;
    int previous_signed_slip_rpm_ = 0;
    int trajectory_slip_rpm_ = 0;
    int integral_correction_ = 0;
    int contact_confirm_cycles_ = 0;
    bool contact_detected_ = false;
    bool coast_open_latched_ = false;
    uint8_t selected_gear_ = 0xFF;
};

#endif
