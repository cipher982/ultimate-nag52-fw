#ifndef TCC_TRANSIENT_CONTROLLER_H
#define TCC_TRANSIENT_CONTROLLER_H

#include <stdint.h>

// Provisional V1 limits. Units are mbar, RPM, and milliseconds.
namespace TccTransientCalibration {
constexpr int kCycleMs = 20;
constexpr int kPostShiftDwellMs = 300;
constexpr int kApplySlewPerCycle = 100;
constexpr int kDemandReleaseSlewPerCycle = 400;
constexpr int kFeedbackGain = 4;
constexpr int kFeedbackLimit = 600;
constexpr int kApplyTargetSlipRpm = 90;
constexpr int kOpenTargetSlipRpm = 110;
constexpr int kContactSlipDropRpm = 25;
constexpr int kTargetSlipSlewPerCycle = 10;
constexpr int kLockSlipBand = 12;
}

inline bool tcc_transient_applies_to_gear(uint8_t gear, bool d2_enabled) {
    return d2_enabled && gear == 2;
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
    PostShiftDwell = 4,
    InvalidSpeed = 5,
    TargetHysteresis = 6,
    InvalidPressure = 7,
};

struct TccTransientInput {
    bool request_apply;
    bool force_open;
    bool shift_active;
    bool gear_match;
    bool speed_valid;
    int actual_slip_rpm;
    int target_slip_rpm;
    int feedforward_pressure;
    uint32_t now_ms;
};

struct TccTransientOutput {
    TccTransientState state;
    TccTransientReason reason;
    int pressure;
    int feedforward_pressure;
    int feedback_correction;
    int slip_rpm;
    int target_slip_rpm;
    int trajectory_slip_rpm;
    bool contact_detected;
};

class TccTransientController {
public:
    TccTransientOutput step(const TccTransientInput& input);
    bool active() const { return state_ != TccTransientState::Open; }
    void reset();

private:
    TccTransientState state_ = TccTransientState::Open;
    TccTransientReason reason_ = TccTransientReason::None;
    int pressure_ = 0;
    bool shift_inhibited_ = false;
    uint32_t dwell_until_ms_ = 0;
    int fill_entry_slip_rpm_ = 0;
    int trajectory_slip_rpm_ = 0;
    bool contact_detected_ = false;
};

#endif
