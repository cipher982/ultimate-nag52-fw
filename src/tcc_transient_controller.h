#ifndef TCC_TRANSIENT_CONTROLLER_H
#define TCC_TRANSIENT_CONTROLLER_H

#include <stdint.h>

// Provisional V1 limits. Units are mbar, RPM, and milliseconds.
namespace TccTransientCalibration {
constexpr int kCycleMs = 20;
constexpr int kPostShiftDwellMs = 300;
constexpr int kApplyPressureFloor = 300;
constexpr int kFillPressure = 2500;
constexpr int kPressureCeiling = 6000;
constexpr int kApplySlewPerCycle = 300;
constexpr int kReleaseSlewPerCycle = 800;
constexpr int kFeedbackGain = 4;
constexpr int kFeedbackLimit = 600;
constexpr int kOpenSlipHysteresis = 110;
constexpr int kApplySlipHysteresis = 90;
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
    StaleSpeed = 6,
    TargetHysteresis = 7,
};

struct TccTransientInput {
    bool request_apply;
    bool shift_active;
    bool gear_match;
    bool speed_valid;
    bool speed_fresh;
    int actual_slip_rpm;
    int target_slip_rpm;
    int feedforward_pressure;
    int configured_prefill_pressure;
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
    bool was_inhibited_ = false;
    uint32_t dwell_until_ms_ = 0;
};

#endif
