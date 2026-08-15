#include "tcc_transient_controller.h"

namespace {
int clamp_int(int value, int low, int high) {
    return value < low ? low : (value > high ? high : value);
}

int slew(int current, int target, int limit) {
    if (target > current) return current + (target - current < limit ? target - current : limit);
    return current - (current - target < limit ? current - target : limit);
}

bool before(uint32_t now, uint32_t deadline) {
    return static_cast<int32_t>(now - deadline) < 0;
}
}

void TccTransientController::reset() {
    state_ = TccTransientState::Open;
    reason_ = TccTransientReason::None;
    pressure_ = 0;
    was_inhibited_ = false;
    dwell_until_ms_ = 0;
}

TccTransientOutput TccTransientController::step(const TccTransientInput& input) {
    const bool invalid_speed = !input.speed_valid;
    const bool stale_speed = input.speed_valid && !input.speed_fresh;
    const bool gear_inhibit = input.shift_active || !input.gear_match;
    bool inhibited = invalid_speed || stale_speed || gear_inhibit;

    if (was_inhibited_ && !inhibited) {
        dwell_until_ms_ = input.now_ms + TccTransientCalibration::kPostShiftDwellMs;
    }
    was_inhibited_ = inhibited;

    TccTransientReason reason = TccTransientReason::None;
    if (invalid_speed) reason = TccTransientReason::InvalidSpeed;
    else if (stale_speed) reason = TccTransientReason::StaleSpeed;
    else if (input.shift_active) reason = TccTransientReason::ShiftInhibit;
    else if (!input.gear_match) reason = TccTransientReason::GearMismatch;
    else if (before(input.now_ms, dwell_until_ms_)) {
        inhibited = true;
        reason = TccTransientReason::PostShiftDwell;
    } else if (!input.request_apply) {
        reason = TccTransientReason::DemandOpen;
    }

    const int target_slip = clamp_int(input.target_slip_rpm, 0, 10000);
    const int feedforward = clamp_int(input.feedforward_pressure, 0, TccTransientCalibration::kPressureCeiling);
    int feedback = 0;

    if (inhibited || !input.request_apply) {
        state_ = inhibited && (invalid_speed || stale_speed)
            ? TccTransientState::ReleaseFault : TccTransientState::Open;
        pressure_ = slew(pressure_, 0, TccTransientCalibration::kReleaseSlewPerCycle);
    } else {
        if (state_ == TccTransientState::Open && input.actual_slip_rpm <= TccTransientCalibration::kApplySlipHysteresis) {
            reason = TccTransientReason::TargetHysteresis;
        } else if (state_ == TccTransientState::Open || state_ == TccTransientState::ReleaseFault) {
            state_ = TccTransientState::Fill;
        }

        if (state_ == TccTransientState::Fill) {
            // A configured 10,000 mbar prefill is a fill input, never an output.
            const int fill_target = clamp_int(input.configured_prefill_pressure / 4,
                TccTransientCalibration::kApplyPressureFloor,
                TccTransientCalibration::kFillPressure);
            pressure_ = slew(pressure_, fill_target, TccTransientCalibration::kApplySlewPerCycle);
            if (pressure_ >= fill_target || input.actual_slip_rpm <= target_slip + 20) {
                state_ = TccTransientState::SlipControl;
            }
        }
        else if (state_ == TccTransientState::SlipControl || state_ == TccTransientState::Locked) {
            feedback = clamp_int((input.actual_slip_rpm - target_slip) * TccTransientCalibration::kFeedbackGain,
                -TccTransientCalibration::kFeedbackLimit, TccTransientCalibration::kFeedbackLimit);
            const int requested = clamp_int(feedforward + feedback,
                TccTransientCalibration::kApplyPressureFloor,
                TccTransientCalibration::kPressureCeiling);
            pressure_ = slew(pressure_, requested, TccTransientCalibration::kApplySlewPerCycle);
            state_ = target_slip <= TccTransientCalibration::kLockSlipBand &&
                input.actual_slip_rpm <= target_slip + TccTransientCalibration::kLockSlipBand
                ? TccTransientState::Locked : TccTransientState::SlipControl;
        }
    }

    reason_ = reason;
    return {state_, reason_, pressure_, feedforward, feedback, input.actual_slip_rpm, target_slip};
}
