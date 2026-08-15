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

bool TccShiftGuard::step(uint32_t now_ms, bool lifecycle_active) {
    if (lifecycle_active) {
        lifecycle_was_active_ = true;
        dwell_active_ = false;
        return true;
    }
    if (lifecycle_was_active_) {
        lifecycle_was_active_ = false;
        dwell_active_ = true;
        dwell_until_ms_ = now_ms + TccTransientCalibration::kPostShiftDwellMs;
    }
    if (dwell_active_ && before(now_ms, dwell_until_ms_)) {
        return true;
    }
    dwell_active_ = false;
    return false;
}

void TccShiftGuard::reset() {
    lifecycle_was_active_ = false;
    dwell_active_ = false;
    dwell_until_ms_ = 0;
}

void TccTransientController::reset() {
    state_ = TccTransientState::Open;
    reason_ = TccTransientReason::None;
    pressure_ = 0;
    fill_entry_slip_rpm_ = 0;
    previous_slip_rpm_ = 0;
    trajectory_slip_rpm_ = 0;
    contact_confirm_cycles_ = 0;
    contact_detected_ = false;
}

TccTransientOutput TccTransientController::step(const TccTransientInput& input) {
    const int actual_slip = clamp_int(input.actual_slip_rpm < 0 ? -input.actual_slip_rpm : input.actual_slip_rpm, 0, 10000);
    const int target_slip = clamp_int(input.target_slip_rpm, 0, 10000);
    const int feedforward = clamp_int(input.feedforward_pressure, 0, 10000);
    if (!input.speed_valid) {
        state_ = TccTransientState::ReleaseFault;
        reason_ = TccTransientReason::InvalidSpeed;
        pressure_ = 0;
        contact_detected_ = false;
        return {state_, reason_, pressure_, feedforward, 0, actual_slip, target_slip,
            trajectory_slip_rpm_, contact_detected_};
    }
    if (feedforward <= 0 && input.request_apply) {
        state_ = TccTransientState::ReleaseFault;
        reason_ = TccTransientReason::InvalidPressure;
        pressure_ = 0;
        contact_detected_ = false;
        return {state_, reason_, pressure_, feedforward, 0, actual_slip, target_slip,
            trajectory_slip_rpm_, contact_detected_};
    }
    if (input.force_open) {
        state_ = TccTransientState::Open;
        reason_ = TccTransientReason::DemandOpen;
        pressure_ = 0;
        contact_detected_ = false;
        return {state_, reason_, pressure_, feedforward, 0, actual_slip, target_slip,
            trajectory_slip_rpm_, contact_detected_};
    }

    const bool already_applying = state_ == TccTransientState::Fill ||
        state_ == TccTransientState::SlipControl || state_ == TccTransientState::Locked;
    const bool apply_allowed = already_applying
        ? (input.request_apply || target_slip < TccTransientCalibration::kOpenTargetSlipRpm)
        : (input.request_apply && target_slip <= TccTransientCalibration::kApplyTargetSlipRpm);

    if (!apply_allowed) {
        reason_ = input.request_apply
            ? TccTransientReason::TargetHysteresis : TccTransientReason::DemandOpen;
        state_ = TccTransientState::Open;
        pressure_ = slew(pressure_, 0, TccTransientCalibration::kDemandReleaseSlewPerCycle);
        contact_detected_ = false;
        return {state_, reason_, pressure_, feedforward, 0, actual_slip, target_slip,
            trajectory_slip_rpm_, contact_detected_};
    }

    reason_ = TccTransientReason::None;
    if (!already_applying) {
        state_ = TccTransientState::Fill;
        fill_entry_slip_rpm_ = actual_slip;
        previous_slip_rpm_ = actual_slip;
        trajectory_slip_rpm_ = actual_slip;
        contact_confirm_cycles_ = 0;
        contact_detected_ = false;
    }

    int feedback = 0;
    if (state_ == TccTransientState::Fill) {
        // Fill approaches, but never exceeds, the existing learned map value.
        // There is no 10,000 mbar transient command in this path.
        const int contact_search_pressure = feedforward < TccTransientCalibration::kContactSearchPressure
            ? feedforward : TccTransientCalibration::kContactSearchPressure;
        pressure_ = slew(pressure_, contact_search_pressure, TccTransientCalibration::kApplySlewPerCycle);
        const bool cumulative_drop = fill_entry_slip_rpm_ - actual_slip >= TccTransientCalibration::kContactSlipDropRpm;
        const bool falling_or_steady = actual_slip <= previous_slip_rpm_ + 2;
        const bool in_target_band = actual_slip <= target_slip + TccTransientCalibration::kLockSlipBand;
        const bool contact_evidence = cumulative_drop || in_target_band;
        if (pressure_ > 0 && contact_evidence && falling_or_steady) {
            contact_confirm_cycles_ += 1;
        } else if (!falling_or_steady) {
            contact_confirm_cycles_ = 0;
        }
        previous_slip_rpm_ = actual_slip;
        contact_detected_ = contact_confirm_cycles_ >= TccTransientCalibration::kContactConfirmCycles;
        if (contact_detected_) {
            trajectory_slip_rpm_ = actual_slip;
            state_ = TccTransientState::SlipControl;
        }
    } else {
        trajectory_slip_rpm_ = slew(trajectory_slip_rpm_, target_slip,
            TccTransientCalibration::kTargetSlipSlewPerCycle);
        feedback = clamp_int((actual_slip - trajectory_slip_rpm_) * TccTransientCalibration::kFeedbackGain,
            -TccTransientCalibration::kFeedbackLimit, TccTransientCalibration::kFeedbackLimit);
        const int requested = clamp_int(feedforward + feedback, 0, feedforward);
        pressure_ = slew(pressure_, requested, TccTransientCalibration::kApplySlewPerCycle);
        state_ = target_slip <= TccTransientCalibration::kLockSlipBand &&
            actual_slip <= target_slip + TccTransientCalibration::kLockSlipBand
            ? TccTransientState::Locked : TccTransientState::SlipControl;
    }

    return {state_, reason_, pressure_, feedforward, feedback, actual_slip, target_slip,
        trajectory_slip_rpm_, contact_detected_};
}
