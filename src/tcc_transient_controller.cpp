#include "tcc_transient_controller.h"

namespace {
int clamp_int(int value, int low, int high) {
    return value < low ? low : (value > high ? high : value);
}

int slew(int current, int target, int limit) {
    if (target > current) return current + (target - current < limit ? target - current : limit);
    return current - (current - target < limit ? current - target : limit);
}

}

TccTransientReason TccShiftGuard::step(uint32_t now_ms, bool gearbox_shift_active,
    bool gear_mismatch, bool ratio_sample_valid, int ratio_error_rpm,
    uint32_t ratio_sample_epoch) {
    if (gearbox_shift_active || gear_mismatch) {
        lifecycle_was_active_ = true;
        settling_active_ = false;
        stable_ratio_cycles_ = 0;
        last_ratio_sample_epoch_ = ratio_sample_epoch;
        return gearbox_shift_active
            ? TccTransientReason::ShiftInhibit : TccTransientReason::GearMismatch;
    }
    if (lifecycle_was_active_) {
        lifecycle_was_active_ = false;
        settling_active_ = true;
        settle_started_ms_ = now_ms;
        stable_ratio_cycles_ = 0;
        last_ratio_sample_epoch_ = ratio_sample_epoch;
    }
    if (settling_active_) {
        if (now_ms - settle_started_ms_ < TccTransientCalibration::kPostShiftMinSettleMs) {
            return TccTransientReason::PostShiftSettling;
        }
        const bool new_ratio_sample = ratio_sample_epoch != last_ratio_sample_epoch_;
        if (new_ratio_sample) {
            last_ratio_sample_epoch_ = ratio_sample_epoch;
            const int ratio_error = ratio_error_rpm < 0 ? -ratio_error_rpm : ratio_error_rpm;
            if (ratio_sample_valid && ratio_error <= TccTransientCalibration::kPostShiftRatioErrorRpm) {
                stable_ratio_cycles_ += 1;
            } else {
                stable_ratio_cycles_ = 0;
            }
        }
        if (stable_ratio_cycles_ < TccTransientCalibration::kPostShiftStableCycles) {
            return TccTransientReason::PostShiftSettling;
        }
        settling_active_ = false;
    }
    return TccTransientReason::None;
}

void TccShiftGuard::reset() {
    lifecycle_was_active_ = false;
    settling_active_ = false;
    settle_started_ms_ = 0;
    stable_ratio_cycles_ = 0;
    last_ratio_sample_epoch_ = 0;
}

void TccTransientController::reset_control_state() {
    state_ = TccTransientState::Open;
    reason_ = TccTransientReason::None;
    pressure_ = 0;
    fill_entry_slip_rpm_ = 0;
    previous_slip_rpm_ = 0;
    trajectory_slip_rpm_ = 0;
    integral_correction_ = 0;
    contact_confirm_cycles_ = 0;
    contact_detected_ = false;
}

void TccTransientController::select_gear(uint8_t gear) {
    if (gear != selected_gear_) {
        reset_control_state();
        selected_gear_ = gear;
    }
}

void TccTransientController::reset() {
    reset_control_state();
    selected_gear_ = 0xFF;
}

TccTransientOutput TccTransientController::step(const TccTransientInput& input) {
    const int actual_slip = clamp_int(input.actual_slip_rpm < 0 ? -input.actual_slip_rpm : input.actual_slip_rpm, 0, 10000);
    const int target_slip = clamp_int(input.target_slip_rpm, 0, 10000);
    // Persisted adaptation maps are untrusted inputs. V1 has an independent
    // ceiling so a stale/corrupt 10,000 mbar cell cannot recreate the legacy
    // full-pressure apply after contact.
    const int feedforward = clamp_int(input.feedforward_pressure, 0,
        TccTransientCalibration::kMaxCommandPressure);
    if (!input.speed_valid) {
        state_ = TccTransientState::ReleaseFault;
        reason_ = TccTransientReason::InvalidSpeed;
        pressure_ = 0;
        contact_detected_ = false;
        integral_correction_ = 0;
        return {state_, reason_, pressure_, feedforward, 0, 0, 0, actual_slip, 0, target_slip,
            trajectory_slip_rpm_, contact_detected_};
    }
    if (feedforward <= 0 && input.request_apply) {
        state_ = TccTransientState::ReleaseFault;
        reason_ = TccTransientReason::InvalidPressure;
        pressure_ = 0;
        contact_detected_ = false;
        integral_correction_ = 0;
        return {state_, reason_, pressure_, feedforward, 0, 0, 0, actual_slip, 0, target_slip,
            trajectory_slip_rpm_, contact_detected_};
    }
    if (input.force_open) {
        state_ = TccTransientState::Open;
        reason_ = TccTransientReason::DemandOpen;
        pressure_ = 0;
        contact_detected_ = false;
        integral_correction_ = 0;
        return {state_, reason_, pressure_, feedforward, 0, 0, 0, actual_slip, 0, target_slip,
            trajectory_slip_rpm_, contact_detected_};
    }
    if (state_ == TccTransientState::ReleaseFault &&
        reason_ == TccTransientReason::ExcessiveSlipRate && input.request_apply) {
        pressure_ = 0;
        return {state_, reason_, pressure_, feedforward, 0, 0, 0, actual_slip, 0, target_slip,
            trajectory_slip_rpm_, false};
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
        integral_correction_ = 0;
        return {state_, reason_, pressure_, feedforward, 0, 0, 0, actual_slip, 0, target_slip,
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
        integral_correction_ = 0;
    }

    int feedback = 0;
    int slip_rate_feedback = 0;
    int slip_delta = 0;
    if (state_ == TccTransientState::Fill) {
        // Fill approaches, but never exceeds, the existing learned map value.
        // There is no 10,000 mbar transient command in this path.
        const int contact_search_pressure = feedforward < TccTransientCalibration::kContactSearchPressure
            ? feedforward : TccTransientCalibration::kContactSearchPressure;
        pressure_ = slew(pressure_, contact_search_pressure, TccTransientCalibration::kApplySlewPerCycle);
        const bool cumulative_drop = fill_entry_slip_rpm_ - actual_slip >= TccTransientCalibration::kContactSlipDropRpm;
        const bool falling_or_steady = actual_slip <= previous_slip_rpm_ + 2;
        if (pressure_ > 0 && cumulative_drop && falling_or_steady) {
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
        const int proportional_feedback = clamp_int(
            (actual_slip - trajectory_slip_rpm_) * TccTransientCalibration::kFeedbackGain,
            -TccTransientCalibration::kFeedbackLimit, TccTransientCalibration::kFeedbackLimit);
        slip_delta = actual_slip - previous_slip_rpm_;
        if (slip_delta < -TccTransientCalibration::kHardSlipClosurePerCycle) {
            state_ = TccTransientState::ReleaseFault;
            reason_ = TccTransientReason::ExcessiveSlipRate;
            pressure_ = 0;
            integral_correction_ = 0;
            contact_detected_ = false;
            previous_slip_rpm_ = actual_slip;
            return {state_, reason_, pressure_, feedforward, 0, 0, 0, actual_slip, slip_delta,
                target_slip, trajectory_slip_rpm_, contact_detected_};
        }
        slip_rate_feedback = clamp_int(
            (slip_delta + TccTransientCalibration::kMaxDesiredSlipClosurePerCycle) *
                TccTransientCalibration::kSlipRateFeedbackGain,
            -TccTransientCalibration::kSlipRateFeedbackLimit,
            0);
        const int slip_error = actual_slip - trajectory_slip_rpm_;
        const int integral_candidate = clamp_int(
            integral_correction_ + slip_error / TccTransientCalibration::kIntegralErrorDivisor,
            -TccTransientCalibration::kIntegralFeedbackLimit,
            TccTransientCalibration::kIntegralFeedbackLimit);
        const int candidate_unclamped = feedforward + proportional_feedback +
            slip_rate_feedback + integral_candidate;
        const bool winds_down_at_zero = candidate_unclamped <= 0 && slip_error < 0;
        const bool winds_up_at_ceiling = candidate_unclamped >= feedforward && slip_error > 0;
        if (!winds_down_at_zero && !winds_up_at_ceiling) {
            integral_correction_ = integral_candidate;
        }
        feedback = clamp_int(proportional_feedback + integral_correction_ + slip_rate_feedback,
            -TccTransientCalibration::kCombinedFeedbackLimit,
            TccTransientCalibration::kCombinedFeedbackLimit);
        const int requested = clamp_int(feedforward + feedback, 0, feedforward);
        const int pressure_slew = slip_rate_feedback < 0 && requested < pressure_
            ? TccTransientCalibration::kFeedbackReliefSlewPerCycle
            : TccTransientCalibration::kApplySlewPerCycle;
        pressure_ = slew(pressure_, requested, pressure_slew);
        previous_slip_rpm_ = actual_slip;
        state_ = target_slip <= TccTransientCalibration::kLockSlipBand &&
            actual_slip <= target_slip + TccTransientCalibration::kLockSlipBand
            ? TccTransientState::Locked : TccTransientState::SlipControl;
    }

    return {state_, reason_, pressure_, feedforward, feedback, integral_correction_, slip_rate_feedback,
        actual_slip, slip_delta, target_slip,
        trajectory_slip_rpm_, contact_detected_};
}
