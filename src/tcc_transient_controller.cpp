#include "tcc_transient_controller.h"

namespace {
int clamp_int(int value, int low, int high) {
    return value < low ? low : (value > high ? high : value);
}

int slew(int current, int target, int limit) {
    if (target > current) return current + (target - current < limit ? target - current : limit);
    return current - (current - target < limit ? current - target : limit);
}

int approach_trajectory_q8(int current, int target) {
    if (current == target) return target;
    const int delta = target - current;
    const int distance = delta < 0 ? -delta : delta;
    int step = distance * TccTransientCalibration::kCycleMs /
        TccTransientCalibration::kTargetSlipTauMs;
    if (step < 1) step = 1;
    const int maximum_step = TccTransientCalibration::kTargetSlipSlewPerCycle * 256;
    if (step > maximum_step) step = maximum_step;
    return current + (delta > 0 ? step : -step);
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
    fill_started_ms_ = 0;
    previous_slip_rpm_ = 0;
    previous_signed_slip_rpm_ = 0;
    trajectory_slip_rpm_ = 0;
    trajectory_slip_q8_ = 0;
    integrator_pressure_ = 0;
    contact_confirm_cycles_ = 0;
    hard_closure_cycles_ = 0;
    fault_started_ms_ = 0;
    previous_feedforward_ = 0;
    contact_detected_ = false;
    coast_overrun_active_ = false;
}

void TccTransientController::select_gear(uint8_t gear) {
    if (gear != selected_gear_) {
        reset_control_state();
        selected_gear_ = gear;
    }
}

void TccTransientController::reset() {
    reset_control_state();
    fault_retry_count_ = 0;
    selected_gear_ = 0xFF;
}

TccTransientOutput TccTransientController::step(const TccTransientInput& input) {
    const int signed_actual_slip = clamp_int(input.signed_actual_slip_rpm, -10000, 10000);
    const int actual_slip = signed_actual_slip < 0 ? -signed_actual_slip : signed_actual_slip;
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
        integrator_pressure_ = 0;
        return {state_, reason_, pressure_, feedforward, 0, 0, 0, actual_slip, 0, target_slip,
            trajectory_slip_rpm_, contact_detected_};
    }
    if (feedforward <= 0 && input.request_apply) {
        state_ = TccTransientState::ReleaseFault;
        reason_ = TccTransientReason::InvalidPressure;
        pressure_ = 0;
        contact_detected_ = false;
        integrator_pressure_ = 0;
        return {state_, reason_, pressure_, feedforward, 0, 0, 0, actual_slip, 0, target_slip,
            trajectory_slip_rpm_, contact_detected_};
    }
    if (input.force_open) {
        state_ = TccTransientState::Open;
        reason_ = TccTransientReason::DemandOpen;
        pressure_ = 0;
        contact_detected_ = false;
        integrator_pressure_ = 0;
        return {state_, reason_, pressure_, feedforward, 0, 0, 0, actual_slip, 0, target_slip,
            trajectory_slip_rpm_, contact_detected_};
    }
    if (!input.coast_mode) {
        coast_overrun_active_ = false;
    }
    if (input.coast_mode &&
        signed_actual_slip < -TccTransientCalibration::kCoastOverrunOpenSlipRpm) {
        coast_overrun_active_ = true;
    }
    if (input.coast_mode && coast_overrun_active_) {
        // This hold is scoped to the current coast event. When torque direction
        // returns, coast_mode clears and the next application starts at Fill.
        state_ = TccTransientState::Open;
        reason_ = TccTransientReason::CoastOverrun;
        pressure_ = 0;
        contact_detected_ = false;
        integrator_pressure_ = 0;
        hard_closure_cycles_ = 0;
        return {state_, reason_, pressure_, feedforward, 0, 0, 0, actual_slip, 0,
            target_slip, trajectory_slip_rpm_, contact_detected_};
    }
    const bool retryable_fault = state_ == TccTransientState::ReleaseFault &&
        (reason_ == TccTransientReason::ExcessiveSlipRate ||
        reason_ == TccTransientReason::ContactNotDetected);
    if (retryable_fault) {
        if (!input.request_apply) {
            fault_retry_count_ = 0;
            reset_control_state();
            reason_ = TccTransientReason::DemandOpen;
        } else if (fault_retry_count_ >= TccTransientCalibration::kMaxFaultRetries) {
            pressure_ = 0;
            return {state_, reason_, pressure_, feedforward, 0, 0, 0, actual_slip, 0, target_slip,
                trajectory_slip_rpm_, false};
        } else if (input.now_ms - fault_started_ms_ <
            TccTransientCalibration::kFaultRetryCooldownMs) {
            pressure_ = 0;
            return {state_, reason_, pressure_, feedforward, 0, 0, 0, actual_slip, 0, target_slip,
                trajectory_slip_rpm_, false};
        } else {
            // A retry starts as a new bounded Fill attempt.
            fault_retry_count_ += 1;
            reset_control_state();
        }
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
        integrator_pressure_ = 0;
        return {state_, reason_, pressure_, feedforward, 0, 0, 0, actual_slip, 0, target_slip,
            trajectory_slip_rpm_, contact_detected_};
    }

    reason_ = TccTransientReason::None;
    if (!already_applying) {
        state_ = TccTransientState::Fill;
        fill_started_ms_ = input.now_ms;
        previous_slip_rpm_ = actual_slip;
        previous_signed_slip_rpm_ = signed_actual_slip;
        trajectory_slip_rpm_ = actual_slip;
        trajectory_slip_q8_ = actual_slip * 256;
        contact_confirm_cycles_ = 0;
        hard_closure_cycles_ = 0;
        contact_detected_ = false;
        integrator_pressure_ = feedforward;
        previous_feedforward_ = feedforward;
    }

    int feedback = 0;
    int slip_rate_feedback = 0;
    const int slip_delta = actual_slip - previous_slip_rpm_;
    const int signed_slip_delta = signed_actual_slip - previous_signed_slip_rpm_;
    if (signed_slip_delta < -TccTransientCalibration::kHardSlipClosurePerCycle) {
        hard_closure_cycles_ += 1;
    } else {
        hard_closure_cycles_ = 0;
    }
    if (hard_closure_cycles_ >= 2) {
        state_ = TccTransientState::ReleaseFault;
        reason_ = TccTransientReason::ExcessiveSlipRate;
        fault_started_ms_ = input.now_ms;
        pressure_ = 0;
        integrator_pressure_ = 0;
        contact_detected_ = false;
        previous_slip_rpm_ = actual_slip;
        previous_signed_slip_rpm_ = signed_actual_slip;
        return {state_, reason_, pressure_, feedforward, 0, 0, 0, actual_slip, signed_slip_delta,
            target_slip, trajectory_slip_rpm_, contact_detected_};
    }
    if (state_ == TccTransientState::Fill) {
        // Fill approaches, but never exceeds, the existing learned map value.
        // There is no 10,000 mbar transient command in this path.
        const int initial_search_pressure = feedforward < TccTransientCalibration::kContactSearchPressure
            ? feedforward : TccTransientCalibration::kContactSearchPressure;
        pressure_ = slew(pressure_, initial_search_pressure,
            TccTransientCalibration::kApplySlewPerCycle);
        const int closure_per_cycle = previous_slip_rpm_ - actual_slip;
        if (pressure_ > 0 &&
            closure_per_cycle >= TccTransientCalibration::kContactClosurePerCycle) {
            contact_confirm_cycles_ += 1;
        } else {
            contact_confirm_cycles_ = 0;
        }
        previous_slip_rpm_ = actual_slip;
        previous_signed_slip_rpm_ = signed_actual_slip;
        const bool low_slip_entry = actual_slip <= TccTransientCalibration::kLowSlipEntryRpm;
        contact_detected_ = low_slip_entry ||
            contact_confirm_cycles_ >= TccTransientCalibration::kContactConfirmCycles;
        if (contact_detected_) {
            trajectory_slip_rpm_ = actual_slip;
            trajectory_slip_q8_ = actual_slip * 256;
            integrator_pressure_ = feedforward;
            previous_feedforward_ = feedforward;
            state_ = TccTransientState::SlipControl;
        } else if (!low_slip_entry && input.now_ms - fill_started_ms_ >=
            TccTransientCalibration::kContactSeekTimeoutMs) {
            state_ = TccTransientState::ReleaseFault;
            reason_ = TccTransientReason::ContactNotDetected;
            fault_started_ms_ = input.now_ms;
            pressure_ = 0;
            integrator_pressure_ = 0;
        }
    } else {
        const int target_slip_q8 = target_slip * 256;
        trajectory_slip_q8_ = approach_trajectory_q8(trajectory_slip_q8_, target_slip_q8);
        trajectory_slip_rpm_ = (trajectory_slip_q8_ + 128) / 256;
        const int proportional_feedback = clamp_int(
            (actual_slip - trajectory_slip_rpm_) * TccTransientCalibration::kFeedbackGain,
            -TccTransientCalibration::kFeedbackLimit, TccTransientCalibration::kFeedbackLimit);
        slip_rate_feedback = clamp_int(
            (slip_delta + TccTransientCalibration::kMaxDesiredSlipClosurePerCycle) *
                TccTransientCalibration::kSlipRateFeedbackGain,
            -TccTransientCalibration::kSlipRateFeedbackLimit,
            0);
        const int slip_error = actual_slip - trajectory_slip_rpm_;
        const int command_ceiling = feedforward + TccTransientCalibration::kFeedbackHeadroom <
            TccTransientCalibration::kMaxCommandPressure
            ? feedforward + TccTransientCalibration::kFeedbackHeadroom
            : TccTransientCalibration::kMaxCommandPressure;
        if (previous_feedforward_ > 0 && feedforward != previous_feedforward_) {
            const int delta_ff = feedforward - previous_feedforward_;
            integrator_pressure_ = clamp_int(integrator_pressure_ + delta_ff, 0, command_ceiling);
        }
        previous_feedforward_ = feedforward;
        const bool in_integrator_deadband =
            (actual_slip - trajectory_slip_rpm_ <= TccTransientCalibration::kLockSlipBand) &&
            (trajectory_slip_rpm_ - actual_slip <= TccTransientCalibration::kLockSlipBand);
        const int integral_candidate = in_integrator_deadband
            ? clamp_int(integrator_pressure_, 0, command_ceiling)
            : clamp_int(integrator_pressure_ +
                slip_error / TccTransientCalibration::kIntegralErrorDivisor,
                0, command_ceiling);
        const int candidate_unclamped = integral_candidate + proportional_feedback +
            slip_rate_feedback;
        const bool winds_down_at_zero = candidate_unclamped < 0 && slip_error < 0;
        const bool winds_up_at_ceiling = candidate_unclamped > command_ceiling && slip_error > 0;
        if (!winds_down_at_zero && !winds_up_at_ceiling) {
            integrator_pressure_ = integral_candidate;
        } else {
            integrator_pressure_ = clamp_int(integrator_pressure_, 0, command_ceiling);
        }
        const int integral_correction = integrator_pressure_ - feedforward;
        feedback = clamp_int(proportional_feedback + integral_correction + slip_rate_feedback,
            -feedforward, command_ceiling - feedforward);
        const int requested = clamp_int(feedforward + feedback, 0, command_ceiling);
        const int pressure_slew = slip_rate_feedback < 0 && requested < pressure_
            ? TccTransientCalibration::kFeedbackReliefSlewPerCycle
            : TccTransientCalibration::kApplySlewPerCycle;
        pressure_ = slew(pressure_, requested, pressure_slew);
        previous_slip_rpm_ = actual_slip;
        previous_signed_slip_rpm_ = signed_actual_slip;
        const int lock_pressure_threshold = (feedforward * 8) / 10;
        state_ = target_slip <= TccTransientCalibration::kLockSlipBand &&
            actual_slip <= target_slip + TccTransientCalibration::kLockSlipBand &&
            pressure_ >= lock_pressure_threshold
            ? TccTransientState::Locked : TccTransientState::SlipControl;
    }

    const int integral_correction = integrator_pressure_ - feedforward;
    return {state_, reason_, pressure_, feedforward, feedback, integral_correction, slip_rate_feedback,
        actual_slip, signed_slip_delta, target_slip,
        trajectory_slip_rpm_, contact_detected_};
}
