#include "tcc_direct_slip_controller.h"

namespace {
int clamp_int(int value, int low, int high) {
    return value < low ? low : (value > high ? high : value);
}

int abs_int(int value) {
    return value < 0 ? -value : value;
}

int move_toward(int value, int target, int rise_limit, int fall_limit) {
    if (value < target) {
        return value + ((target - value) > rise_limit ? rise_limit : target - value);
    }
    if (value > target) {
        return value - ((value - target) > fall_limit ? fall_limit : value - target);
    }
    return value;
}
}

void TccDirectSlipController::reset_control_state() {
    active_ = false;
    feedforward_reached_ = false;
    feedback_ready_ = false;
    direction_settling_ = false;
    nontracking_timer_active_ = false;
    integral_scaled_ = 0;
    active_direction_ = 0;
    tracking_cycles_ = 0;
    feedforward_reached_ms_ = 0;
    direction_changed_ms_ = 0;
    nontracking_started_ms_ = 0;
    trajectory_target_rpm_ = TccDirectSlipCalibration::kRequestedTargetMaximumRpm;
}

void TccDirectSlipController::open(TccDirectSlipReason reason) {
    output_ = {};
    output_.state = TccDirectSlipState::Open;
    output_.reason = reason;
    output_.fault_latched = fault_latched_;
    reset_control_state();
}

void TccDirectSlipController::latch_fault() {
    fault_latched_ = true;
    output_.state = TccDirectSlipState::FaultOpen;
    output_.reason = TccDirectSlipReason::TrackingTimeout;
    output_.pressure_delta_mbar = -output_.pressure_mbar;
    output_.pressure_mbar = 0;
    output_.nontracking_ms = TccDirectSlipCalibration::kNonTrackingTimeoutMs;
    output_.tracking_achieved = false;
    output_.fault_latched = true;
    output_.shift_hold = false;
    reset_control_state();
}

void TccDirectSlipController::clear_fault() {
    fault_latched_ = false;
    open(TccDirectSlipReason::NotSelected);
}

void TccDirectSlipController::reset_nonfault_state() {
    if (fault_latched_) {
        output_.state = TccDirectSlipState::FaultOpen;
        output_.reason = TccDirectSlipReason::TrackingTimeout;
        output_.pressure_mbar = 0;
        output_.pressure_delta_mbar = 0;
        output_.fault_latched = true;
        output_.tracking_achieved = false;
        output_.shift_hold = false;
        reset_control_state();
    } else {
        open(TccDirectSlipReason::NotSelected);
    }
}

TccDirectSlipOutput TccDirectSlipController::step(const TccDirectSlipInput& input) {
    const int signed_slip_rpm = clamp_int(input.signed_slip_rpm, -10000, 10000);
    const int input_direction = clamp_int(input.torque_direction, -1, 1);
    const int requested_target_rpm = clamp_int(
        input.target_slip_rpm,
        TccDirectSlipCalibration::kRequestedTargetMinimumRpm,
        TccDirectSlipCalibration::kRequestedTargetMaximumRpm
    );

    if (fault_latched_) {
        output_.state = TccDirectSlipState::FaultOpen;
        output_.reason = TccDirectSlipReason::TrackingTimeout;
        output_.pressure_mbar = 0;
        output_.pressure_delta_mbar = 0;
        output_.signed_slip_rpm = signed_slip_rpm;
        output_.fault_latched = true;
        output_.tracking_achieved = false;
        output_.shift_hold = false;
        return output_;
    }
    if (!input.selected) {
        open(TccDirectSlipReason::NotSelected);
        output_.signed_slip_rpm = signed_slip_rpm;
        return output_;
    }
    if (input.inhibit_reason != TccDirectSlipReason::None) {
        open(input.inhibit_reason);
        output_.signed_slip_rpm = signed_slip_rpm;
        return output_;
    }
    if (!input.request_control) {
        open(TccDirectSlipReason::DemandOpen);
        output_.signed_slip_rpm = signed_slip_rpm;
        return output_;
    }
    if (!input.speed_valid) {
        open(TccDirectSlipReason::InvalidSpeed);
        output_.signed_slip_rpm = signed_slip_rpm;
        return output_;
    }

    if (!active_) {
        active_ = true;
        output_ = {};
        output_.state = TccDirectSlipState::Applying;
        active_direction_ = input_direction;
        trajectory_target_rpm_ = clamp_int(
            abs_int(signed_slip_rpm),
            requested_target_rpm,
            TccDirectSlipCalibration::kInitialTargetMaximumRpm
        );
    } else if (input_direction != 0 && active_direction_ != 0 &&
        input_direction != active_direction_) {
        active_direction_ = input_direction;
        direction_settling_ = true;
        direction_changed_ms_ = input.now_ms;
        integral_scaled_ = 0;
        tracking_cycles_ = 0;
        nontracking_timer_active_ = false;
        trajectory_target_rpm_ = clamp_int(
            abs_int(signed_slip_rpm),
            requested_target_rpm,
            TccDirectSlipCalibration::kInitialTargetMaximumRpm
        );
    } else if (active_direction_ == 0 && input_direction != 0) {
        active_direction_ = input_direction;
    }

    if (direction_settling_ &&
        input.now_ms - direction_changed_ms_ >= TccDirectSlipCalibration::kDirectionSettleMs) {
        direction_settling_ = false;
    }

    if (input.shift_active) {
        if (trajectory_target_rpm_ < TccDirectSlipCalibration::kShiftTargetRpm) {
            trajectory_target_rpm_ = move_toward(
                trajectory_target_rpm_,
                TccDirectSlipCalibration::kShiftTargetRpm,
                TccDirectSlipCalibration::kShiftTargetRiseRpmPerCycle,
                0
            );
        }
    } else {
        trajectory_target_rpm_ = move_toward(
            trajectory_target_rpm_,
            requested_target_rpm,
            TccDirectSlipCalibration::kTargetSlewRpmPerCycle,
            TccDirectSlipCalibration::kTargetSlewRpmPerCycle
        );
    }

    const int feedforward_pressure_mbar = clamp_int(
        input.feedforward_pressure_mbar,
        TccDirectSlipCalibration::kMinimumFeedForwardPressureMbar,
        TccDirectSlipCalibration::kMaximumFeedForwardPressureMbar
    );
    const int oriented_slip_rpm = active_direction_ == 0
        ? 0 : active_direction_ * signed_slip_rpm;
    const int slip_error_rpm = oriented_slip_rpm - trajectory_target_rpm_;

    output_.signed_slip_rpm = signed_slip_rpm;
    output_.oriented_slip_rpm = oriented_slip_rpm;
    output_.slip_error_rpm = slip_error_rpm;
    output_.target_slip_rpm = trajectory_target_rpm_;
    output_.feedforward_pressure_mbar = feedforward_pressure_mbar;
    output_.torque_direction = input_direction;
    output_.pressure_delta_mbar = 0;
    output_.proportional_pressure_mbar = 0;
    output_.integral_pressure_mbar = integral_scaled_ / TccDirectSlipCalibration::kIntegralScale;
    output_.fault_latched = false;
    output_.shift_hold = input.shift_active;

    if (input.shift_active) {
        output_.state = TccDirectSlipState::ShiftHold;
        output_.reason = TccDirectSlipReason::ShiftActive;
        output_.tracking_achieved = false;
        output_.nontracking_ms = 0;
        nontracking_timer_active_ = false;
        tracking_cycles_ = 0;
        return output_;
    }

    output_.reason = TccDirectSlipReason::None;
    const int previous_pressure_mbar = output_.pressure_mbar;
    if (!feedforward_reached_) {
        output_.pressure_mbar = move_toward(
            previous_pressure_mbar,
            feedforward_pressure_mbar,
            TccDirectSlipCalibration::kApplyRisePerCycleMbar,
            TccDirectSlipCalibration::kRegulateFallPerCycleMbar
        );
        if (output_.pressure_mbar == feedforward_pressure_mbar) {
            feedforward_reached_ = true;
            feedforward_reached_ms_ = input.now_ms;
        }
    } else if (!feedback_ready_) {
        output_.pressure_mbar = move_toward(
            previous_pressure_mbar,
            feedforward_pressure_mbar,
            TccDirectSlipCalibration::kApplyRisePerCycleMbar,
            TccDirectSlipCalibration::kRegulateFallPerCycleMbar
        );
        if (input.now_ms - feedforward_reached_ms_ >= TccDirectSlipCalibration::kFillSettleMs) {
            feedback_ready_ = true;
        }
    }

    const bool feedback_allowed = feedback_ready_ && input_direction != 0 &&
        !direction_settling_;
    if (feedback_allowed) {
        const int proportional_pressure_mbar = clamp_int(
            TccDirectSlipCalibration::kProportionalMbarPerRpm * slip_error_rpm,
            TccDirectSlipCalibration::kProportionalMinimumMbar,
            TccDirectSlipCalibration::kProportionalMaximumMbar
        );
        int proposed_integral_scaled = clamp_int(
            integral_scaled_ + slip_error_rpm,
            TccDirectSlipCalibration::kIntegralMinimumMbar * TccDirectSlipCalibration::kIntegralScale,
            TccDirectSlipCalibration::kIntegralMaximumMbar * TccDirectSlipCalibration::kIntegralScale
        );
        const int proposed_unclamped_pressure_mbar = feedforward_pressure_mbar +
            proportional_pressure_mbar +
            proposed_integral_scaled / TccDirectSlipCalibration::kIntegralScale;
        const bool integrating_further_into_high_saturation =
            proposed_unclamped_pressure_mbar > TccDirectSlipCalibration::kMaxCommandPressureMbar &&
            slip_error_rpm > 0;
        const bool integrating_further_into_low_saturation =
            proposed_unclamped_pressure_mbar < TccDirectSlipCalibration::kMinimumControlPressureMbar &&
            slip_error_rpm < 0;
        if (!integrating_further_into_high_saturation &&
            !integrating_further_into_low_saturation) {
            integral_scaled_ = proposed_integral_scaled;
        }

        const int integral_pressure_mbar =
            integral_scaled_ / TccDirectSlipCalibration::kIntegralScale;
        const int requested_pressure_mbar = clamp_int(
            feedforward_pressure_mbar + proportional_pressure_mbar + integral_pressure_mbar,
            TccDirectSlipCalibration::kMinimumControlPressureMbar,
            TccDirectSlipCalibration::kMaxCommandPressureMbar
        );
        output_.pressure_mbar = move_toward(
            previous_pressure_mbar,
            requested_pressure_mbar,
            TccDirectSlipCalibration::kRegulateRisePerCycleMbar,
            TccDirectSlipCalibration::kRegulateFallPerCycleMbar
        );
        output_.proportional_pressure_mbar = proportional_pressure_mbar;
        output_.integral_pressure_mbar = integral_pressure_mbar;
    }
    output_.pressure_delta_mbar = output_.pressure_mbar - previous_pressure_mbar;

    const bool in_tracking_band = feedback_ready_ && active_direction_ != 0 &&
        abs_int(slip_error_rpm) <= TccDirectSlipCalibration::kTrackingBandRpm;
    if (in_tracking_band) {
        tracking_cycles_ += 1;
        if (tracking_cycles_ >= TccDirectSlipCalibration::kTrackingConfirmCycles) {
            output_.tracking_achieved = true;
            output_.state = TccDirectSlipState::Tracking;
        } else {
            output_.tracking_achieved = false;
            output_.state = TccDirectSlipState::Applying;
        }
    } else {
        tracking_cycles_ = 0;
        output_.tracking_achieved = false;
        output_.state = TccDirectSlipState::Applying;
    }

    const bool qualified_nontracking = input.allow_fault_monitor && feedback_allowed &&
        output_.pressure_mbar >= TccDirectSlipCalibration::kMaxCommandPressureMbar &&
        abs_int(signed_slip_rpm) >=
            trajectory_target_rpm_ + TccDirectSlipCalibration::kFailureExcessSlipRpm;
    if (qualified_nontracking) {
        if (!nontracking_timer_active_) {
            nontracking_timer_active_ = true;
            nontracking_started_ms_ = input.now_ms;
        }
        output_.nontracking_ms = input.now_ms - nontracking_started_ms_;
    } else {
        nontracking_timer_active_ = false;
        output_.nontracking_ms = 0;
    }

    if (nontracking_timer_active_ &&
        output_.nontracking_ms >= TccDirectSlipCalibration::kNonTrackingTimeoutMs) {
        latch_fault();
    }
    return output_;
}
