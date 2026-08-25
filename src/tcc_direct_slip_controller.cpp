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

void TccDirectSlipController::open(TccDirectSlipReason reason, TccDirectSlipState state) {
    output_.state = state;
    output_.reason = reason;
    output_.pressure_mbar = 0;
    output_.pressure_delta_mbar = 0;
    output_.nontracking_ms = 0;
    output_.tracking_achieved = false;
    output_.fault_latched = false;
    applying_ = false;
    base_pressure_reached_ = false;
    nontracking_timer_active_ = false;
    feedback_trim_mbar_ = 0;
    tracking_cycles_ = 0;
}

void TccDirectSlipController::start_retry_cooldown(uint32_t now_ms) {
    open(TccDirectSlipReason::TrackingTimeout, TccDirectSlipState::RetryCooldown);
    retry_until_ms_ = now_ms + TccDirectSlipCalibration::kRetryCooldownMs;
}

void TccDirectSlipController::clear_fault() {
    retry_until_ms_ = 0;
    open(TccDirectSlipReason::NotSelected, TccDirectSlipState::Open);
}

void TccDirectSlipController::reset_nonfault_state() {
    retry_until_ms_ = 0;
    open(TccDirectSlipReason::NotSelected, TccDirectSlipState::Open);
}

TccDirectSlipOutput TccDirectSlipController::step(const TccDirectSlipInput& input) {
    const int slip_magnitude_rpm = abs_int(clamp_int(input.signed_slip_rpm, -10000, 10000));
    output_.signed_slip_rpm = clamp_int(input.signed_slip_rpm, -10000, 10000);
    output_.target_slip_rpm = clamp_int(input.target_slip_rpm, 0, 10000);
    output_.slip_error_rpm = slip_magnitude_rpm - output_.target_slip_rpm;
    output_.pressure_delta_mbar = 0;
    output_.fault_latched = false;

    if (!input.selected) {
        open(TccDirectSlipReason::NotSelected, TccDirectSlipState::Open);
        return output_;
    }
    if (input.inhibit_reason != TccDirectSlipReason::None) {
        const TccDirectSlipState state =
            (input.inhibit_reason == TccDirectSlipReason::ShiftActive ||
             input.inhibit_reason == TccDirectSlipReason::GearMismatch)
            ? TccDirectSlipState::ShiftInhibit : TccDirectSlipState::Open;
        open(input.inhibit_reason, state);
        return output_;
    }
    if (!input.request_control) {
        open(TccDirectSlipReason::DemandOpen, TccDirectSlipState::Open);
        return output_;
    }
    if (!input.speed_valid) {
        open(TccDirectSlipReason::InvalidSpeed, TccDirectSlipState::Open);
        return output_;
    }

    if (output_.state == TccDirectSlipState::RetryCooldown && input.now_ms < retry_until_ms_) {
        output_.reason = TccDirectSlipReason::TrackingTimeout;
        output_.pressure_mbar = 0;
        output_.nontracking_ms = retry_until_ms_ - input.now_ms;
        return output_;
    }

    if (!applying_) {
        applying_ = true;
        base_pressure_reached_ = false;
        nontracking_timer_active_ = false;
        feedback_trim_mbar_ = 0;
        tracking_cycles_ = 0;
        last_feedback_step_ms_ = input.now_ms;
        output_.pressure_mbar = 0;
        output_.tracking_achieved = false;
    }

    const int base_pressure_mbar = clamp_int(
        input.feedforward_pressure_mbar,
        TccDirectSlipCalibration::kMinimumHoldPressureMbar,
        TccDirectSlipCalibration::kMaximumFeedForwardPressureMbar
    );

    // Cross the hydraulic dead zone promptly, then stop at the holding floor.
    // Feedback is deliberately clocked no faster than the measured actuator
    // response so pressure cannot wind up while the clutch is still reacting.
    if (base_pressure_reached_ &&
        input.now_ms - last_feedback_step_ms_ >= TccDirectSlipCalibration::kFeedbackStepIntervalMs) {
        if (output_.slip_error_rpm > TccDirectSlipCalibration::kTrackingBandRpm) {
            feedback_trim_mbar_ = clamp_int(
                feedback_trim_mbar_ + TccDirectSlipCalibration::kFeedbackRiseStepMbar,
                0,
                TccDirectSlipCalibration::kMaxCommandPressureMbar - base_pressure_mbar
            );
        }
        last_feedback_step_ms_ = input.now_ms;
    }

    const int desired_pressure_mbar = clamp_int(
        base_pressure_mbar + feedback_trim_mbar_,
        TccDirectSlipCalibration::kMinimumHoldPressureMbar,
        TccDirectSlipCalibration::kMaxCommandPressureMbar
    );
    const int previous_pressure_mbar = output_.pressure_mbar;
    output_.pressure_mbar = move_toward(
        previous_pressure_mbar,
        desired_pressure_mbar,
        TccDirectSlipCalibration::kPressureRisePerCycleMbar,
        TccDirectSlipCalibration::kPressureFallPerCycleMbar
    );
    output_.pressure_delta_mbar = output_.pressure_mbar - previous_pressure_mbar;

    if (!base_pressure_reached_ && output_.pressure_mbar >= base_pressure_mbar) {
        base_pressure_reached_ = true;
        last_feedback_step_ms_ = input.now_ms;
    }

    const bool in_tracking_band =
        base_pressure_reached_ &&
        abs_int(output_.slip_error_rpm) <= TccDirectSlipCalibration::kTrackingBandRpm;
    if (in_tracking_band) {
        tracking_cycles_ += 1;
        nontracking_timer_active_ = false;
        output_.nontracking_ms = 0;
        if (tracking_cycles_ >= TccDirectSlipCalibration::kTrackingConfirmCycles) {
            output_.tracking_achieved = true;
            output_.state = TccDirectSlipState::Tracking;
        } else {
            output_.state = TccDirectSlipState::Applying;
        }
    } else {
        tracking_cycles_ = 0;
        output_.state = TccDirectSlipState::Applying;
        if (base_pressure_reached_ &&
            output_.slip_error_rpm >= TccDirectSlipCalibration::kFailureSlipRpm) {
            if (!nontracking_timer_active_) {
                nontracking_timer_active_ = true;
                nontracking_started_ms_ = input.now_ms;
            }
            output_.nontracking_ms = input.now_ms - nontracking_started_ms_;
        } else {
            nontracking_timer_active_ = false;
            output_.nontracking_ms = 0;
        }
    }
    output_.reason = TccDirectSlipReason::None;

    // A qualified failure cools down and retries. It never disables lockup for
    // the rest of the drive, and a zero-pressure coast cannot start this timer.
    if (nontracking_timer_active_ &&
        output_.nontracking_ms >= TccDirectSlipCalibration::kNonTrackingTimeoutMs) {
        start_retry_cooldown(input.now_ms);
    }
    return output_;
}
