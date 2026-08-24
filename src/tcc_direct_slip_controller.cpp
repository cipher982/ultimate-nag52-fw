#include "tcc_direct_slip_controller.h"

namespace {
int clamp_int(int value, int low, int high) {
    return value < low ? low : (value > high ? high : value);
}

int abs_int(int value) {
    return value < 0 ? -value : value;
}
}

void TccDirectSlipController::open(TccDirectSlipReason reason, TccDirectSlipState state) {
    output_.state = state;
    output_.reason = reason;
    output_.pressure_mbar = 0;
    output_.pressure_delta_mbar = 0;
    output_.nontracking_ms = 0;
    output_.tracking_achieved = false;
    applying_ = false;
    out_of_band_timer_active_ = false;
    tracking_cycles_ = 0;
}

void TccDirectSlipController::latch_fault(TccDirectSlipReason reason) {
    open(reason, TccDirectSlipState::FaultOpen);
    output_.fault_latched = true;
}

void TccDirectSlipController::clear_fault() {
    output_.fault_latched = false;
    open(TccDirectSlipReason::NotSelected, TccDirectSlipState::Open);
}

void TccDirectSlipController::reset_nonfault_state() {
    if (!output_.fault_latched) {
        open(TccDirectSlipReason::NotSelected, TccDirectSlipState::Open);
    }
}

TccDirectSlipOutput TccDirectSlipController::step(const TccDirectSlipInput& input) {
    output_.signed_slip_rpm = clamp_int(input.signed_slip_rpm, -10000, 10000);
    output_.target_slip_rpm = clamp_int(input.target_slip_rpm, 0, 10000);
    output_.slip_error_rpm = output_.signed_slip_rpm - output_.target_slip_rpm;
    output_.pressure_delta_mbar = 0;

    // A timeout is a latched failed experiment, not a reason to retry while
    // driving. Park/key-cycle must explicitly clear it.
    if (output_.fault_latched) {
        output_.state = TccDirectSlipState::FaultOpen;
        output_.pressure_mbar = 0;
        return output_;
    }

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
        // Missing feedback cannot be controlled safely, but it is not evidence
        // that the clutch failed to track. Open immediately and let fresh data
        // begin a new attempt from zero; only a real tracking timeout latches.
        open(TccDirectSlipReason::InvalidSpeed, TccDirectSlipState::Open);
        return output_;
    }

    if (!applying_) {
        applying_ = true;
        application_started_ms_ = input.now_ms;
        out_of_band_timer_active_ = false;
        tracking_cycles_ = 0;
        output_.pressure_mbar = 0;
        output_.tracking_achieved = false;
    }

    int pressure_delta = output_.slip_error_rpm /
        TccDirectSlipCalibration::kSlipErrorDivisor;
    pressure_delta = clamp_int(
        pressure_delta,
        -TccDirectSlipCalibration::kPressureFallPerCycleMbar,
        TccDirectSlipCalibration::kPressureRisePerCycleMbar
    );
    const int previous_pressure = output_.pressure_mbar;
    output_.pressure_mbar = clamp_int(
        previous_pressure + pressure_delta,
        0,
        TccDirectSlipCalibration::kMaxCommandPressureMbar
    );
    output_.pressure_delta_mbar = output_.pressure_mbar - previous_pressure;

    const bool in_tracking_band =
        abs_int(output_.slip_error_rpm) <= TccDirectSlipCalibration::kTrackingBandRpm;
    if (in_tracking_band) {
        tracking_cycles_ += 1;
        output_.nontracking_ms = output_.tracking_achieved && out_of_band_timer_active_
            ? input.now_ms - out_of_band_started_ms_
            : (output_.tracking_achieved ? 0 : input.now_ms - application_started_ms_);
        if (tracking_cycles_ >= TccDirectSlipCalibration::kTrackingConfirmCycles) {
            output_.tracking_achieved = true;
            out_of_band_timer_active_ = false;
            output_.state = TccDirectSlipState::Tracking;
            output_.reason = TccDirectSlipReason::None;
            output_.nontracking_ms = 0;
        } else {
            output_.state = TccDirectSlipState::Regulating;
            output_.reason = TccDirectSlipReason::None;
        }
    } else {
        tracking_cycles_ = 0;
        output_.state = TccDirectSlipState::Regulating;
        output_.reason = TccDirectSlipReason::None;
        if (!output_.tracking_achieved) {
            output_.nontracking_ms = input.now_ms - application_started_ms_;
        } else {
            if (!out_of_band_timer_active_) {
                out_of_band_timer_active_ = true;
                out_of_band_started_ms_ = input.now_ms;
            }
            output_.nontracking_ms = input.now_ms - out_of_band_started_ms_;
        }
    }

    // First acquisition gets one absolute deadline; a single noisy in-band
    // sample cannot restart it. After acquisition, the loss timer resets only
    // after the same five-cycle tracking confirmation, so chatter cannot evade
    // the deadline one sample at a time.
    if (output_.nontracking_ms >= TccDirectSlipCalibration::kNonTrackingTimeoutMs) {
        latch_fault(TccDirectSlipReason::TrackingTimeout);
    }
    return output_;
}
