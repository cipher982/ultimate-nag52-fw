#ifndef TCC_DIRECT_SLIP_CONTROLLER_H
#define TCC_DIRECT_SLIP_CONTROLLER_H

#include <stdint.h>

// V6 deliberately crosses the measured D2 hydraulic dead zone quickly, then
// lets signed slip feedback control pressure. Reaching acquisition pressure
// only proves hydraulic authority; tracking still requires measured slip.
namespace TccDirectSlipCalibration {
constexpr int kCycleMs = 20;
constexpr int kAcquisitionPressureMbar = 1500;
constexpr int kAcquisitionRisePerCycleMbar = 100;
constexpr int kPressureRisePerCycleMbar = 60;
constexpr int kPressureFallPerCycleMbar = 100;
constexpr int kSlipErrorDivisor = 8;
constexpr int kMaxCommandPressureMbar = 3000;
constexpr int kTrackingBandRpm = 50;
constexpr int kTrackingConfirmCycles = 3;
constexpr uint32_t kNonTrackingTimeoutMs = 4000;
}

enum class TccDirectSlipState : uint8_t {
    Open = 0,
    ShiftInhibit = 1,
    Regulating = 2,
    Tracking = 3,
    FaultOpen = 4,
};

enum class TccDirectSlipReason : uint8_t {
    None = 0,
    NotSelected = 1,
    DemandOpen = 2,
    ShiftActive = 3,
    GearMismatch = 4,
    InvalidSpeed = 5,
    NonPositiveTorque = 6,
    TrackingTimeout = 7,
    ControllerDisabled = 8,
};

struct TccDirectSlipInput {
    bool selected;
    bool request_control;
    bool speed_valid;
    TccDirectSlipReason inhibit_reason;
    int signed_slip_rpm;
    int target_slip_rpm;
    uint32_t now_ms;
};

struct TccDirectSlipOutput {
    TccDirectSlipState state;
    TccDirectSlipReason reason;
    int pressure_mbar;
    int pressure_delta_mbar;
    int signed_slip_rpm;
    int slip_error_rpm;
    int target_slip_rpm;
    uint32_t nontracking_ms;
    bool tracking_achieved;
    bool fault_latched;
};

class TccDirectSlipController {
public:
    TccDirectSlipOutput step(const TccDirectSlipInput& input);
    void clear_fault();
    void reset_nonfault_state();
    TccDirectSlipOutput snapshot() const { return output_; }

private:
    void open(TccDirectSlipReason reason, TccDirectSlipState state);
    void latch_fault(TccDirectSlipReason reason);

    TccDirectSlipOutput output_ = {
        TccDirectSlipState::Open,
        TccDirectSlipReason::NotSelected,
        0, 0, 0, 0, 100, 0, false, false
    };
    uint32_t application_started_ms_ = 0;
    uint32_t out_of_band_started_ms_ = 0;
    int tracking_cycles_ = 0;
    bool applying_ = false;
    bool acquisition_pressure_reached_ = false;
    bool out_of_band_timer_active_ = false;
};

#endif
