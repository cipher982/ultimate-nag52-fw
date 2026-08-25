#include <cstdlib>
#include <iostream>

#include "tcc_direct_slip_controller.h"

namespace {
void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

TccDirectSlipInput enabled_input(uint32_t now_ms, int slip_rpm, int target_rpm = 20) {
    return {
        true,
        true,
        true,
        TccDirectSlipReason::None,
        slip_rpm,
        target_rpm,
        now_ms,
    };
}

void test_aggressively_crosses_measured_dead_zone() {
    TccDirectSlipController controller;
    auto out = controller.step(enabled_input(0, 300));
    require(out.state == TccDirectSlipState::Regulating, "controller should regulate immediately");
    require(out.pressure_mbar == 100, "first acquisition command should rise aggressively");

    for (uint32_t now = 20; now <= 600; now += 20) {
        out = controller.step(enabled_input(now, 300));
    }
    require(out.pressure_mbar >= 1500, "acquisition must cross the measured D2 dead zone");
    require(!out.fault_latched, "valid feedback ramp must not fault before the deadline");
}

void test_tracks_high_contact_plant_before_deadline() {
    TccDirectSlipController controller;
    int slip_rpm = 300;
    TccDirectSlipOutput out = {};
    uint32_t now = 0;
    for (; now < 4000 && !out.tracking_achieved && !out.fault_latched; now += 20) {
        out = controller.step(enabled_input(now, slip_rpm));
        // Static high-contact plant: no clutch effect below 1340 mbar.
        const int coupled_pressure = out.pressure_mbar > 1340 ? out.pressure_mbar - 1340 : 0;
        const int plant_slip = 300 - coupled_pressure;
        slip_rpm = plant_slip < 0 ? 0 : plant_slip;
    }
    require(out.tracking_achieved, "controller should track a 1340 mbar-contact plant");
    require(!out.fault_latched, "reachable high-contact plant must not fault");
    require(now < 4000, "tracking must occur before the absolute deadline");
    require(out.pressure_mbar > 1340, "tracking pressure should be discovered from slip feedback");
}

void test_unresponsive_plant_faults_open_without_retry() {
    TccDirectSlipController controller;
    TccDirectSlipOutput out = {};
    for (uint32_t now = 0; now <= 4100; now += 20) {
        out = controller.step(enabled_input(now, 300));
    }
    require(out.state == TccDirectSlipState::FaultOpen, "unresponsive plant should fault");
    require(out.reason == TccDirectSlipReason::TrackingTimeout, "timeout reason should be observable");
    require(out.pressure_mbar == 0, "fault output must be open");
    require(out.fault_latched, "fault must latch");

    out = controller.step(enabled_input(6000, 300));
    require(out.state == TccDirectSlipState::FaultOpen && out.pressure_mbar == 0,
        "latched fault must not retry itself");
}

void test_shift_and_mismatch_are_immediate_open_inhibits() {
    TccDirectSlipController controller;
    auto out = controller.step(enabled_input(0, 300));
    require(out.pressure_mbar == 100, "precondition: regulating");

    auto input = enabled_input(20, 300);
    input.inhibit_reason = TccDirectSlipReason::ShiftActive;
    out = controller.step(input);
    require(out.state == TccDirectSlipState::ShiftInhibit && out.pressure_mbar == 0,
        "shift activity should open immediately");

    input = enabled_input(40, 300);
    input.inhibit_reason = TccDirectSlipReason::GearMismatch;
    out = controller.step(input);
    require(out.state == TccDirectSlipState::ShiftInhibit && out.pressure_mbar == 0,
        "actual/target mismatch should open immediately");

    out = controller.step(enabled_input(60, 300));
    require(out.pressure_mbar == 100, "post-shift control should restart from zero");
}

void test_fault_only_clears_explicitly() {
    TccDirectSlipController controller;
    TccDirectSlipOutput out = {};
    for (uint32_t now = 0; now <= 4000; now += 20) {
        out = controller.step(enabled_input(now, 300));
    }
    auto not_selected = enabled_input(2100, 300);
    not_selected.selected = false;
    out = controller.step(not_selected);
    require(out.fault_latched && out.state == TccDirectSlipState::FaultOpen,
        "leaving D2 must not clear a latched fault");

    controller.clear_fault();
    out = controller.step(enabled_input(2200, 300));
    require(!out.fault_latched && out.pressure_mbar == 100,
        "explicit Park/key-cycle clear should permit a new attempt");
}

void test_signed_negative_error_reduces_pressure() {
    TccDirectSlipController controller;
    TccDirectSlipOutput out = {};
    for (uint32_t now = 0; now <= 400; now += 20) {
        out = controller.step(enabled_input(now, 300, 20));
    }
    const int before = out.pressure_mbar;
    out = controller.step(enabled_input(420, -100, 20));
    require(out.pressure_mbar < before, "negative signed slip error should release pressure");
}

void test_invalid_speed_opens_without_latching_and_restarts_from_zero() {
    TccDirectSlipController controller;
    auto out = controller.step(enabled_input(0, 300));
    require(out.pressure_mbar == 100, "precondition: pressure should be rising");

    auto invalid = enabled_input(20, 300);
    invalid.speed_valid = false;
    out = controller.step(invalid);
    require(out.state == TccDirectSlipState::Open && out.pressure_mbar == 0,
        "invalid speed must open immediately");
    require(out.reason == TccDirectSlipReason::InvalidSpeed && !out.fault_latched,
        "invalid speed must remain distinguishable from a tracking failure");

    out = controller.step(enabled_input(40, 300));
    require(out.pressure_mbar == 100 && !out.fault_latched,
        "fresh speed should begin a bounded attempt from zero");
}

void test_post_tracking_chatter_cannot_evade_loss_deadline() {
    TccDirectSlipController controller;
    TccDirectSlipOutput out = {};
    uint32_t now = 0;
    for (int cycle = 0; cycle < 100 && !out.tracking_achieved; ++cycle) {
        out = controller.step(enabled_input(now, 20));
        now += 20;
    }
    require(out.tracking_achieved, "precondition: initial tracking should be acquired");

    // Two in-band cycles are deliberately insufficient to reset the loss
    // timer; alternate them with one out-of-band cycle past the 4 s deadline.
    for (int cycle = 0; cycle < 220 && !out.fault_latched; ++cycle) {
        const bool out_of_band = cycle % 3 == 0;
        out = controller.step(enabled_input(now, out_of_band ? 80 : 20));
        now += 20;
    }
    require(out.fault_latched && out.reason == TccDirectSlipReason::TrackingTimeout,
        "sub-confirmation chatter must not evade the loss-of-tracking deadline");
    require(out.pressure_mbar == 0, "loss-of-tracking timeout must fault open");
}

void test_reachable_delayed_plants_track_before_deadline() {
    struct Scenario {
        int contact_mbar;
        int free_slip_rpm;
        double rpm_per_mbar;
        int hydraulic_tau_cycles;
        int slip_tau_cycles;
        int target_rpm;
    };
    const Scenario scenarios[] = {
        {900, 250, 1.0, 3, 3, 20},
        {1200, 300, 1.0, 5, 4, 20},
        {1340, 300, 1.0, 5, 4, 20},
        {1450, 350, 1.2, 4, 3, 50},
        {1500, 300, 1.5, 3, 3, 90},
    };
    for (const Scenario& scenario : scenarios) {
        TccDirectSlipController controller;
        TccDirectSlipOutput out = {};
        double applied_pressure = 0;
        double slip = scenario.free_slip_rpm;
        for (uint32_t now = 0; now < 4200 && !out.tracking_achieved && !out.fault_latched;
             now += 20) {
            out = controller.step(enabled_input(now, static_cast<int>(slip), scenario.target_rpm));
            applied_pressure += (out.pressure_mbar - applied_pressure) /
                scenario.hydraulic_tau_cycles;
            const double coupling = applied_pressure > scenario.contact_mbar
                ? (applied_pressure - scenario.contact_mbar) * scenario.rpm_per_mbar : 0;
            const double equilibrium_slip = scenario.free_slip_rpm - coupling;
            slip += (equilibrium_slip - slip) / scenario.slip_tau_cycles;
        }
        if (!out.tracking_achieved) {
            std::cerr << "scenario contact=" << scenario.contact_mbar
                      << " free_slip=" << scenario.free_slip_rpm
                      << " target=" << scenario.target_rpm
                      << " command=" << out.pressure_mbar
                      << " slip=" << static_cast<int>(slip) << '\n';
        }
        require(out.tracking_achieved, "reachable delayed plant should track before deadline");
        require(!out.fault_latched, "reachable delayed plant must not fault");
    }
}
}

int main() {
    test_aggressively_crosses_measured_dead_zone();
    test_tracks_high_contact_plant_before_deadline();
    test_unresponsive_plant_faults_open_without_retry();
    test_shift_and_mismatch_are_immediate_open_inhibits();
    test_fault_only_clears_explicitly();
    test_signed_negative_error_reduces_pressure();
    test_invalid_speed_opens_without_latching_and_restarts_from_zero();
    test_post_tracking_chatter_cannot_evade_loss_deadline();
    test_reachable_delayed_plants_track_before_deadline();
    std::cout << "TCC direct-slip host tests passed\n";
    return 0;
}
