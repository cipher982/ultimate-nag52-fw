#include "g55_road_response_policy.h"

#include <cassert>
#include <iostream>

namespace {

void test_d5_gentle_cruise_preserves_stored_map() {
    assert(G55RoadResponsePolicy::d5_downshift_threshold_rpm(
        1250, 56, 92
    ) == 1250);
    assert(!G55RoadResponsePolicy::should_downshift_from_d5(
        1360, 1250, 56, 92
    ));
}

void test_d5_measured_moderate_load_downshifts() {
    assert(G55RoadResponsePolicy::d5_downshift_threshold_rpm(
        1360, 90, 220
    ) == 1434);
    assert(G55RoadResponsePolicy::should_downshift_from_d5(
        1320, 1360, 90, 220
    ));

    assert(G55RoadResponsePolicy::d5_downshift_threshold_rpm(
        1430, 108, 300
    ) == 1600);
    assert(G55RoadResponsePolicy::should_downshift_from_d5(
        1500, 1430, 108, 300
    ));
}

void test_d5_floor_is_bounded_and_map_keeps_authority() {
    assert(G55RoadResponsePolicy::d5_downshift_threshold_rpm(
        1400, 250, 900
    ) == 1800);
    assert(!G55RoadResponsePolicy::should_downshift_from_d5(
        1800, 1400, 250, 900
    ));
    assert(G55RoadResponsePolicy::d5_downshift_threshold_rpm(
        2400, 150, 400
    ) == 2400);
}

void test_d5_floor_blends_continuously_at_boundaries() {
    assert(G55RoadResponsePolicy::d5_downshift_threshold_rpm(
        1300, 64, 300
    ) == 1300);
    assert(G55RoadResponsePolicy::d5_downshift_threshold_rpm(
        1300, 100, 150
    ) == 1300);

    const uint16_t first_pedal_step =
        G55RoadResponsePolicy::d5_downshift_threshold_rpm(
            1300, 65, 300
        );
    const uint16_t first_torque_step =
        G55RoadResponsePolicy::d5_downshift_threshold_rpm(
            1300, 100, 151
        );
    assert(first_pedal_step >= 1300 && first_pedal_step <= 1310);
    assert(first_torque_step >= 1300 && first_torque_step <= 1310);
}

void test_d5_additional_request_requires_200_ms() {
    G55RoadResponsePolicy::D5DownshiftQualifier qualifier;
    for (int cycle = 1;
        cycle < G55RoadResponsePolicy::kD5DownshiftQualificationCycles;
        ++cycle) {
        assert(!qualifier.step(true));
        assert(qualifier.qualifying_cycles() == cycle);
    }
    assert(qualifier.step(true));
    assert(qualifier.qualifying_cycles() == 10);
    assert(qualifier.step(true));
    assert(!qualifier.step(false));
    assert(qualifier.qualifying_cycles() == 0);
}

void test_d5_stored_map_request_remains_immediate() {
    G55RoadResponsePolicy::D5DownshiftQualifier qualifier;
    assert(G55RoadResponsePolicy::d5_downshift_decision(
        1199, 1200, 56, 92, &qualifier
    ));
    assert(qualifier.qualifying_cycles() == 0);
}

void test_logged_d5_loaded_run_qualifies_on_tenth_sample() {
    // Consecutive 20 ms samples beginning at 23:31:30.533 in road-2.
    const uint16_t input_rpm[] = {
        1279, 1284, 1287, 1289, 1281, 1284, 1287, 1284, 1287, 1289
    };
    G55RoadResponsePolicy::D5DownshiftQualifier qualifier;
    for (int index = 0; index < 10; ++index) {
        const bool request = G55RoadResponsePolicy::d5_downshift_decision(
            input_rpm[index],
            1248,
            87,
            index == 0 ? 209 : 210,
            &qualifier
        );
        assert(request == (index == 9));
    }
}

void test_noisy_additional_request_does_not_qualify() {
    G55RoadResponsePolicy::D5DownshiftQualifier qualifier;
    for (int cycle = 0; cycle < 30; ++cycle) {
        assert(!qualifier.step((cycle % 2) == 0));
    }
}

void test_d5_upshift_gate_prevents_immediate_reversal() {
    assert(G55RoadResponsePolicy::predicted_d5_input_rpm(
        1930, 820
    ) == 1582);
    assert(!G55RoadResponsePolicy::d5_upshift_is_compatible(
        1600, 1500
    ));
    assert(G55RoadResponsePolicy::d5_upshift_is_compatible(
        1650, 1500
    ));
    assert(!G55RoadResponsePolicy::four_to_five_is_allowed_by_d5_policy(
        1930, 820, 1500, 100, 300
    ));
}

void test_release_4_3_hold_is_narrowly_scoped() {
    assert(G55RoadResponsePolicy::release_4_3_post_overlap_hold_cycles(
        true, false, 305
    ) == 5);
    assert(G55RoadResponsePolicy::release_4_3_post_overlap_hold_cycles(
        false, false, 305
    ) == 0);
    assert(G55RoadResponsePolicy::release_4_3_post_overlap_hold_cycles(
        true, true, 305
    ) == 0);
    assert(G55RoadResponsePolicy::release_4_3_post_overlap_hold_cycles(
        true, false, 99
    ) == 0);
}

void test_release_4_3_handback_starts_100_ms_after_baseline() {
    uint8_t remaining = 5;
    for (int cycle = 1; cycle < 5; ++cycle) {
        assert(!G55RoadResponsePolicy::consume_post_overlap_hold_cycle(
            &remaining
        ));
        assert(remaining == 5 - cycle);
    }
    assert(G55RoadResponsePolicy::consume_post_overlap_hold_cycle(
        &remaining
    ));
    assert(remaining == 0);
    assert(!G55RoadResponsePolicy::consume_post_overlap_hold_cycle(
        &remaining
    ));
}

void test_loaded_upshift_handback_matches_measured_2_3_window() {
    // V10 22:23:47.922-.940: the clean loaded 2->3 had 568 RPM left,
    // then crossed 476 RPM with pedal 145 and about 466 Nm input torque.
    assert(G55RoadResponsePolicy::loaded_upshift_torque_handback_cycles(
        true, false, true, 145, 466, 568, 2
    ) == 0);
    assert(G55RoadResponsePolicy::loaded_upshift_torque_handback_cycles(
        true, false, true, 145, 466, 476, 2
    ) == 6);
    assert(G55RoadResponsePolicy::loaded_upshift_torque_handback_cycles(
        true, false, true, 96, 300, 500, 2
    ) == 6);
    assert(G55RoadResponsePolicy::loaded_upshift_torque_handback_cycles(
        true, false, true, 96, 300, 501, 2
    ) == 0);
}

void test_loaded_upshift_handback_preserves_gentle_and_d1_d2_behavior() {
    assert(G55RoadResponsePolicy::loaded_upshift_torque_handback_cycles(
        true, false, true, 90, 234, 476, 2
    ) == 0);
    assert(G55RoadResponsePolicy::loaded_upshift_torque_handback_cycles(
        true, true, true, 145, 466, 476, 2
    ) == 0);
    assert(G55RoadResponsePolicy::loaded_upshift_torque_handback_cycles(
        false, false, true, 145, 466, 476, 2
    ) == 0);
}

void test_loaded_upshift_handback_requires_active_reduction_and_inertia_phase() {
    assert(G55RoadResponsePolicy::loaded_upshift_torque_handback_cycles(
        true, false, false, 145, 466, 476, 2
    ) == 0);
    assert(G55RoadResponsePolicy::loaded_upshift_torque_handback_cycles(
        true, false, true, 145, 466, 476, 1
    ) == 0);
    assert(G55RoadResponsePolicy::loaded_upshift_torque_handback_cycles(
        true, false, true, 145, 466, 0, 3
    ) == 0);
}

void test_subphase_transition_does_not_restart_active_handback() {
    uint8_t timer = 2;
    G55RoadResponsePolicy::initialize_torque_handback_timer_if_inactive(
        true, 6, &timer
    );
    assert(timer == 2);

    G55RoadResponsePolicy::initialize_torque_handback_timer_if_inactive(
        false, 6, &timer
    );
    assert(timer == 6);
}

} // namespace

int main() {
    test_d5_gentle_cruise_preserves_stored_map();
    test_d5_measured_moderate_load_downshifts();
    test_d5_floor_is_bounded_and_map_keeps_authority();
    test_d5_floor_blends_continuously_at_boundaries();
    test_d5_additional_request_requires_200_ms();
    test_d5_stored_map_request_remains_immediate();
    test_logged_d5_loaded_run_qualifies_on_tenth_sample();
    test_noisy_additional_request_does_not_qualify();
    test_d5_upshift_gate_prevents_immediate_reversal();
    test_release_4_3_hold_is_narrowly_scoped();
    test_release_4_3_handback_starts_100_ms_after_baseline();
    test_loaded_upshift_handback_matches_measured_2_3_window();
    test_loaded_upshift_handback_preserves_gentle_and_d1_d2_behavior();
    test_loaded_upshift_handback_requires_active_reduction_and_inertia_phase();
    test_subphase_transition_does_not_restart_active_handback();
    std::cout << "G55 road-response policy tests passed\n";
    return 0;
}
