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
    ) == 1520);
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

void test_d5_floor_requires_both_pedal_and_measured_torque() {
    assert(G55RoadResponsePolicy::d5_downshift_threshold_rpm(
        1300, 79, 300
    ) == 1300);
    assert(G55RoadResponsePolicy::d5_downshift_threshold_rpm(
        1300, 100, 199
    ) == 1300);
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

void test_release_4_3_handback_starts_after_five_complete_cycles() {
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

} // namespace

int main() {
    test_d5_gentle_cruise_preserves_stored_map();
    test_d5_measured_moderate_load_downshifts();
    test_d5_floor_is_bounded_and_map_keeps_authority();
    test_d5_floor_requires_both_pedal_and_measured_torque();
    test_release_4_3_hold_is_narrowly_scoped();
    test_release_4_3_handback_starts_after_five_complete_cycles();
    std::cout << "G55 road-response policy tests passed\n";
    return 0;
}
