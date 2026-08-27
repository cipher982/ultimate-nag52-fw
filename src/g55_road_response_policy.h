#ifndef G55_ROAD_RESPONSE_POLICY_H
#define G55_ROAD_RESPONSE_POLICY_H

#include <stdint.h>

namespace G55RoadResponsePolicy {

constexpr uint8_t kLoadedPedalMinimumRaw = 80;
constexpr int16_t kLoadedInputTorqueMinimumNm = 200;
constexpr int16_t kLoadedInputTorqueMaximumNm = 500;
constexpr uint16_t kD5DownshiftFloorMinimumRpm = 1500;
constexpr uint16_t kD5DownshiftFloorMaximumRpm = 1800;

constexpr int16_t kRelease43InputTorqueMinimumNm = 100;
constexpr uint8_t kRelease43PostOverlapHoldCycles = 5;

inline uint16_t d5_downshift_threshold_rpm(
    uint16_t stored_map_threshold_rpm,
    uint8_t pedal_raw,
    int16_t input_torque_nm
) {
    if (pedal_raw < kLoadedPedalMinimumRaw ||
        input_torque_nm < kLoadedInputTorqueMinimumNm) {
        return stored_map_threshold_rpm;
    }

    int32_t bounded_torque_nm = input_torque_nm;
    if (bounded_torque_nm > kLoadedInputTorqueMaximumNm) {
        bounded_torque_nm = kLoadedInputTorqueMaximumNm;
    }
    const int32_t torque_range_nm =
        kLoadedInputTorqueMaximumNm - kLoadedInputTorqueMinimumNm;
    const int32_t rpm_range =
        kD5DownshiftFloorMaximumRpm - kD5DownshiftFloorMinimumRpm;
    const uint16_t load_floor_rpm = kD5DownshiftFloorMinimumRpm +
        ((bounded_torque_nm - kLoadedInputTorqueMinimumNm) * rpm_range) /
            torque_range_nm;

    return stored_map_threshold_rpm > load_floor_rpm
        ? stored_map_threshold_rpm : load_floor_rpm;
}

inline bool should_downshift_from_d5(
    uint16_t input_rpm,
    uint16_t stored_map_threshold_rpm,
    uint8_t pedal_raw,
    int16_t input_torque_nm
) {
    return input_rpm < d5_downshift_threshold_rpm(
        stored_map_threshold_rpm,
        pedal_raw,
        input_torque_nm
    );
}

inline uint8_t release_4_3_post_overlap_hold_cycles(
    bool is_release_4_3,
    bool is_coast_shift,
    int16_t input_torque_nm
) {
    return is_release_4_3 && !is_coast_shift &&
        input_torque_nm >= kRelease43InputTorqueMinimumNm
        ? kRelease43PostOverlapHoldCycles : 0;
}

inline bool consume_post_overlap_hold_cycle(uint8_t* remaining_cycles) {
    if (*remaining_cycles == 0) {
        return false;
    }
    *remaining_cycles -= 1;
    return *remaining_cycles == 0;
}

} // namespace G55RoadResponsePolicy

#endif
