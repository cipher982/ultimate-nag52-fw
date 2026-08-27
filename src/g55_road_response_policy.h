#ifndef G55_ROAD_RESPONSE_POLICY_H
#define G55_ROAD_RESPONSE_POLICY_H

#include <stdint.h>

namespace G55RoadResponsePolicy {

constexpr uint8_t kBlendPedalStartRaw = 64;
constexpr uint8_t kBlendPedalFullRaw = 96;
constexpr int16_t kBlendInputTorqueStartNm = 150;
constexpr int16_t kBlendInputTorqueFullNm = 300;
constexpr int16_t kFloorInputTorqueMinimumNm = 200;
constexpr int16_t kLoadedInputTorqueMaximumNm = 500;
constexpr uint16_t kD5DownshiftFloorMinimumRpm = 1500;
constexpr uint16_t kD5DownshiftFloorMaximumRpm = 1800;
constexpr uint16_t kD5UpshiftMarginRpm = 150;
constexpr uint8_t kD5DownshiftQualificationCycles = 10;

constexpr int16_t kRelease43InputTorqueMinimumNm = 100;
constexpr uint8_t kRelease43HandbackDelayCycles = 5;

// The V10 road trace shows ME torque still recovering after physical ratio
// synchronization on loaded D2-D5 upshifts.  Begin a six-cycle handback over
// the final 500 RPM of oncoming-clutch synchronization so the existing torque
// reduction reaches zero before, rather than after, the driver-felt endpoint.
// D1->D2 is excluded because its converter is intentionally open and the same
// trace shows no post-sync torque deficit there.
constexpr uint8_t kLoadedUpshiftPedalMinimumRaw = 96;
constexpr int16_t kLoadedUpshiftInputTorqueMinimumNm = 300;
constexpr int16_t kLoadedUpshiftHandbackStartRpm = 500;
constexpr uint8_t kLoadedUpshiftHandbackCycles = 6;

inline int32_t blend_factor_per_mille(
    int32_t value,
    int32_t start,
    int32_t full
) {
    if (value <= start) {
        return 0;
    }
    if (value >= full) {
        return 1000;
    }
    return ((value - start) * 1000) / (full - start);
}

inline uint16_t d5_downshift_threshold_rpm(
    uint16_t stored_map_threshold_rpm,
    uint8_t pedal_raw,
    int16_t input_torque_nm
) {
    const int32_t pedal_blend = blend_factor_per_mille(
        pedal_raw,
        kBlendPedalStartRaw,
        kBlendPedalFullRaw
    );
    const int32_t torque_blend = blend_factor_per_mille(
        input_torque_nm,
        kBlendInputTorqueStartNm,
        kBlendInputTorqueFullNm
    );
    const int32_t blend = pedal_blend < torque_blend
        ? pedal_blend : torque_blend;
    if (blend == 0) {
        return stored_map_threshold_rpm;
    }

    int32_t bounded_torque_nm = input_torque_nm;
    if (bounded_torque_nm < kFloorInputTorqueMinimumNm) {
        bounded_torque_nm = kFloorInputTorqueMinimumNm;
    }
    if (bounded_torque_nm > kLoadedInputTorqueMaximumNm) {
        bounded_torque_nm = kLoadedInputTorqueMaximumNm;
    }
    const int32_t torque_range_nm =
        kLoadedInputTorqueMaximumNm - kFloorInputTorqueMinimumNm;
    const int32_t rpm_range =
        kD5DownshiftFloorMaximumRpm - kD5DownshiftFloorMinimumRpm;
    const uint16_t load_floor_rpm = kD5DownshiftFloorMinimumRpm +
        ((bounded_torque_nm - kFloorInputTorqueMinimumNm) * rpm_range) /
            torque_range_nm;
    if (stored_map_threshold_rpm >= load_floor_rpm) {
        return stored_map_threshold_rpm;
    }
    return stored_map_threshold_rpm +
        ((load_floor_rpm - stored_map_threshold_rpm) * blend) / 1000;
}

inline uint16_t predicted_d5_input_rpm(
    uint16_t output_rpm,
    uint16_t fifth_gear_ratio_x1000
) {
    return (static_cast<uint32_t>(output_rpm) * fifth_gear_ratio_x1000) /
        1000;
}

inline bool d5_upshift_is_compatible(
    uint16_t predicted_input_rpm,
    uint16_t d5_downshift_threshold_rpm
) {
    return static_cast<uint32_t>(predicted_input_rpm) >=
        static_cast<uint32_t>(d5_downshift_threshold_rpm) +
            kD5UpshiftMarginRpm;
}

class D5DownshiftQualifier {
public:
    bool step(bool additional_downshift_request) {
        if (!additional_downshift_request) {
            qualifying_cycles_ = 0;
            return false;
        }
        if (qualifying_cycles_ < kD5DownshiftQualificationCycles) {
            qualifying_cycles_ += 1;
        }
        return qualifying_cycles_ >= kD5DownshiftQualificationCycles;
    }

    void reset() {
        qualifying_cycles_ = 0;
    }

    uint8_t qualifying_cycles() const {
        return qualifying_cycles_;
    }

private:
    uint8_t qualifying_cycles_ = 0;
};

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

inline bool d5_downshift_decision(
    uint16_t input_rpm,
    uint16_t stored_map_threshold_rpm,
    uint8_t pedal_raw,
    int16_t input_torque_nm,
    D5DownshiftQualifier* qualifier
) {
    if (input_rpm < stored_map_threshold_rpm) {
        qualifier->reset();
        return true;
    }
    return qualifier->step(should_downshift_from_d5(
        input_rpm,
        stored_map_threshold_rpm,
        pedal_raw,
        input_torque_nm
    ));
}

inline bool four_to_five_is_allowed_by_d5_policy(
    uint16_t output_rpm,
    uint16_t fifth_gear_ratio_x1000,
    uint16_t d5_stored_map_threshold_rpm,
    uint8_t pedal_raw,
    int16_t input_torque_nm
) {
    const uint16_t predicted_input_rpm = predicted_d5_input_rpm(
        output_rpm,
        fifth_gear_ratio_x1000
    );
    const uint16_t downshift_threshold_rpm = d5_downshift_threshold_rpm(
        d5_stored_map_threshold_rpm,
        pedal_raw,
        input_torque_nm
    );
    return d5_upshift_is_compatible(
        predicted_input_rpm,
        downshift_threshold_rpm
    );
}

inline uint8_t release_4_3_post_overlap_hold_cycles(
    bool is_release_4_3,
    bool is_coast_shift,
    int16_t input_torque_nm
) {
    return is_release_4_3 && !is_coast_shift &&
        input_torque_nm >= kRelease43InputTorqueMinimumNm
        ? kRelease43HandbackDelayCycles : 0;
}

inline bool consume_post_overlap_hold_cycle(uint8_t* remaining_cycles) {
    if (*remaining_cycles == 0) {
        return false;
    }
    *remaining_cycles -= 1;
    return *remaining_cycles == 0;
}

inline uint8_t loaded_upshift_torque_handback_cycles(
    bool is_upshift,
    bool is_first_to_second,
    bool torque_reduction_active,
    uint8_t pedal_raw,
    int16_t input_torque_nm,
    int16_t on_clutch_speed_rpm,
    uint8_t overlap_subphase
) {
    const bool in_final_inertia_window = overlap_subphase >= 2 &&
        on_clutch_speed_rpm > 0 &&
        on_clutch_speed_rpm <= kLoadedUpshiftHandbackStartRpm;
    return is_upshift && !is_first_to_second && torque_reduction_active &&
        pedal_raw >= kLoadedUpshiftPedalMinimumRaw &&
        input_torque_nm >= kLoadedUpshiftInputTorqueMinimumNm &&
        in_final_inertia_window
        ? kLoadedUpshiftHandbackCycles : 0;
}

inline void initialize_torque_handback_timer_if_inactive(
    bool handback_active,
    uint8_t initial_cycles,
    uint8_t* timer_cycles
) {
    if (!handback_active) {
        *timer_cycles = initial_cycles;
    }
}

} // namespace G55RoadResponsePolicy

#endif
