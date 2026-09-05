#ifndef PERSISTENCE_SAFETY_H
#define PERSISTENCE_SAFETY_H

#include "canbus/can_hal.h"

inline bool is_engine_off(EgsBaseCan* can) {
    return can != nullptr && can->get_engine_rpm(250) == 0;
}

inline bool is_shifter_passive(EgsBaseCan* can) {
    if (can == nullptr) return false;
    const ShifterPosition pos = can->get_shifter_position(250);
    return pos == ShifterPosition::N || pos == ShifterPosition::P;
}

inline bool is_stationary_passive(EgsBaseCan* can) {
    // Unknown/stale values are UINT16_MAX, never evidence of standstill.
    return is_shifter_passive(can) &&
        can->get_rear_left_wheel(250) == 0 &&
        can->get_rear_right_wheel(250) == 0;
}

#endif
