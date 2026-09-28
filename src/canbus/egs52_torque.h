#pragma once

#include <climits>
#include <cstdint>

// The all-ones 13-bit value means unavailable, not 1547 Nm.
constexpr int16_t decode_egs52_torque(uint16_t raw) {
    return raw == 0x1FFF ? INT16_MAX : static_cast<int16_t>(raw / 4 - 500);
}
