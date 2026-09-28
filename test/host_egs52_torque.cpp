#include "canbus/egs52_torque.h"

#include <cassert>
#include <cstdint>

int main() {
    // Valid 13-bit fields retain the firmware's existing integer Nm conversion.
    assert(decode_egs52_torque(0) == -500);
    assert(decode_egs52_torque(2000) == 0);
    assert(decode_egs52_torque(2115) == 28);
    assert(decode_egs52_torque(3682) == 420);
    assert(decode_egs52_torque(8190) == 1547);

    // On every new frame an unavailable field must remain unavailable, not
    // enter the maximum-torque correction or driver/static clamp as 1547 Nm.
    const uint16_t sequence[] = {2115, 0x1FFF, 2107};
    const int16_t expected[] = {28, INT16_MAX, 26};
    for (unsigned i = 0; i < 3; ++i) {
        assert(decode_egs52_torque(sequence[i]) == expected[i]);
    }
    assert(decode_egs52_torque(0x1FFF) == INT16_MAX);
}
