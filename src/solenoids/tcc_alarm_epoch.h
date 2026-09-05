#ifndef TCC_ALARM_EPOCH_H
#define TCC_ALARM_EPOCH_H

#include <stdint.h>

// Access only under the callback gate, or while that gate is closed. Methods
// inline into IRAM callers; no allocation, SDK call or cached data dependency.
struct TccAlarmEpoch {
    uint64_t expected = 0;

    __attribute__((always_inline)) bool is_due(uint64_t alarm, uint64_t count) const {
        return alarm == expected && count >= expected;
    }

    __attribute__((always_inline)) bool advance(uint64_t delay) {
        if (delay == 0 || expected > UINT64_MAX - delay) return false;
        expected += delay;
        return true;
    }

    __attribute__((always_inline)) bool restart_after(uint64_t stopped_count, uint64_t delay) {
        const uint64_t base = stopped_count > expected ? stopped_count : expected;
        if (delay == 0 || base > UINT64_MAX - delay) return false;
        expected = base + delay;
        return true;
    }
};

#endif
