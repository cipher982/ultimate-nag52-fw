#!/usr/bin/env python3
"""Compile real task guard/device-mode code against host-only hardware adapters.

These checks do not emulate GPTimer, cache stalls or loaded actuators. Temporary
headers/binary are deleted by TemporaryDirectory, including on compile failure.
"""
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

REPO = Path(__file__).resolve().parents[1]
HEADERS = {
    "esp_err.h": r'''
#pragma once
#include <cstdint>
#ifndef BIT
#define BIT(n) (1U << (n))
#endif
using esp_err_t = int;
constexpr int ESP_OK = 0, ESP_FAIL = -1, ESP_ERR_INVALID_STATE = 0x103, ESP_ERR_TIMEOUT = 0x107;
constexpr int ESP_ERR_INVALID_SIZE = 0x104, ESP_ERR_INVALID_ARG = 0x102, ESP_ERR_NO_MEM = 0x101;
constexpr int ESP_ERR_NVS_INVALID_LENGTH = 0x110C;
inline const char* esp_err_to_name(int) { return "host error"; }
''',
    "esp_log.h": '#pragma once\n#define ESP_LOGE(...) ((void)0)\n#define ESP_LOGW(...) ((void)0)\n#define ESP_LOGI(...) ((void)0)\n',
    "common_structs.h": '#pragma once\n#include <cstdint>\n',
    "nvs_flash.h": '#pragma once\n',
    "nvs.h": r'''
#pragma once
#include <cstdint>
#include "esp_err.h"
using nvs_handle_t = unsigned;
enum nvs_open_mode_t { NVS_READONLY, NVS_READWRITE };
inline int active_handles = 0;
inline int open_error = ESP_OK;
inline int nvs_open(const char*, nvs_open_mode_t, nvs_handle_t* handle) {
    if (open_error != ESP_OK) return open_error;
    *handle = ++active_handles;
    return ESP_OK;
}
inline void nvs_close(nvs_handle_t) { --active_handles; }
''',
    "canbus/can_hal.h": r'''
#pragma once
#include <cstdint>
#include <cassert>
#include "freertos/semphr.h"
enum class ShifterPosition : uint8_t { P, P_R, R, R_N, N, N_D, D, PLUS, MINUS, FOUR, THREE, TWO, ONE, SignalNotAvailable = 255 };
struct twai_message_t { uint32_t identifier = 0; uint8_t data_length_code = 8; uint8_t data[8] = {}; };
using QueueHandle_t = int;
inline unsigned diagnostic_frames = 0;
inline int xQueueSend(QueueHandle_t, const void*, int) { ++diagnostic_frames; return 1; }
struct SlaveDecoder {
    unsigned imports = 0;
    void import_frames(uint64_t, uint32_t, uint32_t) { ++imports; }
};
class EgsBaseCan {
public:
    ShifterPosition position = ShifterPosition::SignalNotAvailable;
    uint16_t left = UINT16_MAX, right = UINT16_MAX, rpm = UINT16_MAX;
    bool enforce_freshness = false;
    uint32_t now_ms = 0, last_rx_ms = 0, diag_rx_id = 0;
    QueueHandle_t* diag_rx_queue = nullptr;
    SlaveDecoder egs_slave_mode_tester;
    bool stale(unsigned ttl) const { return enforce_freshness && now_ms - last_rx_ms > ttl; }
    ShifterPosition get_shifter_position(unsigned ttl) { assert(ttl == 250); return stale(ttl) ? ShifterPosition::SignalNotAvailable : position; }
    uint16_t get_rear_left_wheel(unsigned ttl) { assert(ttl == 250); return stale(ttl) ? UINT16_MAX : left; }
    uint16_t get_rear_right_wheel(unsigned ttl) { assert(ttl == 250); return stale(ttl) ? UINT16_MAX : right; }
    uint16_t get_engine_rpm(unsigned ttl) { assert(ttl == 250); return stale(ttl) ? UINT16_MAX : rpm; }
    void dispatch_received_frame(const twai_message_t& rx, uint32_t now);
    void on_rx_frame(uint32_t id, uint8_t, uint64_t data, uint32_t timestamp) {
        if (id == 0x100) {
            position = static_cast<ShifterPosition>(data >> 56);
            left = (data >> 48) & 0xFF;
            right = (data >> 40) & 0xFF;
            last_rx_ms = timestamp;
        }
    }
};
extern EgsBaseCan* egs_can_hal;
''',
    "diag/kwp_utils.h": '#pragma once\n#include "diag/persistence_safety.h"\n',
    "solenoids/solenoids.h": r'''
#pragma once
#include "esp_err.h"
#include "freertos/semphr.h"
#include <functional>
struct InrushControlSolenoid {
    int stops = 0, starts = 0;
    bool paused = false;
    esp_err_t stop_error = ESP_OK, start_error = ESP_OK;
    std::function<void()> stop_hook;
    esp_err_t pause_timer() { ++stops; paused = true; if (stop_hook) stop_hook(); return stop_error; }
    esp_err_t resume_timer() { ++starts; if (start_error == ESP_OK) paused = false; return start_error; }
};
extern InrushControlSolenoid* sol_tcc;
''',
    "freertos/semphr.h": r'''
#pragma once
#include <mutex>
#include <thread>
#include <cassert>
struct StaticSemaphore_t {
    std::recursive_mutex mutex;
    std::thread::id owner;
    unsigned depth = 0;
};
using SemaphoreHandle_t = StaticSemaphore_t*;
constexpr int pdTRUE = 1;
#define configASSERT(x) assert(x)
inline auto xTaskGetCurrentTaskHandle() { return std::this_thread::get_id(); }
inline auto xSemaphoreGetMutexHolder(SemaphoreHandle_t s) { return s->owner; }
inline SemaphoreHandle_t xSemaphoreCreateRecursiveMutexStatic(StaticSemaphore_t* s) { return s; }
inline int xSemaphoreTakeRecursive(SemaphoreHandle_t s, int) {
    if (!s->mutex.try_lock()) return 0;
    s->owner = std::this_thread::get_id(); ++s->depth; return pdTRUE;
}
inline int xSemaphoreGiveRecursive(SemaphoreHandle_t s) {
    assert(s->owner == std::this_thread::get_id() && s->depth);
    if (--s->depth == 0) s->owner = {};
    s->mutex.unlock(); return pdTRUE;
}
''',
    "esp_check.h": '#pragma once\n',
    "tcu_alloc.h": '#pragma once\n#include <cstdlib>\n#define TCU_HEAP_ALLOC(n) malloc(n)\n#define TCU_FREE(p) free(p)\n',
    "common_structs_ops.h": '#pragma once\n',
    "maps.h": '#pragma once\ninline const int16_t GEAR_ADAPT_MAP[8] = {};\n',
    "nvs/module_settings.h": '#pragma once\nstruct HostAdaptSettings { int prefill_max_time_delta = 100; };\ninline HostAdaptSettings ADP_CURRENT_SETTINGS;\n',
    "nvs/all_keys.h": '#pragma once\ninline constexpr char NVS_KEY_MAP_NAME_ADAPT_PREFILL_TIME[] = "prefill";\ninline constexpr char NVS_KEY_MAP_NAME_ADAPT_APPLYING_TRQ[] = "apply";\ninline constexpr char NVS_KEY_MAP_NAME_ADAPT_FREEING_TRQ[] = "free";\ninline constexpr char NVS_KEY_MAP_NAME_ADAPT_SPC_OFFSET[] = "spc";\n',
}


class PersistenceHostTest(unittest.TestCase):
    def test_ownership_and_rejected_mode_changes(self):
        with tempfile.TemporaryDirectory(prefix="nag52-persistence-") as temp:
            root = Path(temp)
            for name, body in HEADERS.items():
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(body)
            binary = root / 'host_persistence'
            subprocess.run([
                os.environ.get('CXX', 'c++'), '-std=c++17', '-pthread', '-Wall', '-Wextra', '-Werror',
                '-include', 'cstdlib', '-I' + str(REPO / 'src/nvs'),
                '-I' + str(root), '-I' + str(REPO / 'src'),
                str(REPO / 'test/host_persistence.cpp'),
                str(REPO / 'test/host_persistent_maps.cpp'),
                str(REPO / 'src/stored_map.cpp'), str(REPO / 'src/stored_table.cpp'),
                str(REPO / 'src/stored_data.cpp'), str(REPO / 'src/adaptation/shift_adaptation.cpp'),
                str(REPO / 'src/solenoids/flash_guard.cpp'),
                str(REPO / 'src/canbus/can_rx_dispatch.cpp'),
                str(REPO / 'src/diag/device_mode.cpp'), '-o', str(binary),
            ], check=True)
            subprocess.run([str(binary)], check=True, timeout=15)


if __name__ == '__main__':
    unittest.main()
