#include "canbus/can_hal.h"
#include "nvs/device_mode.h"
#include "esp_log.h"
#include <stdio.h>

void EgsBaseCan::dispatch_received_frame(const twai_message_t& rx, uint32_t now) {
    if (rx.data_length_code > sizeof(rx.data)) {
        return;
    }
    if (CHECK_MODE_BIT_ENABLED(DEVICE_MODE_CANLOGGER)) {
        char buf[35];
        int pos = 0;
        pos += snprintf(buf + pos, sizeof(buf) - pos, "CF->0x%04X", static_cast<unsigned>(rx.identifier & 0xFFFF));
        for (uint8_t i = 0; i < rx.data_length_code; i++) {
            pos += snprintf(buf + pos, sizeof(buf) - pos, "%02X", static_cast<unsigned>(rx.data[i]));
        }
        printf("%.*s\n", pos, buf);
    }
    if (this->diag_rx_id != 0 && rx.identifier == this->diag_rx_id) {
        // Diagnostics remain reachable even in logging mode.
        if (this->diag_rx_queue != nullptr && rx.data_length_code == 8) {
            if (xQueueSend(*this->diag_rx_queue, rx.data, 0) != pdTRUE) {
                ESP_LOGE("EGS_BASIC_CAN", "Discarded ISO-TP endpoint frame. Queue send failed");
            }
        }
    } else {
        uint64_t data = 0;
        for (uint8_t i = 0; i < rx.data_length_code; i++) {
            data |= uint64_t(rx.data[i]) << (8 * (7 - i));
        }
        // Every CAN implementation only imports snapshots here. Normal TX and
        // controller/actuator execution remain separately mode-gated.
        this->on_rx_frame(rx.identifier, rx.data_length_code, data, now);
        if (CHECK_MODE_BIT_ENABLED(DEVICE_MODE_SLAVE) &&
            !CHECK_MODE_BIT_ENABLED(DEVICE_MODE_CANLOGGER)) {
            this->egs_slave_mode_tester.import_frames(data, rx.identifier, now);
        }
    }
}
