#include "boot_info.h"
#include "esp_system.h"
#include "esp_log.h"

static uint8_t latched_reset_reason = 0u;

void BootInfo::record_reset_reason(void) {
    latched_reset_reason = static_cast<uint8_t>(esp_reset_reason());
    // Printed as well as latched: when a cable *is* attached, seeing it in the
    // boot log is faster than asking for it.
    ESP_LOGW("BOOT", "reset reason: %u", static_cast<unsigned>(latched_reset_reason));
}

uint8_t BootInfo::get_reset_reason(void) {
    return latched_reset_reason;
}
