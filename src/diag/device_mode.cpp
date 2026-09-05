#include "nvs/device_mode.h"
#include "nvs/eeprom_config.h"
#include "diag/kwp_utils.h"

esp_err_t change_device_mode(EgsBaseCan* can, uint16_t requested, bool persist) {
    // Reject before guard acquisition can alter GPIO, or CURRENT_DEVICE_MODE can
    // redirect the controller. Read-only mode reporting does not use this path.
    if (!is_stationary_passive(can)) return ESP_ERR_INVALID_STATE;
    TccFlashGuard flash_guard;
    if (flash_guard.status() != ESP_OK) return flash_guard.status();
    esp_err_t e = persist ? EEPROM::set_device_mode(requested) : ESP_OK;
    const esp_err_t resumed = flash_guard.release();
    if (e == ESP_OK) e = resumed;
    if (e == ESP_OK) CURRENT_DEVICE_MODE = requested;
    return e;
}
