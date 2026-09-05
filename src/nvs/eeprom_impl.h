/** @file */
#ifndef EEPROM_IMPL_H
#define EEPROM_IMPL_H

#include "eeprom_config.h"

namespace EEPROM {
    template <typename T>
    esp_err_t read_subsystem_settings(const char* key_name, T* dest, const T* default_settings) {
        NvsHandle handle;
        if (handle.error != ESP_OK) return handle.error;
        size_t size = sizeof(T);
        esp_err_t e = nvs_get_blob(handle.value, key_name, dest, &size);
        if (e == ESP_ERR_NVS_NOT_FOUND && default_settings != nullptr) {
            memcpy(dest, default_settings, sizeof(T));
            return write_subsystem_settings(key_name, default_settings);
        }
        return e == ESP_OK && size != sizeof(T) ? ESP_ERR_INVALID_SIZE : e;
    }

    template <typename T>
    esp_err_t write_subsystem_settings(const char* key_name, const T* write) {
        TccFlashGuard flash_guard;
        if (flash_guard.status() != ESP_OK) return flash_guard.status();
        NvsHandle handle(NVS_READWRITE);
        if (handle.error != ESP_OK) return handle.error;
        esp_err_t e = nvs_set_blob(handle.value, key_name, write, sizeof(T));
        if (e == ESP_OK) e = nvs_commit(handle.value);
        const esp_err_t resumed = flash_guard.release();
        return e == ESP_OK ? resumed : e;
    }

}
#endif