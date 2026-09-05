#include "eeprom_config.h"
#include "esp_log.h"
#include "speaker.h"
#include <string.h>
#include "profiles.h"
#include "efuse/efuse.h"
#include "esp_efuse.h"
#include "esp_check.h"
#include "device_mode.h"
#include "esp_app_desc.h"
#include "all_keys.h"
#include "esp_flash.h"

uint16_t CURRENT_DEVICE_MODE = DEVICE_MODE_NORMAL;

esp_err_t EEPROM::read_nvs_map_data(const char* map_name, int16_t* dest, const int16_t* default_map, size_t map_element_count) {
    NvsHandle handle;
    if (handle.error != ESP_OK) return handle.error;
    const size_t expected = map_element_count * sizeof(int16_t);
    size_t size = expected;
    esp_err_t e = nvs_get_blob(handle.value, map_name, dest, &size);
    if (e == ESP_ERR_NVS_NOT_FOUND && default_map != nullptr) {
        memcpy(dest, default_map, expected);
        return write_nvs_map_data(map_name, default_map, map_element_count);
    }
    return e == ESP_OK && size != expected ? ESP_ERR_INVALID_SIZE : e;
}

esp_err_t EEPROM::check_if_new_fw(bool* dest) {
    TccFlashGuard flash_guard;
    if (flash_guard.status() != ESP_OK) return flash_guard.status();
    NvsHandle handle(NVS_READWRITE);
    if (handle.error != ESP_OK) return handle.error;
    const esp_app_desc_t* now = esp_app_get_description();
    uint8_t sha[32];
    size_t len = sizeof(sha);
    esp_err_t e = nvs_get_blob(handle.value, NVS_KEY_LAST_FW, sha, &len);
    if (e != ESP_OK && e != ESP_ERR_NVS_NOT_FOUND) return e;
    *dest = e == ESP_ERR_NVS_NOT_FOUND || len != sizeof(sha) || memcmp(sha, now->app_elf_sha256, sizeof(sha)) != 0;
    if (*dest) {
        e = nvs_set_blob(handle.value, NVS_KEY_LAST_FW, now->app_elf_sha256, sizeof(sha));
        if (e == ESP_OK) e = nvs_commit(handle.value);
    }
    const esp_err_t resumed = flash_guard.release();
    return e == ESP_OK ? resumed : e;
}

esp_err_t EEPROM::write_nvs_map_data(const char* map_name, const int16_t* to_write, size_t map_element_count) {
    TccFlashGuard flash_guard;
    if (flash_guard.status() != ESP_OK) return flash_guard.status();
    NvsHandle handle(NVS_READWRITE);
    if (handle.error != ESP_OK) return handle.error;
    esp_err_t e = nvs_set_blob(handle.value, map_name, to_write, map_element_count * sizeof(int16_t));
    if (e == ESP_OK) e = nvs_commit(handle.value);
    const esp_err_t resumed = flash_guard.release();
    return e == ESP_OK ? resumed : e;
}

esp_err_t EEPROM::read_device_mode(uint16_t* mode) {
    NvsHandle handle;
    if (handle.error != ESP_OK) return handle.error;
    esp_err_t e = nvs_get_u16(handle.value, NVS_KEY_DEV_MODE, mode);
    if (e == ESP_ERR_NVS_NOT_FOUND) {
        *mode = DEVICE_MODE_NORMAL;
        return set_device_mode(*mode);
    }
    return e;
}

esp_err_t EEPROM::set_device_mode(uint16_t mode) {
    TccFlashGuard flash_guard;
    if (flash_guard.status() != ESP_OK) return flash_guard.status();
    NvsHandle handle(NVS_READWRITE);
    if (handle.error != ESP_OK) return handle.error;
    esp_err_t e = nvs_set_u16(handle.value, NVS_KEY_DEV_MODE, mode);
    if (e == ESP_OK) e = nvs_commit(handle.value);
    const esp_err_t resumed = flash_guard.release();
    return e == ESP_OK ? resumed : e;
}


esp_err_t EEPROM::init_eeprom() {
    TccFlashGuard flash_guard;
    if (flash_guard.status() != ESP_OK) return flash_guard.status();
    esp_err_t e = nvs_flash_init();
    if (e != ESP_OK) return e; // Never erase user configuration to recover init.
    NvsHandle handle(NVS_READWRITE);
    if (handle.error != ESP_OK) return handle.error;
    bool new_fw = false;
    e = check_if_new_fw(&new_fw);
    if (e != ESP_OK) return e;
    if (new_fw) {
        nvs_iterator_t it = nullptr;
        esp_err_t iter = nvs_entry_find("nvs", NVS_PARTITION_USER_CFG, NVS_TYPE_ANY, &it);
        while (iter == ESP_OK) {
            nvs_entry_info_t info;
            e = nvs_entry_info(it, &info);
            if (e != ESP_OK) break;
            iter = nvs_entry_next(&it);
            bool found = false;
            for (int i = 0; i < ALL_NVS_KEYS_LEN; ++i) {
                if (strcmp(*ALL_NVS_KEYS[i], info.key) == 0) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                e = nvs_erase_key(handle.value, info.key);
                if (e != ESP_OK) break;
            }
        }
        nvs_release_iterator(it);
        if (e != ESP_OK) return e;
        if (iter != ESP_ERR_NVS_NOT_FOUND) return iter;
        e = nvs_commit(handle.value);
        if (e != ESP_OK) return e;
        FLASH_NVS_SETTINGS_DESC desc = {};
        e = esp_flash_read(esp_flash_default_chip, &desc, 0x330000, sizeof(desc));
        if (e != ESP_OK) return e;
        if (desc.magic[0] == 0xDE && desc.magic[1] == 0xAD && desc.magic[2] == 0xBE && desc.magic[3] == 0xEF) {
            uint32_t key_cs = 0;
            for (int i = 0; i < ALL_NVS_KEYS_LEN; ++i) {
                const char* key = *ALL_NVS_KEYS[i];
                const size_t len = strlen(key);
                for (size_t l = 0; l < len; ++l) key_cs += l + key[l];
            }
            if (key_cs != desc.key_cs) {
                e = esp_flash_erase_region(esp_flash_default_chip, 0x330000, 4096);
                if (e != ESP_OK) return e;
            }
        }
    }
    e = read_core_config(&VEHICLE_CONFIG);
    if (e == ESP_OK) e = read_device_mode(&CURRENT_DEVICE_MODE);
    const esp_err_t resumed = flash_guard.release();
    return e == ESP_OK ? resumed : e;
}

esp_err_t EEPROM::read_core_config(TCM_CORE_CONFIG* dest) {
    NvsHandle handle;
    if (handle.error != ESP_OK) return handle.error;
    size_t s = sizeof(TCM_CORE_CONFIG);
    esp_err_t result = nvs_get_blob(handle.value, NVS_KEY_CORE_SCN, dest, &s);
    if (result == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOG_LEVEL(ESP_LOG_WARN, "EEPROM", "SCN Config not found. Creating a new one");
        TCM_CORE_CONFIG c = {
            .deprecated_is_large_nag = 0,
            .diff_ratio = 1000,
            .wheel_circumference = 2850,
            .is_four_matic = 0,
            .transfer_case_high_ratio = 1000,
            .transfer_case_low_ratio = 1000,
            .default_profile = 0, // Standard
            .red_line_rpm_diesel = 4500, // Safe for diesels, petrol-heads can change this!
            .red_line_rpm_petrol = 6000,
            .engine_type = 0,
            .egs_can_type = 0,
            .shifter_style = 0,
            .io_0_usage = 0,
            .input_sensor_pulses_per_rev = 1,
            .output_pulse_width_per_kmh = 1,
            .gen_mosfet_purpose = 0,
            .throttlevalve_maxopeningangle = 255, // 89.25°
            .c_eng = 1000,
            .engine_drag_torque = 400, // 40Nm
            .jeep_chrysler = false
        };
        result = save_core_config(&c);
        if (result == ESP_OK) *dest = c;
        return result;
    }
    return result == ESP_OK && s != sizeof(*dest) ? ESP_ERR_INVALID_SIZE : result;
}

esp_err_t EEPROM::save_core_config(TCM_CORE_CONFIG* write) {
    TccFlashGuard flash_guard;
    if (flash_guard.status() != ESP_OK) return flash_guard.status();
    NvsHandle handle(NVS_READWRITE);
    if (handle.error != ESP_OK) return handle.error;
    esp_err_t e = nvs_set_blob(handle.value, NVS_KEY_CORE_SCN, write, sizeof(*write));
    if (e == ESP_OK) e = nvs_commit(handle.value);
    const esp_err_t resumed = flash_guard.release();
    return e == ESP_OK ? resumed : e;
}

esp_err_t EEPROM::ewm_btn_get_saved_profile(uint8_t* dest) {
    NvsHandle handle;
    if (handle.error != ESP_OK) return handle.error;
    return nvs_get_u8(handle.value, NVS_KEY_LAST_PROFILE, dest);
}

esp_err_t EEPROM::ewm_btn_save_profile(uint8_t save_profile) {
    TccFlashGuard flash_guard;
    if (flash_guard.status() != ESP_OK) return flash_guard.status();
    NvsHandle handle(NVS_READWRITE);
    if (handle.error != ESP_OK) return handle.error;
    esp_err_t e = nvs_set_u8(handle.value, NVS_KEY_LAST_PROFILE, save_profile);
    if (e == ESP_OK) e = nvs_commit(handle.value);
    const esp_err_t resumed = flash_guard.release();
    return e == ESP_OK ? resumed : e;
}

esp_err_t EEPROM::read_efuse_config(TCM_EFUSE_CONFIG* dest) {
    if (dest == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    ESP_RETURN_ON_ERROR(esp_efuse_read_field_blob(ESP_EFUSE_BOARD_VER, &dest->board_ver, 8), "EFUSE_CFG", "Could not read board ver");
    ESP_RETURN_ON_ERROR(esp_efuse_read_field_blob(ESP_EFUSE_M_DAY, &dest->manufacture_day, 8), "EFUSE_CFG", "Could not read manf. day");
    ESP_RETURN_ON_ERROR(esp_efuse_read_field_blob(ESP_EFUSE_M_WEEK, &dest->manufacture_week, 8), "EFUSE_CFG", "Could not read manf. week");
    ESP_RETURN_ON_ERROR(esp_efuse_read_field_blob(ESP_EFUSE_M_MONTH, &dest->manufacture_month, 8), "EFUSE_CFG", "Could not read manf. month");
    ESP_RETURN_ON_ERROR(esp_efuse_read_field_blob(ESP_EFUSE_M_YEAR, &dest->manufacture_year, 8), "EFUSE_CFG", "Could not read manf. year");
    return ESP_OK;
}

esp_err_t EEPROM::write_efuse_config(TCM_EFUSE_CONFIG* dest) {
    if (dest == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }
    if (dest->manufacture_month > 12 || dest->manufacture_month == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (dest->manufacture_day > 31 || dest->manufacture_day == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (dest->manufacture_week > 52 || dest->manufacture_week == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (dest->manufacture_year < 22) {
        return ESP_ERR_INVALID_ARG;
    }
    if (dest->board_ver == 0 || dest->board_ver > 3) {
        return ESP_ERR_INVALID_ARG;
    }
    if (esp_efuse_write_field_blob(ESP_EFUSE_BOARD_VER, &dest->board_ver, 8) != ESP_OK) {
        return ESP_ERR_INVALID_ARG;
    }
    if (esp_efuse_write_field_blob(ESP_EFUSE_M_DAY, &dest->manufacture_day, 8) != ESP_OK) {
        return ESP_ERR_INVALID_ARG;
    }
    if (esp_efuse_write_field_blob(ESP_EFUSE_M_WEEK, &dest->manufacture_week, 8) != ESP_OK) {
        return ESP_ERR_INVALID_ARG;
    }
    if (esp_efuse_write_field_blob(ESP_EFUSE_M_MONTH, &dest->manufacture_month, 8) != ESP_OK) {
        return ESP_ERR_INVALID_ARG;
    }
    if (esp_efuse_write_field_blob(ESP_EFUSE_M_YEAR, &dest->manufacture_year, 8) != ESP_OK) {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
    
}

TCM_CORE_CONFIG VEHICLE_CONFIG = {};
TCM_EFUSE_CONFIG BOARD_CONFIG = {};