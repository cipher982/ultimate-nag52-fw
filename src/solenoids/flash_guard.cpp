#include "flash_guard.h"
#include "solenoids/solenoids.h"
#include "diag/kwp_utils.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include <atomic>
// Static internal RAM, constructed once before app_main starts any tasks.
static StaticSemaphore_t mutex_storage;
static SemaphoreHandle_t const persistence_mutex = xSemaphoreCreateRecursiveMutexStatic(&mutex_storage);
static unsigned depth = 0; // Access only while owning persistence_mutex.

static std::atomic<bool> boot_complete{false};

void TccFlashGuard::end_boot() {
    boot_complete.store(true, std::memory_order_release);
}
TccFlashGuard::TccFlashGuard(bool acquire_now) {
    if (acquire_now) {
        acquire();
    }
}

TccFlashGuard::~TccFlashGuard() {
    if (held) {
        const esp_err_t e = release();
        if (e != ESP_OK) {
            ESP_LOGE("TCC_GUARD", "Timer resume failed: %s; output remains inhibited", esp_err_to_name(e));
        }
    }
}

esp_err_t TccFlashGuard::acquire() {
    if (held) {
        return ESP_OK;
    }
    if (xSemaphoreTakeRecursive(persistence_mutex, 0) != pdTRUE) {
        return result = ESP_ERR_TIMEOUT;
    }
    result = ESP_OK;
    if (depth == 0) {
        // Boot creates every default before start_timer(). A failed startup is
        // NOT an exemption for subsequent diagnostic writes.
        if (boot_complete.load(std::memory_order_acquire) && !is_stationary_passive(egs_can_hal)) {
            result = ESP_ERR_INVALID_STATE;
        } else if (sol_tcc != nullptr) {
            result = sol_tcc->pause_timer();
        }
    }
    if (result != ESP_OK) {
        xSemaphoreGiveRecursive(persistence_mutex);
        return result;
    }
    ++depth;
    held = true;
    return ESP_OK;
}

esp_err_t TccFlashGuard::release() {
    if (!held) {
        return result;
    }
    // A session guard is acquired, used and destroyed by the KWP server task.
    configASSERT(xSemaphoreGetMutexHolder(persistence_mutex) == xTaskGetCurrentTaskHandle());
    configASSERT(depth != 0);
    --depth;
    result = ESP_OK;
    if (depth == 0 && sol_tcc != nullptr) {
        result = sol_tcc->resume_timer();
    }
    held = false;
    xSemaphoreGiveRecursive(persistence_mutex);
    return result;
}
