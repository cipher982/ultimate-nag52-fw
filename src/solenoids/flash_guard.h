#ifndef TCC_FLASH_GUARD_H
#define TCC_FLASH_GUARD_H

#include "esp_err.h"

// Task-affine, recursive ownership of the entire persistence operation. Contention
// fails immediately: the gearbox task must never wait behind a diagnostic session.
class TccFlashGuard {
public:
    explicit TccFlashGuard(bool acquire_now = true);
    ~TccFlashGuard();
    static void end_boot();
    TccFlashGuard(const TccFlashGuard&) = delete;
    TccFlashGuard& operator=(const TccFlashGuard&) = delete;
    esp_err_t acquire();
    esp_err_t release();
    esp_err_t status() const { return result; }
    bool owns_lock() const { return held; }
private:
    bool held = false;
    esp_err_t result = ESP_ERR_INVALID_STATE;
};

#endif
