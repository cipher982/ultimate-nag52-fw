#include "solenoids/flash_guard.h"
#include "solenoids/solenoids.h"
#include "diag/persistence_safety.h"
#include "nvs/eeprom_config.h"
#include "nvs/device_mode.h"
#include "solenoids/tcc_alarm_epoch.h"
#include <cassert>
#include <future>
#include <thread>
#include <iostream>
void test_persistent_maps();

EgsBaseCan can;
EgsBaseCan* egs_can_hal = &can;
InrushControlSolenoid timer;
InrushControlSolenoid* sol_tcc = &timer;
uint16_t CURRENT_DEVICE_MODE = 1;
static esp_err_t write_error = ESP_OK;
static unsigned writes = 0;
esp_err_t EEPROM::set_device_mode(uint16_t) {
    assert(timer.paused);
    ++writes;
    return write_error;
}

int main() {
    // A dispatch captured before stop cannot be mistaken for the new epoch,
    // including a pending IRQ relabeled by the driver with the new alarm.
    TccAlarmEpoch epoch;
    assert(epoch.restart_after(0, 100000));
    const uint64_t previous = epoch.expected;
    assert(epoch.is_due(previous, previous));
    assert(epoch.restart_after(500, 100000));
    assert(epoch.expected > previous);
    assert(!epoch.is_due(previous, previous));
    assert(!epoch.is_due(epoch.expected, 500));
    assert(!epoch.is_due(epoch.expected, epoch.expected - 1));
    assert(epoch.is_due(epoch.expected, epoch.expected));
    const uint64_t resumed = epoch.expected;
    assert(epoch.advance(15000));
    assert(!epoch.is_due(resumed, epoch.expected));
    assert(epoch.restart_after(epoch.expected + 1000000, 100000));
    assert(!epoch.restart_after(UINT64_MAX - 1, 2));
    // Boot can create defaults with no CAN evidence. Runtime cannot inherit
    // that exemption even when startup failed before a timer existed.
    { TccFlashGuard boot; assert(boot.status() == ESP_OK); }
    TccFlashGuard::end_boot();
    sol_tcc = nullptr;
    { TccFlashGuard denied; assert(denied.status() == ESP_ERR_INVALID_STATE); }
    sol_tcc = &timer;
    can.position = ShifterPosition::P;
    can.left = can.right = 0;
    {
        TccFlashGuard outer;
        assert(outer.status() == ESP_OK && timer.paused);
        const int starts = timer.starts;
        { TccFlashGuard inner; assert(inner.status() == ESP_OK); }
        assert(timer.paused && timer.starts == starts);
        std::thread competitor([&] {
            TccFlashGuard other;
            assert(other.status() == ESP_ERR_TIMEOUT);
        });
        competitor.join();
        assert(timer.paused);
    }
    assert(!timer.paused);

    // Contention during the hardware stop itself cannot interleave a start.
    std::promise<void> stopping, finish_stop;
    auto finished = finish_stop.get_future();
    timer.stop_hook = [&] { stopping.set_value(); finished.wait(); };
    std::thread owner([&] { TccFlashGuard guard; assert(guard.status() == ESP_OK); });
    stopping.get_future().wait();
    const int starts = timer.starts;
    { TccFlashGuard competitor; assert(competitor.status() == ESP_ERR_TIMEOUT); }
    assert(timer.starts == starts);
    finish_stop.set_value();
    owner.join();
    timer.stop_hook = {};
    assert(!timer.paused);

    timer.stop_error = ESP_FAIL;
    { TccFlashGuard failed; assert(failed.status() == ESP_FAIL && !failed.owns_lock()); }
    timer.stop_error = ESP_OK;
    { TccFlashGuard recovered; assert(recovered.status() == ESP_OK); }
    timer.start_error = ESP_FAIL;
    { TccFlashGuard guard; assert(guard.release() == ESP_FAIL); }
    timer.start_error = ESP_OK;
    { TccFlashGuard recovered; assert(recovered.status() == ESP_OK); }

    // All active/intermediate/undefined/SNV positions deny both transient and
    // persisted mode changes before any timer/GPIO or RAM-mode mutation.
    for (unsigned pos = 0; pos < 256; ++pos) {
        can.position = static_cast<ShifterPosition>(pos);
        if (can.position == ShifterPosition::P || can.position == ShifterPosition::N) continue;
        const int stops = timer.stops;
        const unsigned old_writes = writes;
        assert(change_device_mode(&can, 8, false) == ESP_ERR_INVALID_STATE);
        assert(change_device_mode(&can, 8, true) == ESP_ERR_INVALID_STATE);
        assert(CURRENT_DEVICE_MODE == 1 && timer.stops == stops && writes == old_writes);
    }
    can.position = ShifterPosition::N;
    for (uint16_t speed : {uint16_t(1), uint16_t(UINT16_MAX)}) {
        can.left = speed;
        assert(change_device_mode(&can, 8, true) == ESP_ERR_INVALID_STATE);
        can.left = 0;
        can.right = speed;
        assert(change_device_mode(&can, 8, true) == ESP_ERR_INVALID_STATE);
        can.right = 0;
    }
    assert(change_device_mode(nullptr, 8, true) == ESP_ERR_INVALID_STATE);
    write_error = ESP_FAIL;
    assert(change_device_mode(&can, 8, true) == ESP_FAIL);
    assert(CURRENT_DEVICE_MODE == 1 && !timer.paused);
    write_error = ESP_OK;
    assert(change_device_mode(&can, 8, true) == ESP_OK);
    assert(CURRENT_DEVICE_MODE == 8 && !timer.paused);
    assert(change_device_mode(&can, 1, false) == ESP_OK);
    assert(CURRENT_DEVICE_MODE == 1);
    // SLAVE and CANLOGGER must retain fresh safety decoding and diagnostic RX
    // without feeding logger frames to the slave actuation-command decoder.
    can.enforce_freshness = true;
    twai_message_t frame;
    frame.identifier = 0x100;
    frame.data[0] = static_cast<uint8_t>(ShifterPosition::P);
    for (uint16_t mode : {uint16_t(DEVICE_MODE_SLAVE), uint16_t(DEVICE_MODE_CANLOGGER)}) {
        can.now_ms += 1000;
        can.dispatch_received_frame(frame, can.now_ms);
        assert(change_device_mode(&can, mode, true) == ESP_OK);
        can.now_ms += 1000;
        assert(!is_stationary_passive(&can));
        const unsigned slave_imports = can.egs_slave_mode_tester.imports;
        can.dispatch_received_frame(frame, can.now_ms);
        assert(is_stationary_passive(&can));
        assert(can.egs_slave_mode_tester.imports == slave_imports + (mode == DEVICE_MODE_SLAVE ? 1 : 0));
        QueueHandle_t queue = 1;
        can.diag_rx_id = 0x7E1;
        can.diag_rx_queue = &queue;
        twai_message_t diagnostic;
        diagnostic.identifier = can.diag_rx_id;
        const unsigned received = diagnostic_frames;
        can.dispatch_received_frame(diagnostic, can.now_ms);
        assert(diagnostic_frames == received + 1);
        assert(change_device_mode(&can, DEVICE_MODE_NORMAL, true) == ESP_OK);
        can.diag_rx_id = 0;
        can.diag_rx_queue = nullptr;
    }
    can.enforce_freshness = false;
    test_persistent_maps();

    // An unchanged NVS value still requires closing its successfully opened
    // handle. Resource exhaustion is independent of physical flash writes.
    for (unsigned i = 0; i < 1000; ++i) {
        EEPROM::NvsHandle handle(NVS_READWRITE);
        assert(handle.error == ESP_OK && active_handles == 1);
    }
    assert(active_handles == 0);
    open_error = ESP_FAIL;
    { EEPROM::NvsHandle failed; assert(failed.error == ESP_FAIL); }
    assert(active_handles == 0);
    std::cout << "persistence ownership, rejection, error cleanup and handle lifetime: OK\n";
}
