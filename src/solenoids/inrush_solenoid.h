#ifndef __INRUSH_SOLENOID_H_
#define __INRUSH_SOLENOID_H_

#include "pwm_solenoid.h"
#include "driver/gptimer.h"
#include "flash_guard.h"
class InrushControlSolenoid : public PwmSolenoid {
public:
    explicit InrushControlSolenoid(const char *name, ledc_timer_t ledc_timer, gpio_num_t pwm_pin, gpio_num_t zener_pin, ledc_channel_t channel, adc_channel_t read_channel, uint16_t period_hz, uint16_t target_hold_current_ma, uint16_t phase_duration_ms);
    void __write_pwm(float vref_compensation, float temperature_factor);
    uint32_t on_timer_interrupt();
    uint32_t on_timer_interrupt_new();
    void set_duty(uint16_t duty);
    esp_err_t pre_current_test() override;
    esp_err_t post_current_test() override;
    esp_err_t start_timer();
    bool is_disabled();
    bool timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata);
private:
    friend class TccFlashGuard;
    esp_err_t pause_timer();
    esp_err_t resume_timer();
    void force_output_low();
    TccFlashGuard current_test_guard{false};
    bool timer_started = false;
    bool timer_fault = false;
    ledc_timer_t ledc_timer;
    float vref = 1.0;
    // 0 - Inrush
    // 1 - Hold
    // 2 - Off
    uint8_t phase_id = 2; // So we start at 0 again
    uint32_t period_on_time = 0;
    uint16_t target_hold_current = 0;
    uint16_t calc_hold_pwm = 1024;
    uint32_t inrush_time = 20000; // at 12V and 25C
    uint32_t hold_time = 0; 
    uint32_t off_time  = 0;

    uint32_t inrush_time_this_cycle = 0;
    uint32_t hold_time_this_cycle = 0;
    uint32_t off_time_this_cycle = 0;
    uint32_t pwm_on_time = 0;
    uint32_t pwm_off_time = 0;

    gptimer_handle_t timer = nullptr;
    bool off = false;
    gpio_num_t zener_pin = GPIO_NUM_NC;
    gpio_num_t pwm_pin = GPIO_NUM_NC;
};

#endif