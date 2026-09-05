#include "inrush_solenoid.h"
#include "esp_check.h"
#include "tcu_maths.h"
#include "soc/gpio_struct.h"
#include "soc/ledc_struct.h"
#include "tcc_alarm_epoch.h"

// AT 12.0V
const DRAM_ATTR uint16_t INRUSH_START_PWM = 224; // Any PWM below this will just write 0 to solenoid (Not enough open time for arm to move)
const DRAM_ATTR uint16_t INRUSH_SKIP_PWM = 3220; // Any PWM above this will skip inrush and just go to hold as there is enough current
const DRAM_ATTR uint16_t INRUSH_TIME_US = 15000; 
const DRAM_ATTR uint16_t INRUSH_PWM = 4096;
const DRAM_ATTR uint16_t HOLD_PWM = 1300;
const DRAM_ATTR uint32_t TOTAL_PERIOD_TIME_US = 100000; // Timer runs at 10MHz, Hydralic PWM is 100Hz, so 10_000_000/100


// This singleton TCC gate is internal RAM even if a solenoid was allocated in
// PSRAM. The IRAM callback tests it BEFORE dereferencing user_data. Holding this
// short lock spans all callback work, including alarm reprogramming: a task that
// closes the gate has synchronously drained any callback already admitted.
static DRAM_ATTR portMUX_TYPE callback_mux = portMUX_INITIALIZER_UNLOCKED;
static DRAM_ATTR bool callback_blocked = true;
static DRAM_ATTR TccAlarmEpoch alarm_epoch;

static bool IRAM_ATTR inrush_solenoid_timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    portENTER_CRITICAL_ISR(&callback_mux);
    if (!callback_blocked && alarm_epoch.is_due(edata->alarm_value, edata->count_value)) {
        auto* solenoid = static_cast<InrushControlSolenoid*>(user_data);
        if (!solenoid->timer_callback(timer, edata)) {
            callback_blocked = true;
        }
    }
    portEXIT_CRITICAL_ISR(&callback_mux);
    return false;
}

bool IRAM_ATTR InrushControlSolenoid::timer_callback(gptimer_handle_t timer, const gptimer_alarm_event_data_t* edata) {
    uint32_t next = this->zener_pin == GPIO_NUM_NC ? on_timer_interrupt() : on_timer_interrupt_new();
    gptimer_alarm_config_t alarm_config = {};
    if (!alarm_epoch.advance(MAX(next, 1u))) {
        this->timer_fault = true;
        force_output_low();
        return false;
    }
    alarm_config.alarm_count = alarm_epoch.expected;
    const esp_err_t e = gptimer_set_alarm_action(timer, &alarm_config);
    if (e != ESP_OK) {
        this->timer_fault = true;
        force_output_low();
    }
    return e == ESP_OK;
}

InrushControlSolenoid::InrushControlSolenoid(const char *name, ledc_timer_t ledc_timer, gpio_num_t pwm_pin, gpio_num_t zener_pin, ledc_channel_t channel, adc_channel_t read_channel, uint16_t period_hz, uint16_t target_hold_current_ma, uint16_t phase_duration_ms)
: PwmSolenoid(name, ledc_timer, pwm_pin, channel, read_channel, phase_duration_ms) {
    this->ledc_timer = ledc_timer;
    this->target_hold_current = target_hold_current_ma;
    this->zener_pin = zener_pin;
    this->pwm_pin = pwm_pin;
    // Old defaults
    int freq = 10000;
    gptimer_alarm_cb_t callback = inrush_solenoid_timer_isr;
    if (GPIO_NUM_NC != this->zener_pin) { // Override! New mechanics
        freq = 1000;
        ledc_stop(LEDC_HIGH_SPEED_MODE, channel, 0);
        gpio_set_direction(pwm_pin, gpio_mode_t::GPIO_MODE_OUTPUT);
        gpio_set_direction(zener_pin, gpio_mode_t::GPIO_MODE_OUTPUT);
        this->inrush_time = 0;
        this->hold_time = 0;
        this->off_time = TOTAL_PERIOD_TIME_US;
    } else {
        ledc_set_freq(LEDC_HIGH_SPEED_MODE, ledc_timer, freq);
    }
    if (ESP_OK != this->ready) {
        return; // Error trying to init base class, so skip
    }

    const gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = (10u * 1000u * 1000u), // 10MHz
        .flags = {
            .intr_shared = 0,
            .allow_pd = 0,
            .backup_before_sleep = 0
        }
    };

    this->ready = gptimer_new_timer(&timer_config, &this->timer);
    if (ESP_OK == ready) {
        const gptimer_alarm_config_t alarm_config = {
            .alarm_count = 0u,
            .reload_count = 0u,
            .flags = {
                .auto_reload_on_alarm = 0u,
            }
        };
        this->ready = gptimer_set_alarm_action(this->timer, &alarm_config);
        if (ESP_OK == ready) {
            gptimer_event_callbacks_t cbs = {
                .on_alarm = callback
            };
            this->ready = gptimer_set_alarm_action(this->timer, &alarm_config);
            if (ESP_OK == ready) {
                this->ready = gptimer_register_event_callbacks(this->timer, &cbs, reinterpret_cast<void*>(this));
                if (ESP_OK == ready) {
                    this->ready = gptimer_enable(this->timer);
                    if (ESP_OK == ready) {
                        // Defaults/maps are loaded after construction. Start only
                        // once setup_tcm has completed all boot persistence.
                        ESP_LOGI("ICSolenoid", "ICSolenoid %s initialized, timer deferred", this->name);
                    } else {
                        ESP_LOGE("ICSolenoid", "ICSolenoid %s gptimer_enable failed: %s", this->name, esp_err_to_name(this->ready));
                    }
                } else {
                    ESP_LOGE("ICSolenoid", "ICSolenoid %s callback registration failed: %s", this->name, esp_err_to_name(this->ready));
                }
            }
        }
    }
}

esp_err_t InrushControlSolenoid::pre_current_test() {
    return current_test_guard.acquire();
}

esp_err_t InrushControlSolenoid::post_current_test() {
    return current_test_guard.release();
}

void IRAM_ATTR InrushControlSolenoid::force_output_low() {
    if (GPIO_NUM_NC != this->zener_pin) {
        GPIO.out_w1tc = (uint32_t(1) << this->zener_pin) | (uint32_t(1) << this->pwm_pin);
    } else {
        auto& channel = LEDC.channel_group[LEDC_HIGH_SPEED_MODE].channel[this->channel];
        channel.conf0.idle_lv = 0;
        channel.conf0.sig_out_en = 0;
        channel.conf1.duty_start = 0;
    }
}

esp_err_t InrushControlSolenoid::pause_timer() {
    portENTER_CRITICAL(&callback_mux);
    callback_blocked = true;
    const bool faulted = timer_fault;
    portEXIT_CRITICAL(&callback_mux);
    // No admitted callback can now touch GPIO/LEDC or re-arm an alarm. The
    // spinlock is RELEASED before all driver calls and all flash operations.
    if (!timer_started) {
        return ESP_OK;
    }
    const esp_err_t e = gptimer_stop(timer);
    if (e != ESP_OK) {
        timer_fault = true;
    }
    force_output_low();
    return e == ESP_OK && faulted ? ESP_ERR_INVALID_STATE : e;
}

esp_err_t InrushControlSolenoid::resume_timer() {
    if (!timer_started) {
        return ESP_OK;
    }
    if (timer_fault) {
        return ESP_ERR_INVALID_STATE; // Fail closed; never write with a bad stop.
    }
    // Never reset the hardware counter or reuse an alarm identity. A driver ISR
    // may have captured edata before reaching our gate, and a pending interrupt
    // may be relabeled with the new alarm. Identity plus count>=deadline rejects
    // both cases until the newly scheduled alarm really becomes due.
    this->phase_id = 0;
    uint64_t stopped_count = 0;
    esp_err_t e = gptimer_get_raw_count(timer, &stopped_count);
    gptimer_alarm_config_t alarm = {};
    if (e == ESP_OK && !alarm_epoch.restart_after(stopped_count, TOTAL_PERIOD_TIME_US)) {
        e = ESP_ERR_INVALID_STATE; // Fail closed rather than reuse after wrap.
    }
    alarm.alarm_count = alarm_epoch.expected;
    if (e == ESP_OK) {
        e = gptimer_set_alarm_action(timer, &alarm);
    }
    if (e == ESP_OK) {
        portENTER_CRITICAL(&callback_mux);
        callback_blocked = false;
        portEXIT_CRITICAL(&callback_mux);
        e = gptimer_start(timer);
    }
    if (e != ESP_OK) {
        portENTER_CRITICAL(&callback_mux);
        callback_blocked = true;
        timer_fault = true;
        portEXIT_CRITICAL(&callback_mux);
        force_output_low();
    }
    return e;
}

esp_err_t InrushControlSolenoid::start_timer() {
    // Boot-only, before any controller/diagnostic task may request persistence.
    if (timer_started || ready != ESP_OK) {
        return ESP_ERR_INVALID_STATE;
    }
    TccFlashGuard::end_boot();
    timer_started = true;
    return resume_timer();
}

bool InrushControlSolenoid::is_disabled() {
    portENTER_CRITICAL(&callback_mux);
    const bool disabled = callback_blocked;
    portEXIT_CRITICAL(&callback_mux);
    return disabled;
}

bool on = false;
bool pwm_on = false;
bool zener_on = false;
uint32_t total  = 0;
bool pwm_en = false;
uint32_t IRAM_ATTR InrushControlSolenoid::on_timer_interrupt_new() {
    // Control the zener phase
    int ret = TOTAL_PERIOD_TIME_US;
    if (this->inrush_time != 0 || this->hold_time != 0) {
        if (this->phase_id == 0) { // Off -> Inrush
            pwm_on = true;
            zener_on = false;
            ret = this->inrush_time;
            if (this->hold_time != 0) {
                this->phase_id = 1; // Inrush -> hold
                pwm_en = false;
                total = 0;
            } else {
                this->phase_id = 2; // Inrush -> off (No hold)
            }
        } else if (this->phase_id == 1) { // Inrush -> Hold
            zener_on = false;
            pwm_on = pwm_en;
            pwm_en = !pwm_en;
            // Phase on/off for pwm

            ret = MIN(!pwm_en ? this->pwm_on_time : this->pwm_off_time, total - this->hold_time);
            if (total+ret >= this->hold_time) {
                if (this->off_time == 0) {
                    // We go back to this phase (Constant hold)
                    total = 0;
                } else {
                    this->phase_id = 2; // Done, turn off!
                }
            }
            total += ret;
        } else { // Hold -> Off
            zener_on = true;
            pwm_on = false;
            this->phase_id = 0;
            ret = this->off_time;
        }
    } else {
        zener_on = false;
        pwm_on = false;
        this->phase_id = 0; // Off
    }
    volatile uint32_t* reg_pwm = pwm_on ? &GPIO.out_w1ts : &GPIO.out_w1tc;
    volatile uint32_t* reg_zen = zener_on ? &GPIO.out_w1ts : &GPIO.out_w1tc;
    *reg_pwm = (uint32_t)1 << (this->pwm_pin);
    *reg_zen = (uint32_t)1 << (this->zener_pin);
    return ret;
}

// 100,000 is 10ms of time
uint32_t IRAM_ATTR InrushControlSolenoid::on_timer_interrupt() {
    uint32_t ret = 0;
    uint16_t write_pwm = 0;
    // Special handling for Min/Max PWM
    if (this->pwm_raw < INRUSH_START_PWM) {
        write_pwm = 0;
        ret = TOTAL_PERIOD_TIME_US;
    } else if (this->pwm_raw > INRUSH_SKIP_PWM) {
        write_pwm = this->calc_hold_pwm;
        ret = TOTAL_PERIOD_TIME_US;
    } else {
        if (this->phase_id == 0) { // Off -> Inrush
            write_pwm = 4096;
            // Grab all values now
            this->inrush_time_this_cycle = this->inrush_time;
            this->hold_time_this_cycle = this->hold_time;
            this->off_time_this_cycle = this->off_time;
            ret = this->inrush_time_this_cycle;
            this->phase_id = this->hold_time_this_cycle == 0 ? 2 : 1;
        } else if (this->phase_id == 1) { // Inrush -> hold
            ret = this->hold_time_this_cycle;
            this->phase_id = 2;
            write_pwm = this->calc_hold_pwm;
        } else { // Hold -> Off
            write_pwm = 0;
            this->phase_id = 0;
            ret = this->off_time_this_cycle;
        }
    }
    // This channel is owned by the callback, or by a task holding its closed
    // gate. Do not call ledc_set_duty here: IDF may wait on a fade semaphore.
    auto& channel = LEDC.channel_group[LEDC_HIGH_SPEED_MODE].channel[this->channel];
    channel.duty.duty = uint32_t(write_pwm) << 4;
    channel.conf1.duty_inc = 1;
    channel.conf1.duty_num = 1;
    channel.conf1.duty_cycle = 1;
    channel.conf1.duty_scale = 0;
    channel.conf0.sig_out_en = 1;
    channel.conf1.duty_start = 1;
    return ret;
}

const float TOTAL_PERIOD_PWM = TOTAL_PERIOD_TIME_US/10; // Per cycle

void InrushControlSolenoid::__write_pwm(float vref_compensation, float temperature_factor) {
    if (this->hold_time != 0) {
        float on_pwm_ratio = 0.35 * vref_compensation;
        if (inrush_time == 0) {
            on_pwm_ratio = 0.5;
        }
        this->pwm_on_time = (int)(TOTAL_PERIOD_PWM * on_pwm_ratio);
        this->pwm_off_time = TOTAL_PERIOD_PWM - this->pwm_on_time;
    }
}

void InrushControlSolenoid::set_duty(uint16_t duty) {
    this->pwm_raw = duty;
    this->pwm = duty;
    if (GPIO_NUM_NC != this->zener_pin) {
        // NEW! TCC Zener mode
        int total_on_time = (float)TOTAL_PERIOD_TIME_US * ((float)duty / 4096.0);
        if (total_on_time < TOTAL_PERIOD_TIME_US/20) { // < 5%
            this->inrush_time = 0;
            this->hold_time = 0;
            this->off_time = TOTAL_PERIOD_TIME_US;
        } else if (total_on_time > TOTAL_PERIOD_TIME_US*0.95) { // > 95%
            this->inrush_time = 0;
            this->hold_time = TOTAL_PERIOD_TIME_US;
            this->off_time = 0;
        } else {
            this->inrush_time = MIN(total_on_time, 25000); // MIN period, 2.5ms (Inrush phase)
            this->hold_time = 0;
            if (total_on_time > this->inrush_time) {
                this->hold_time = total_on_time - this->inrush_time;
            }
            this->off_time = TOTAL_PERIOD_TIME_US - (this->inrush_time + this->hold_time); 
        }
    } else {
        this->period_on_time = ((float)duty / 4096.0) * ((float)TOTAL_PERIOD_TIME_US/2);
        if (this->period_on_time > INRUSH_TIME_US) {
            this->hold_time = this->period_on_time - (INRUSH_TIME_US);
            this->inrush_time = INRUSH_TIME_US;
        } else {
            this->inrush_time = this->period_on_time;
            this->hold_time = 0;
        }
        this->off_time = TOTAL_PERIOD_TIME_US - this->period_on_time;
    }
}
