#include "inrush_solenoid.h"
#include "esp_check.h"
#include "tcu_maths.h"
#include "soc/gpio_struct.h"
#include "soc/ledc_struct.h"

// AT 12.0V
const DRAM_ATTR uint16_t INRUSH_START_PWM = 224; // Any PWM below this will just write 0 to solenoid (Not enough open time for arm to move)
const DRAM_ATTR uint16_t INRUSH_SKIP_PWM = 3220; // Any PWM above this will skip inrush and just go to hold as there is enough current
const DRAM_ATTR uint16_t INRUSH_TIME_US = 15000; 
const DRAM_ATTR uint16_t INRUSH_PWM = 4096;
const DRAM_ATTR uint16_t HOLD_PWM = 1300;
const DRAM_ATTR uint32_t TOTAL_PERIOD_TIME_US = 100000; // Timer runs at 10MHz, Hydralic PWM is 100Hz, so 10_000_000/100

static DRAM_ATTR portMUX_TYPE callback_mux = portMUX_INITIALIZER_UNLOCKED;
static DRAM_ATTR bool callback_blocked = true;

static bool IRAM_ATTR inrush_solenoid_timer_isr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    portENTER_CRITICAL_ISR(&callback_mux);
    if (!callback_blocked) {
        InrushControlSolenoid* solenoid = reinterpret_cast<InrushControlSolenoid*>(user_data);
        uint32_t next_alarm_in = MAX(solenoid->on_timer_interrupt(), 1u);
        gptimer_alarm_config_t alarm_config = {
            .alarm_count = edata->alarm_value + next_alarm_in,
            .reload_count = 0u,
            .flags = {
                .auto_reload_on_alarm = 0u
            }
        };
        gptimer_set_alarm_action(timer, &alarm_config);
    }
    portEXIT_CRITICAL_ISR(&callback_mux);
    return false;
}

static bool IRAM_ATTR inrush_solenoid_timer_isr_new(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    portENTER_CRITICAL_ISR(&callback_mux);
    if (!callback_blocked) {
        InrushControlSolenoid* solenoid = static_cast<InrushControlSolenoid*>(user_data);
        uint32_t next_alarm_in = MAX(solenoid->on_timer_interrupt_new(), 1u);
        gptimer_alarm_config_t alarm_config = {
            .alarm_count = edata->alarm_value + next_alarm_in,
            .reload_count = 0u,
            .flags = {
                .auto_reload_on_alarm = 0u
            }
        };
        gptimer_set_alarm_action(timer, &alarm_config);
    }
    portEXIT_CRITICAL_ISR(&callback_mux);
    return false;
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
        callback = inrush_solenoid_timer_isr_new;
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
                } else {
                    ESP_LOGE("ICSolenoid", "ICSolenoid %s gptimer_enable failed: %s", this->name, esp_err_to_name(this->ready));
                }
            }
        }
    }
}

void InrushControlSolenoid::pre_current_test() {
    this->isr_disable();
}

void InrushControlSolenoid::post_current_test() {
    this->isr_enable();
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

void InrushControlSolenoid::start_timer() {
    portENTER_CRITICAL(&callback_mux);
    if (this->timer_started) {
        portEXIT_CRITICAL(&callback_mux);
        return;
    }
    this->timer_started = true;
    const bool should_start = (0 == this->isr_disable_depth);
    if (should_start) {
        this->phase_id = 0;
        callback_blocked = false;
    }
    portEXIT_CRITICAL(&callback_mux);
    if (should_start) {
        uint64_t count = 0;
        gptimer_get_raw_count(this->timer, &count);
        gptimer_alarm_config_t alarm_config = {
            .alarm_count = count + TOTAL_PERIOD_TIME_US,
            .reload_count = 0u,
            .flags = { .auto_reload_on_alarm = 0u }
        };
        gptimer_set_alarm_action(this->timer, &alarm_config);
        esp_err_t res = gptimer_start(this->timer);
        if (ESP_OK != res) {
            portENTER_CRITICAL(&callback_mux);
            callback_blocked = true;
            portEXIT_CRITICAL(&callback_mux);
            this->force_output_low();
            ESP_LOGE("ICSolenoid", "ICSolenoid %s gptimer_start failed: %s", this->name, esp_err_to_name(res));
        }
    }
}

void InrushControlSolenoid::isr_disable() {
    portENTER_CRITICAL(&callback_mux);
    this->isr_disable_depth++;
    callback_blocked = true;
    const bool should_stop = (1 == this->isr_disable_depth) && this->timer_started;
    portEXIT_CRITICAL(&callback_mux);
    if (should_stop) {
        gptimer_stop(this->timer);
        this->force_output_low();
    }
}

void InrushControlSolenoid::isr_enable() {
    portENTER_CRITICAL(&callback_mux);
    if (0 == this->isr_disable_depth) {
        portEXIT_CRITICAL(&callback_mux);
        return;
    }
    this->isr_disable_depth--;
    const bool should_start = (0 == this->isr_disable_depth) && this->timer_started;
    if (should_start) {
        this->phase_id = 0;
        callback_blocked = false;
    }
    portEXIT_CRITICAL(&callback_mux);
    if (should_start) {
        uint64_t count = 0;
        gptimer_get_raw_count(this->timer, &count);
        gptimer_alarm_config_t alarm_config = {
            .alarm_count = count + TOTAL_PERIOD_TIME_US,
            .reload_count = 0u,
            .flags = { .auto_reload_on_alarm = 0u }
        };
        gptimer_set_alarm_action(this->timer, &alarm_config);
        esp_err_t res = gptimer_start(this->timer);
        if (ESP_OK != res) {
            portENTER_CRITICAL(&callback_mux);
            callback_blocked = true;
            portEXIT_CRITICAL(&callback_mux);
            this->force_output_low();
            ESP_LOGE("ICSolenoid", "ICSolenoid %s gptimer_start failed: %s", this->name, esp_err_to_name(res));
        }
    }
}

bool InrushControlSolenoid::is_disabled() {
    portENTER_CRITICAL(&callback_mux);
    const bool disabled = callback_blocked;
    portEXIT_CRITICAL(&callback_mux);
    return disabled;
}

DRAM_ATTR bool on = false;
DRAM_ATTR bool pwm_on = false;
DRAM_ATTR bool zener_on = false;
DRAM_ATTR uint32_t total  = 0;
DRAM_ATTR bool pwm_en = false;
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

            uint32_t remaining = (total < this->hold_time) ? (this->hold_time - total) : 0;
            ret = MIN(!pwm_en ? this->pwm_on_time : this->pwm_off_time, remaining);
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
    // Direct hardware register write inside ISR; ledc_set_duty may take a fade semaphore.
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
