#include "include.h"
// use 12 bit precision for LEDC timer
#define LEDC_TIMER_12_BIT 12

// use 5000 Hz as a LEDC base frequency
#define LEDC_BASE_FREQ 1000

// define starting duty, target duty and maximum fade time
#define LEDC_START_DUTY (0)
#define LEDC_TARGET_DUTY (4095)
static const char *TAG = "HAL"; // Thẻ (tag) cho log
pwm_pin_t EX_HAL::pwm_pin_data[MAX_EX_PIN];
EX_HAL::EX_HAL()
{
}
void EX_HAL::init_hal()
{
    i2s_init();
}
// Hàm thiết lập chu kỳ PWM (duty) cho chân cụ thể
void EX_HAL::set_pwm_duty(int pin, int duty) {
    if (pin > 127) {
        const uint8_t pinlo = pin & 0x7F;
        pwm_pin_t &pindata = pwm_pin_data[pinlo];
        
        // Tính toán số lượng ticks PWM dựa trên chu kỳ duty
        const uint32_t duty_ticks = (pindata.pwm_cycle_ticks * duty) / 255;
        pindata.pwm_duty_ticks = duty_ticks;
        
        ESP_LOGI(TAG, "Set duty: pin: %d, duty: %d, pwm_duty_ticks: %d", pin, duty, duty_ticks);
    }
    else {
        // Sử dụng LEDC driver để thiết lập duty cho PWM
        ledcWrite(pin, duty);
    }
}
void EX_HAL::set_pwm_frequency(int pin, uint32_t f)
{
    if (pin > 127)
    {
        // Tính toán chu kỳ PWM dựa trên tần số (f)
        pwm_pin_data[pin & 0x7F].pwm_cycle_ticks = 1000000UL / f / 4; // Số 4µs tick trong một chu kỳ đầy đủ của PWM
        ESP_LOGI(TAG, "pin: %d, pwm_cycle_ticks: %d, pwm_duty_ticks: %d, pwm_tick_count: %d",
                 pin,
                 pwm_pin_data[pin & 0x7F].pwm_cycle_ticks,
                 pwm_pin_data[pin & 0x7F].pwm_duty_ticks,
                 pwm_pin_data[pin & 0x7F].pwm_tick_count);
        return;
    }

    // Nếu pin <= 127, thay đổi tần số PWM sử dụng LEDC driver
    ledcChangeFrequency(pin, f, LEDC_TIMER_12_BIT);
}
// Hàm thiết lập giá trị PWM cho chân cụ thể với chu kỳ duty và tần số f
void EX_HAL::set_pwm(int pin, int duty, uint32_t f)
{
    EX_HAL::set_pwm_frequency(pin, f); // Thiết lập tần số PWM
    EX_HAL::set_pwm_duty(pin, duty);   // Thiết lập chu kỳ PWM (duty cycle)
}
