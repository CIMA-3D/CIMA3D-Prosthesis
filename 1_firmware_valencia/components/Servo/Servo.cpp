#include "Servo.h"

bool Servo::_timersInitialized[LEDC_TIMER_MAX] = { false };

Servo::Servo(uint8_t gpio, ledc_channel_t channel, ledc_timer_t timer)
    : _gpio(gpio), _ledcChannel(channel), _ledcTimer(timer) {

    initTimerOnce(timer, _freq_hz, _resolution, _speedMode);

    uint32_t duty = ledc_get_duty(_speedMode,_ledcChannel);

    ledc_channel_config_t chan_conf = {};
    chan_conf.gpio_num              = _gpio;
    chan_conf.speed_mode            = _speedMode;
    chan_conf.channel               = _ledcChannel;
    chan_conf.timer_sel             = _ledcTimer;
    // chan_conf.duty                  = 0;
    chan_conf.duty                  = duty; // as in Arduino
    chan_conf.hpoint                = 0;
    chan_conf.intr_type             = LEDC_INTR_DISABLE;
    chan_conf.flags.output_invert   = 0;
    
    ledc_channel_config(&chan_conf);
}

void Servo::initTimerOnce(ledc_timer_t timer, uint32_t freq_hz, ledc_timer_bit_t resolution, ledc_mode_t speedMode) {
    if (!_timersInitialized[timer]) {
        ledc_timer_config_t timer_conf = {};
        timer_conf.speed_mode       = speedMode;
        timer_conf.timer_num        = timer;
        timer_conf.duty_resolution  = resolution;
        timer_conf.freq_hz          = freq_hz;
        // timer_conf.clk_cfg          = LEDC_AUTO_CLK;
        timer_conf.clk_cfg          = LEDC_USE_XTAL_CLK; // as in Arduino

        ledc_timer_config(&timer_conf);
        _timersInitialized[timer] = true;
    }
}

void Servo::writeMicroseconds(uint32_t us) {
    uint16_t duty = (us * 4095) / 20000;    // Map 1000-2000 µs to 0-4095 (12-bit)
    ledc_set_duty(_speedMode, _ledcChannel, duty);
    ledc_update_duty(_speedMode, _ledcChannel);
    // ledc_set_duty_and_update(_speedMode, _ledcChannel, duty, 0);
}