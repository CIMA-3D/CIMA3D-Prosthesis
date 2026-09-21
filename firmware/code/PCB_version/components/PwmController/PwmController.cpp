#include "PwmController.h"

bool PwmController::_timersInitialized[LEDC_TIMER_MAX] = { false };

PwmController::PwmController(uint8_t pin, ledc_channel_t channel, ledc_timer_t timer)
    : _pin(pin), _channel(channel), _timer(timer) {

    initTimerOnce(timer, _freq_hz, _resolution, _speedMode);

    ledc_channel_config_t chan_conf = {};
    chan_conf.gpio_num              = _pin;
    chan_conf.speed_mode            = _speedMode;
    chan_conf.channel               = _channel;
    chan_conf.timer_sel             = _timer;
    chan_conf.duty                  = 0;
    chan_conf.hpoint                = 0;
    chan_conf.flags.output_invert   = 0;

    ledc_channel_config(&chan_conf);

    write(0);
}

void PwmController::initTimerOnce(ledc_timer_t timer, uint32_t freq_hz, ledc_timer_bit_t resolution, ledc_mode_t speedMode) {
    if (!_timersInitialized[timer]) {
        ledc_timer_config_t timer_conf = {};
        timer_conf.speed_mode       = speedMode;
        timer_conf.timer_num        = timer;
        timer_conf.duty_resolution  = resolution;
        timer_conf.freq_hz          = freq_hz;
        timer_conf.clk_cfg          = LEDC_AUTO_CLK;

        ledc_timer_config(&timer_conf);
        _timersInitialized[timer] = true;
    }
}

void PwmController::write(uint32_t duty) {
    if (duty > ((1 << 8) - 1)) duty = (1 << 8) - 1; // clamp for 8-bit resolution (change if you use different)
    ledc_set_duty(_speedMode, _channel, duty);
    ledc_update_duty(_speedMode, _channel);
}

void PwmController::stop() {
    write(0); 
}