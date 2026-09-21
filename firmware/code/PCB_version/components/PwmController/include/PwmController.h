#pragma once

#include "driver/ledc.h"

class PwmController {
public:
    PwmController(uint8_t pin, ledc_channel_t channel, ledc_timer_t timer);

    void write(uint32_t duty); // duty 0-255 (or max depends on resolution)
    void stop();

private:
    uint8_t _pin;
    ledc_channel_t _channel;
    ledc_timer_t _timer;
    static const ledc_mode_t _speedMode = LEDC_LOW_SPEED_MODE;
    static const ledc_timer_bit_t _resolution = LEDC_TIMER_8_BIT;
    static const uint32_t _freq_hz = 1000;

    static bool _timersInitialized[LEDC_TIMER_MAX];
    static void initTimerOnce(ledc_timer_t timer, uint32_t freq_hz, ledc_timer_bit_t resolution, ledc_mode_t speedMode);
};