#pragma once

#include "driver/ledc.h"
// #include "iot_servo.h"

class Servo {
public:
    Servo(uint8_t gpio, ledc_channel_t channel, ledc_timer_t timer);
    
    void writeMicroseconds(uint32_t us);

private:
    uint8_t _gpio;
    ledc_channel_t _ledcChannel;
    ledc_timer_t _ledcTimer;
    static const ledc_mode_t _speedMode = LEDC_LOW_SPEED_MODE;
    static const ledc_timer_bit_t _resolution = LEDC_TIMER_12_BIT;
    static const uint32_t _freq_hz = 50;

    static void initTimerOnce(ledc_timer_t timer, uint32_t freq_hz, ledc_timer_bit_t resolution, ledc_mode_t speedMode);
    static bool _timersInitialized[LEDC_TIMER_MAX];
};