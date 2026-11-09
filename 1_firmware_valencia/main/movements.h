#pragma once

#include "Servo.h"
#include "PwmController.h"

class DcDriver {
    public:
        DcDriver(uint8_t pwmPin, uint8_t dir1Pin, uint8_t dir2Pin, ledc_channel_t pwmChannel, ledc_timer_t timer);

        void forward(uint8_t speed);
        void reverse(uint8_t speed);
        void stop();

    private:
        PwmController _pwm;
        gpio_num_t _dir1Pin, _dir2Pin;
};

extern DcDriver* fingers[];

void initMovements();
void updateMovements();

void openHand();
void closeHand();
void openFingers();
void closeFingers(int v);
void pinch();
void unpinch();
void stopFingers();
void controlledPinch(int v);

void testLinearThumb();

void rotateWrist(int pos);
void blockWrist();
void unblockWrist();


