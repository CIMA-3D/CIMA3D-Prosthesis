#pragma once

#include <stdint.h>


class DcDriver {
public:
    DcDriver(uint8_t pwmPin, uint8_t dir1Pin, uint8_t dir2Pin, bool dirOnEsp32);
    void init();
    void forward(uint8_t speed);
    void reverse(uint8_t speed);
    void stop();

private:
    uint8_t _pwmPin;
    uint8_t _dir1Pin;
    uint8_t _dir2Pin;
    bool _dirOnEsp32;
};

// Arreglo global de 6 motores (5 dedos + punta del pulgar)
extern DcDriver* fingers[6]; 

void initMovements();
void openHand();
void closeHand();
void openFingers();
void closeFingers(int v);
void stopFingers();
void pinch();
void unpinch();
void controlledPinch(int v);
void rotateWrist(int pos);
void moveThumbBase(int pos);
void moveThumbTip(int pos);