#include "movements.h"
#include "globals.h"
#include <Arduino.h>
#include <Wire.h>
#include <SparkFunSX1509.h>
#include <Servo.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

SX1509 ioExpander;

Servo wrist(wristPin, LEDC_CHANNEL_0, LEDC_TIMER_0);
Servo wristBrake1(wristBrake1Pin, LEDC_CHANNEL_1, LEDC_TIMER_0);
Servo wristBrake2(wristBrake2Pin, LEDC_CHANNEL_2, LEDC_TIMER_0);
Servo thumbBase(thumbBasePin, LEDC_CHANNEL_3, LEDC_TIMER_0);

// --- IMPLEMENTACIÓN DEL DRIVER DC ---
DcDriver::DcDriver(uint8_t pwmPin, uint8_t dir1Pin, uint8_t dir2Pin, bool dirOnEsp32) {
    _pwmPin = pwmPin;
    _dir1Pin = dir1Pin;
    _dir2Pin = dir2Pin;
    _dirOnEsp32 = dirOnEsp32;
}

void DcDriver::init() {
    // El PWM siempre va al expansor en esta placa
    ioExpander.pinMode(_pwmPin, OUTPUT);
    
    // Las direcciones pueden ir al ESP32 o al Expansor
    if (_dirOnEsp32) {
        pinMode(_dir1Pin, OUTPUT);
        pinMode(_dir2Pin, OUTPUT);
    } else {
        ioExpander.pinMode(_dir1Pin, OUTPUT);
        ioExpander.pinMode(_dir2Pin, OUTPUT);
    }
    stop(); // Aseguramos que arranca parado
}

void DcDriver::forward(uint8_t speed) {
    if (_dirOnEsp32) {
        digitalWrite(_dir1Pin, HIGH);
        digitalWrite(_dir2Pin, LOW);
    } else {
        ioExpander.digitalWrite(_dir1Pin, HIGH);
        ioExpander.digitalWrite(_dir2Pin, LOW);
    }
    ioExpander.analogWrite(_pwmPin, speed);
}

void DcDriver::reverse(uint8_t speed) {
    if (_dirOnEsp32) {
        digitalWrite(_dir1Pin, LOW);
        digitalWrite(_dir2Pin, HIGH);
    } else {
        ioExpander.digitalWrite(_dir1Pin, LOW);
        ioExpander.digitalWrite(_dir2Pin, HIGH);
    }
    ioExpander.analogWrite(_pwmPin, speed);
}

void DcDriver::stop() {
    if (_dirOnEsp32) {
        digitalWrite(_dir1Pin, LOW);
        digitalWrite(_dir2Pin, LOW);
    } else {
        ioExpander.digitalWrite(_dir1Pin, LOW);
        ioExpander.digitalWrite(_dir2Pin, LOW);
    }
    ioExpander.analogWrite(_pwmPin, 0);
}

// Motores 1 al 5: Todo conectado al expansor (false)
static DcDriver finger1(M1_PWM, M1_D1, M1_D2, false); 
static DcDriver finger2(M2_PWM, M2_D1, M2_D2, false); 
static DcDriver finger3(M3_PWM, M3_D1, M3_D2, false); 
static DcDriver finger4(M4_PWM, M4_D1, M4_D2, false); 
static DcDriver finger5(M5_PWM, M5_D1, M5_D2, false); 

// Motor 6 (Punta del Pulgar): PWM en expansor, Direcciones en ESP32 (true)
static DcDriver thumbTipMotor(M6_PWM, M6_D1, M6_D2, true); 

// Array para iterar fácilmente sobre todos los dedos (incluyendo la punta del pulgar)
DcDriver* fingers[6] = {
    &finger1, &finger2, &finger3, &finger4, &finger5, &thumbTipMotor
};


void initMovements() {
    printf("Inicializando sistema de movimientos...\n");

    // 1. Iniciar Bus I2C y Expansor
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    if (!ioExpander.begin(SX1509_ADDR)) {
        printf("ERROR: No se encuentra el expansor SX1509 en la direccion 0x3E\n");
    } else {
        printf("Expansor SX1509 inicializado correctamente.\n");
    }

    // 2. Iniciar reloj interno del expansor para usar el PWM
    ioExpander.clock(2, 1, 0);

    // 3. Iniciar todos los drivers de los dedos
    for (int i = 0; i < 6; i++) {
        fingers[i]->init();
    }

    printf("Motores y Servos inicializados.\n");
}

void openHand() {
    printf("Abriendo mano completa...\n");
    for (int i = 0; i < 6; i++) {
        fingers[i]->reverse(255); // Usamos velocidad máxima (255)
    }
    vTaskDelay(pdMS_TO_TICKS(timeFingersOpening)); 
    stopFingers();
    handState = HandState::OPEN;
}

void closeHand() {
    printf("Cerrando mano completa...\n");
    for (int i = 0; i < 6; i++) {
        fingers[i]->forward(255);
    }
    vTaskDelay(pdMS_TO_TICKS(timeFingersClosing));
    stopFingers();
    handState = HandState::CLOSED;
}

void openFingers() {
    for (int i = 0; i < 5; i++) { // Solo los 5 dedos, ignorando pulgar si es necesario
        fingers[i]->reverse(255);
    }
}

void closeFingers(int v) {
    // Recibe velocidad v (0-255)
    for (int i = 0; i < 5; i++) {
        fingers[i]->forward(v);
    }
}

void stopFingers() {
    for (int i = 0; i < 6; i++) {
        fingers[i]->stop(); // CORREGIDO: ahora usa stop()
    }
}

void pinch() {
    printf("Ejecutando agarre Pinch...\n");
    finger2.forward(255);       // Índice
    thumbTipMotor.forward(255); // Punta del pulgar
    
    moveThumbBase(thumbBasePinched);
    
    vTaskDelay(pdMS_TO_TICKS(timePinching));
    stopFingers();
    handState = HandState::PINCHED;
}

void unpinch() {
    printf("Soltando agarre Pinch...\n");
    finger2.reverse(255);
    thumbTipMotor.reverse(255);
    moveThumbBase(thumbBaseUnpinched);
    
    vTaskDelay(pdMS_TO_TICKS(timePinching));
    stopFingers();
    handState = HandState::UNPINCHED;
}

void controlledPinch(int v) {
    finger2.forward(v);
    thumbTipMotor.forward(v);
}

void rotateWrist(int pos) {
    int us = map(pos, 0, 100, 500, 2500); 
    wrist.writeMicroseconds(us);
}

void moveThumbBase(int pos) {
    int us = map(pos, 0, 100, 500, 2500);
    thumbBase.writeMicroseconds(us);
}

void moveThumbTip(int pos) {
    if (pos > 0) {
        thumbTipMotor.forward(pos);
    } else if (pos < 0) {
        thumbTipMotor.reverse(abs(pos));
    } else {
        thumbTipMotor.stop();
    }
}