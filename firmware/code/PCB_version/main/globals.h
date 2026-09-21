#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "EMGController.h"



// --- BUS I2C Y DIRECCIONES ---
inline constexpr uint8_t I2C_SDA_PIN = 9;
inline constexpr uint8_t I2C_SCL_PIN = 8;
inline constexpr uint8_t SX1509_ADDR = 0x3E;
inline constexpr uint8_t ADS1115_ADDR = 0x48; // (Asumiendo 0x48 por defecto para ADC I2C)

// --- SENSORES EMG (ESP32) ---
inline constexpr uint8_t emgPins[] = {6, 7}; 
inline constexpr size_t numEmgChannels = sizeof(emgPins) / sizeof(emgPins[0]);

// --- SENSORES GENERALES (ESP32) ---
inline constexpr uint8_t temperatureSensorPin = 19; // No sale explícito en esquema, mantenemos 19
inline constexpr uint8_t batterySensorPin     = 2;  

// --- SERVOS Y FRENOS (ESP32) ---
inline constexpr uint8_t wristPin       = 14; 
inline constexpr uint8_t wristBrake1Pin = 15; 
inline constexpr uint8_t wristBrake2Pin = 16; 
inline constexpr uint8_t thumbBasePin   = 40; // OJO: No aparece en esquema. Mantengo pin libre.

// --- CONFIGURACIÓN DE MOTORES DC (M1 - M6) ---
constexpr int numFingers = 6; 

// Motor 1 (Todo en Expansor)
inline constexpr uint8_t M1_PWM = 14; // ENA1
inline constexpr uint8_t M1_D1  = 8;  // I1
inline constexpr uint8_t M1_D2  = 7;  // I2

// Motor 2 (Todo en Expansor)
inline constexpr uint8_t M2_PWM = 11; // ENB1
inline constexpr uint8_t M2_D1  = 6;  // I3
inline constexpr uint8_t M2_D2  = 5;  // I4

// Motor 3 (Todo en Expansor)
inline constexpr uint8_t M3_PWM = 13; // ENA2
inline constexpr uint8_t M3_D1  = 4;  // I5
inline constexpr uint8_t M3_D2  = 3;  // I6

// Motor 4 (Todo en Expansor)
inline constexpr uint8_t M4_PWM = 10; // ENB2
inline constexpr uint8_t M4_D1  = 2;  // I7
inline constexpr uint8_t M4_D2  = 1;  // I8

// Motor 5 (Todo en Expansor)
inline constexpr uint8_t M5_PWM = 12; // ENA3
inline constexpr uint8_t M5_D1  = 0;  // I9
inline constexpr uint8_t M5_D2  = 15; // I10

// Motor 6 / Pulgar Tip (MIXTO: PWM Expansor, Dir ESP32)
inline constexpr uint8_t M6_PWM = 9;  // ENB3 (Expansor)
inline constexpr uint8_t M6_D1  = 1;  // I11 (ESP32)
inline constexpr uint8_t M6_D2  = 13; // I12 (ESP32)



// DEDOS
inline constexpr int timeFingersClosing = 1000;
inline constexpr int timePinch = 1000;
inline constexpr int timeFingersOpening = 1000;
inline constexpr int timeThumbOpening = 500;
inline constexpr int timeThumbClosing = 500;
inline constexpr int timePinching = 1600;

// PULGAR (0 a 100)
inline constexpr int thumbBaseOpen = 0, thumbTipOpen = 0;
inline constexpr int thumbBaseClosed = 100, thumbTipClosed = 50;
inline constexpr int thumbBasePinched = 100, thumbTipPinched = 75;
inline constexpr int thumbBaseUnpinched = 100, thumbTipUnpinched = 50;

// MUÑECA
inline constexpr int timeWristUnitRotation = 15;
inline constexpr int timeWristBlocking = 500;
inline constexpr int timeWristUnblocking = 1000;
inline constexpr int wristBrakeBlocked = 65;
inline constexpr int wristBrakeUnblocked = 30;

// SENSORES DE CORRIENTE Y UMBRALES
inline constexpr int thresholdTemperature = 40; 
inline constexpr float thresholdBatteryMedium = 3.7;
inline constexpr float thresholdBatteryLow = 3.3;
inline constexpr float voltageDividerRatio = 0.25;
inline constexpr float collisionThresholdVoltage = 1.5;

// (Nota: Los pines de tacto ahora usan canales del ADC 0-3 y el M5 usa el GPIO 5)
// Dejo esto comentado para que cuando programes el tacto recuerdes la separación:
// M1 a M4 -> Canales ADC externo (0, 1, 2, 3)
// M5      -> ESP32 GPIO 5 (VIOUT5)

// --- ESTADOS Y OBJETOS GLOBALES ---
enum BatteryState { BAT_LOW, BAT_MEDIUM, BAT_HIGH };
extern BatteryState currentBatteryState;

extern float temperature;
extern float batteryLevel;

enum class HandState {
    CLOSED,
    OPEN,
    PINCHED,
    UNPINCHED
};
extern HandState handState;

extern EMGController *emgs[numEmgChannels];
extern uint16_t rawEmgSampleBuffer[numEmgChannels];

#define EMG_ENVELOPE_BUFFER_SIZE 8

struct Sample {
    float samples[numEmgChannels];
    uint16_t rawSamples[numEmgChannels];
};
extern QueueHandle_t emgSampleQueue;

struct EmgEnvelopeBuffer {
    int index;
    float values[numEmgChannels][EMG_ENVELOPE_BUFFER_SIZE];
    bool flag;
};
extern EmgEnvelopeBuffer emgEnvelopeBuffer;

extern volatile bool emgSampleQueueFull;
extern bool emgCalibrated;