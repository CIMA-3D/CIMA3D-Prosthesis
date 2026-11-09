#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "EMGController.h"

// Todos los pines: 0-21, 35-42, 45-48

// ADC1: 1-10
// ADC2: 11-20
// Touch: 1-14
// PWM: all

// Pines que mejor no usar:
// 0, 45, 46  boot, log, y no sé qué SEGURO
// 19, 20     USB data (CDC?)
// 48         LED_BUILTIN (RBG). No hay led simple (como 13 en Arduino)
// 1, 3       USB Serial Debug. Se pueden usar

// ADC only up to 3.3 V (4095)
// ADC2 is used by the Wi-Fi module, so we cannot use the ADC2 pins when Wi-Fi is enabled
// Advanced ADC functions: https://www.luisllamas.es/en/esp32-adc/


//---------------------------------------------PINS-------------------------------------------------------------
// inline constexpr uint8_t emgPins[] =    {1,
//                                          2,
//                                          3,4,
//                                          5,6,7,8,9,10
//                                         };

// inline constexpr size_t numEmgChannels = sizeof(emgPins) / sizeof(emgPins[0]);


// inline constexpr uint8_t temperatureSensorPin      = 11;
// inline constexpr uint8_t batterySensorPin          = 12;

// inline constexpr uint8_t fingersSpeedPins[2]    = {21,35};
// inline constexpr uint8_t fingersDirPins1[2]         = {36,37};
// inline constexpr uint8_t fingersDirPins2[2]         = {38,39};
// inline constexpr uint8_t thumbBasePin             = 40;
// inline constexpr uint8_t thumbTipPin            = 41;
// inline constexpr uint8_t wristPin                = 42;
// inline constexpr uint8_t wristBrake1Pin          = 47;
// inline constexpr uint8_t wristBrake2Pin          = 18;



// Protesis actual

inline constexpr uint8_t emgPins[] = {2, 4};


inline constexpr size_t numEmgChannels = sizeof(emgPins) / sizeof(emgPins[0]);


inline constexpr uint8_t temperatureSensorPin      = 19;
inline constexpr uint8_t batterySensorPin          = 5;

inline constexpr uint8_t fingersSpeedPins[2]    = {11,14};
inline constexpr uint8_t fingersDirPins1[2]         = {12,15};
inline constexpr uint8_t fingersDirPins2[2]         = {13,16};
inline constexpr uint8_t wristPin                = 21;
inline constexpr uint8_t wristBrake1Pin          = 35;
inline constexpr uint8_t wristBrake2Pin          = 36;
// --- PINES DEL DRV8871 (Pulgar Lineal) ---
inline constexpr uint8_t thumbIn1Pin = 9;
inline constexpr uint8_t thumbIn2Pin = 46;



//----------------------------------------POSICIONES Y TIMINGS-------------------------------------------
// DEDOS
inline constexpr int timeFingersClosing = 2000; // controla la posición
inline constexpr int timePinch = 2000; // controla la posición

inline constexpr int timeFingersOpening = 2000; // sólo para la coordinación

inline constexpr int timePinching = 1600; // para control
// inline constexpr int timePinching = 5000;

// --- TIEMPOS DEL PULGAR LINEAL ---
inline constexpr int timeThumbExtending = 1500; // 1.5s para salir (lo ajustaremos)
inline constexpr int timeThumbRetracting = 1500; // 1.5s para entrar




//---------------------------------------MUÑECA---------------------------------------------------------------
inline constexpr int timeWristUnitRotation = 15; // tiempo ente cada unidad (100 en total)
inline constexpr int timeWristBlocking = 500; // tiempo de movimiento del freno
inline constexpr int timeWristUnblocking = 1000;
inline constexpr int wristBrakeBlocked = 65; // posición de bloqueo (de 0 a 100)
inline constexpr int wristBrakeUnblocked = 30;


//---------------------------------------SENSORES--------------------------------------------------------------
// SENSOR TEMPERATURA
inline constexpr int thresholdTemperature = 40;  // Valor (en ºC) a partir del cual se enciende el ventilador. (el sensor da 10mV / ºC)

// SENSOR TENSION BATERIA
inline constexpr float thresholdBatteryMedium = 3.7;  // valor de batería media (3.9 V)
inline constexpr float thresholdBatteryLow = 3.5;   // valor de batería baja (3.6 V)
inline constexpr float voltageDividerRatio = 0.32;
// En teoría con la batería cargada son unos 4.2 V



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