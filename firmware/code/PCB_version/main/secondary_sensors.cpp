#include "secondary_sensors.h"
#include "AdcSingleRead.h" 
#include "globals.h"
#include "movements.h"
#include "NonBlockingTimer.h"
#include <Adafruit_ADS1X15.h> 



// OBJETOS FISICOS SEGUN TU PCB
Adafruit_ADS1115 ads;          // El chip para motores 1, 2, 3 y 4
AdcSensor* sensorMotor5;       // Sensor directo al GPIO 5
AdcSensor* batterySensor;         // Sensor batería al GPIO 2

void initSecondarySensors() {
    // 1. Iniciar I2C para el ADC externo (Pines 8 y 9 configurados en el setup principal)
    if (!ads.begin(0x48)) {
        printf("ERROR: ADS1115 no encontrado en 0x48\n");
    }

    // 2. Iniciar sensores internos de la ESP32
    sensorMotor5 = new AdcSensor(5); // VIOUT5 -> GPIO 5
    batterySensor   = new AdcSensor(2);   // VBAT   -> GPIO 2
}

float leerVoltajeMotor(int id) {
    if (id >= 0 && id <= 3) {
        // Canales 0, 1, 2, 3 del ADS1115
        int16_t raw = ads.readADC_SingleEnded(id);
        return ads.computeVolts(raw);
    } else if (id == 4) {
        // Motor 5 está en el pin interno
        return sensorMotor5->read();
    }
    return 0.0;
}

void checkTouchFeedback() {
    for (int i = 0; i < 5; i++) {
        float voltios = leerVoltajeMotor(i);
        
        // Si el voltaje sube, hay presión (choque)
        if (voltios > collisionThresholdVoltage) {
            if (fingers[i] != nullptr) {
                fingers[i]->stop(); // Ahora sí funcionará porque lo añadimos en el Paso 1
                printf("Dedo %d DETENIDO por contacto (%.2fV)\n", i, voltios);
            }
        }
    }
}

// --- LÓGICA DE BATERÍA (Mantenemos la tuya que ya funcionaba bien) ---
BatteryState currentBatteryState = BAT_HIGH; 

void readSecondarySensors() {
    static float kB = 1.0/voltageDividerRatio; // Asegúrate de tener voltageDividerRatio en globals
    static NonBlockingTimer timer;
    static uint16_t samplingInterval = 10; 
    static float alphaB = exp(-5.0 * samplingInterval / 2000.0);

    // Simplificado temporalmente solo a batería para aislar errores.
    // Si usas el de temperatura, añádelo aquí con la misma lógica.
    if (timer.check()) {
        batteryLevel = alphaB * batteryLevel + (1 - alphaB) * batterySensor->read() * kB;
        timer.setMillis(samplingInterval);
    }
}

void checkBatteryLevel() {
    static float lastBatteryLevel = batteryLevel;
    static NonBlockingTimer timer;
    static bool drop = false;

    if (fabs(lastBatteryLevel - batteryLevel) < 0.05) {
        if (currentBatteryState == BAT_HIGH && batteryLevel <= thresholdBatteryLow) currentBatteryState = BAT_LOW;
        else if (currentBatteryState == BAT_HIGH && batteryLevel <= thresholdBatteryMedium) currentBatteryState = BAT_MEDIUM;
        else if (currentBatteryState == BAT_MEDIUM && batteryLevel <= thresholdBatteryLow) currentBatteryState = BAT_LOW;
        else if (currentBatteryState == BAT_MEDIUM && batteryLevel > thresholdBatteryMedium + 0.2) currentBatteryState = BAT_HIGH;
        else if (currentBatteryState == BAT_LOW && batteryLevel > thresholdBatteryMedium) currentBatteryState = BAT_HIGH;
        else if (currentBatteryState == BAT_LOW && batteryLevel > thresholdBatteryLow + 0.2) currentBatteryState = BAT_MEDIUM;

        lastBatteryLevel = batteryLevel;
        drop = false;
    } else if (!drop) {
        drop = true;
        timer.setMillis(5000);
    } else if (drop && timer.check()) {
        drop = false;
        lastBatteryLevel = batteryLevel;
    }
}