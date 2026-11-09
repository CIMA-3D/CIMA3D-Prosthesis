#include "secondary_sensors.h"

#include "globals.h"
#include "NonBlockingTimer.h"
#include "math.h"

static AdcSensor* temperatureSensor;
static AdcSensor* batterySensor;

void initSecondarySensors() {
    temperatureSensor = new AdcSensor(temperatureSensorPin);
    batterySensor = new AdcSensor(batterySensorPin);
}

void readSecondarySensors() {
    // LA ESP LEE HASTA 3.1 V. Verificar ques esté configurada?
    static float kT = 1.0/10.0; // 10 mV / ºC
    static float kB = 1.0/voltageDividerRatio;
    static NonBlockingTimer timer;
    static uint16_t samplingInterval = 10; // millis
    // setting time = -5/ln(alpha) * samplingInterval (95%)
    static float alphaT = exp(-5.0 * samplingInterval / 2000.0); // 2 seconds setting time (95%)
    static float alphaB = exp(-5.0 * samplingInterval / 2000.0);

    if (timer.check()) {
        temperature = alphaT * temperature + (1 - alphaT) * temperatureSensor->read() * kT;
        //batteryLevel = alphaB * batteryLevel + (1 - alphaB) * batterySensor->read() * kB;
        float rawBat = batterySensor->read();
        printf("RAW BAT: %.3f\n", rawBat);
        batteryLevel = alphaB * batteryLevel + (1 - alphaB) * rawBat * kB;

        timer.setMillis(samplingInterval);
    }
}

void checkBatteryLevel() {
    enum State {LOW, MEDIUM, HIGH};
    static float lastBatteryLevel = batteryLevel;
    static State state;
    static NonBlockingTimer timer;
    static bool drop = false;

    if (abs(lastBatteryLevel-batteryLevel) < 0.05) {
        if (state == HIGH && batteryLevel <= thresholdBatteryLow) state = LOW;
        else if (state == HIGH && batteryLevel <= thresholdBatteryMedium) state = MEDIUM;

        else if (state == MEDIUM && batteryLevel <= thresholdBatteryLow) state = LOW;
        else if (state == MEDIUM && batteryLevel > thresholdBatteryMedium + 0.2) state = HIGH;

        else if (state == LOW && batteryLevel > thresholdBatteryMedium) state = HIGH;
        else if (state == LOW && batteryLevel > thresholdBatteryLow + 0.2) state = MEDIUM;

    lastBatteryLevel = batteryLevel;
    drop = false;
    } else if (!drop) {
        drop = true;
        timer.setMillis(5000);
    } else if (drop && timer.check()) {
        drop = false;
        lastBatteryLevel = batteryLevel;
    }


    // if (state == LOW) {
    //   digitalWrite(ledVerdePin, LOW);
    //   digitalWrite(ledAmarilloPin, LOW);
    //   digitalWrite(ledRojoPin, HIGH);
    // } else if (state == MEDIUM) {
    //   digitalWrite(ledVerdePin, LOW);
    //   digitalWrite(ledAmarilloPin, HIGH);
    //   digitalWrite(ledRojoPin, LOW);
    // } else if (state == HIGH) {
    //   digitalWrite(ledVerdePin, HIGH);
    //   digitalWrite(ledAmarilloPin, LOW);
    //   digitalWrite(ledRojoPin, LOW);
    // }
}
