#include "NonBlockingTimer.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void NonBlockingTimer::setMillis(uint64_t t) {
    waitingTime = t * 1000;
    lastTime = esp_timer_get_time();
    timerFree = false;
}

void NonBlockingTimer::setMicros(uint64_t t) {
    waitingTime = t;
    lastTime = esp_timer_get_time();
    timerFree = false;
}

bool NonBlockingTimer::check() {
    if (esp_timer_get_time() - lastTime >= waitingTime) timerFree = true;
    return timerFree;
}

// NOT TESTED
void NonBlockingTimer::delayMicros(uint64_t t) {
    setMicros(t);
    while (!check());
}

void NonBlockingTimer::delayMillis(uint64_t t, bool blockCPU) {
    if (!blockCPU)
        vTaskDelay(pdMS_TO_TICKS(t));
    else {
        setMillis(t);
        while (!check());
    }
}