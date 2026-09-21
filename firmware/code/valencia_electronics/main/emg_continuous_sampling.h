#pragma once

#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern bool ovf;

void initAdc(uint16_t* buffer, TaskHandle_t *callbackTaskHandle);
void readAdc();
