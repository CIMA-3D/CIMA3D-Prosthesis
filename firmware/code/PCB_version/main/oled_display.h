#pragma once

// Este bloque es vital en ESP-IDF cuando mezclamos C y C++
#ifdef __cplusplus
extern "C" {
#endif

void initOLED();
void updateOLED(const char* handStateStr, int batteryLevel);
void showMessageOLED(const char* message);

#ifdef __cplusplus
}
#endif