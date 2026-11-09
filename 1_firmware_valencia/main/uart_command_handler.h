#pragma once

#include <cstdint>

void beginUart(uint32_t baudrate);
void pollUart(void (*parser)(const char* cmd));
