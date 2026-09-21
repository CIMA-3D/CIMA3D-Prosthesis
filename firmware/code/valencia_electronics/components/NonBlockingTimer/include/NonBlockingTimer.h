#pragma once

#include <cstdint>

class NonBlockingTimer {
    public:
        void setMillis(uint32_t t);
        void setMicros(uint32_t t);
        bool check();
        void delayMicros(uint32_t t);
        void delayMillis(uint32_t t, bool blockCPU = false);
        
    private:
        bool timerFree = true;
        uint32_t waitingTime;
        uint32_t lastTime;
};
