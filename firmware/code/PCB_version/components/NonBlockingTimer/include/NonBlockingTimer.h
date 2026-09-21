#pragma once

#include <cstdint>

class NonBlockingTimer {
    public:
        void setMillis(uint64_t t);
        void setMicros(uint64_t t);
        bool check();
        void delayMicros(uint64_t t);
        void delayMillis(uint64_t t, bool blockCPU = false);
        
    private:
        bool timerFree = true;
        uint64_t waitingTime;
        uint64_t lastTime;
};