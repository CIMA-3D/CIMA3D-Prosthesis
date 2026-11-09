#pragma once

#include <cstdint>

class EMGController {
    public:
        EMGController(uint8_t id, uint16_t* inputSamplePointer);
        void applyPowerLineFilter();
        void applyULPFilter();
        void applyLPFilter();
        void normalize();
        void update();
        void resetVars();
        void updateSmooth();
        
        void setSquareEnvelope(bool square) {
            _squareEnvelope = square; }
        void setThreshold(float threshold) {
            _threshold = threshold; }
        void setMaxDuration(int maxDuration) {
            _pulseVars.maxDuration = maxDuration; }
        void setHysteresis(int hysteresis) {
            _pulseVars.hysteresis = hysteresis; }
        void setFreezeDuration(int freezeDuration) {
            _pulseVars.freezeDuration = freezeDuration; }
        float getFilteredSample() {
            return _sample; }
        float getEnvelope() {
            return _envelope; }
        float getSmooth() {
            return _smooth; }
        bool getActive() {
            return _active; }

        bool pulse; // will be set to true when a pulse FINISHES, and has to be reset by the user
        bool mavFlag; // will be set to true when the MAV is ready to be read, and has to be reset by the user

    private:
        struct PLFilter { // 50 Hz comb
            float xBuffer[20] = {0}; // <--- Inicializa todo a 0
            float yBuffer[20] = {0}; // <--- Inicializa todo a 0
            int index = 0;           // <--- Importante: Empieza en 0
        };

        struct LPFilter { // 2 Hz Butter
            const float b[3] = {1.0f, 2.0f, 1.0f}; 
            const float a[2] = {-1.98222893f,  0.982385451f}; 
            const float g = 0.0000391302054f;
            float s[2] = {0}; // <--- Inicializa a 0
        };

        struct ULPFilter {
            const float b1[3] = {1.0f, 2.0f, 1.0f};
            const float a1[2] = {-1.9990371f,  0.9990387f};
            const float b2[3] = {1.0f, 2.0f, 1.0f};
            const float a2[2] = {-1.9976792f,  0.99768073f};
            const float g[2] = {0.0000004f, 1.0f};
            float s1[2] = {0}; // <--- Inicializa a 0
            float s2[2] = {0}; // <--- Inicializa a 0
        };

        struct NormVars {
            float curMin = 0.0;
            float curMax = 1.0;
            float goalMin = 0.0;
            float goalMax = 1.0;
            float tempMin = 0.0;
            float tempMax = 1.0;
            float invRange = 0.01;
            float threshLow = 0.25;
            float threshHigh = 0.75;
            uint16_t counterMax = 0;
            uint16_t counterMin = 0;
            uint16_t counter;
            const uint16_t updateInterval = 3e3; // in millis
            const uint16_t changeDuration = 1e3; // in millis
            const float invChangeDuration = 1.0/changeDuration;
            float stepMax;
            float stepMin;
            unsigned long lastTime;
        };

        struct PulseVars {
            int hysteresis, maxDuration, freezeDuration;
            int freezeTimer, windowCounter;
            int offsetCrossingCount, onsetCrossingCount; 
            bool tooLong;
        };

        float _sample = 0, _envelope = 0, _smooth = 0; 
        float _threshold = 0, _secondThresholdScale = 0.5;
        bool _active = false; // true when chanel active, false otherwise

        uint8_t _id;
        uint16_t* _rawSample;
        PLFilter _plFilter{};
        LPFilter _lpFilter{};
        ULPFilter _ulpFilter{};
        NormVars _normVars{};
        PulseVars _pulseVars{};
        bool _squareEnvelope = false;

        float _scale = 1.0f;
};

