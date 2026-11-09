#include "EMGController.h"

#include <math.h>

EMGController::EMGController(uint8_t id, uint16_t* inputSamplePointer)
    : _id(id), _rawSample(inputSamplePointer) {
    
    _plFilter.index = 0;
    for(int i=0; i<20; i++) {
        _plFilter.xBuffer[i] = 0;
        _plFilter.yBuffer[i] = 0;
    }
}

void EMGController::applyPowerLineFilter() {
    float y; 
    
    if (_plFilter.index < 0 || _plFilter.index >= 20) _plFilter.index = 0;

    y = *_rawSample - _plFilter.xBuffer[_plFilter.index] + 0.5f * _plFilter.yBuffer[_plFilter.index];
    
    _plFilter.xBuffer[_plFilter.index] = *_rawSample;
    _plFilter.yBuffer[_plFilter.index] = y;

    _plFilter.index++;
    if (_plFilter.index >= 20) _plFilter.index = 0; 

    _sample = y;
}

void EMGController::applyLPFilter() {
    float absSample;
    absSample = abs(_sample);
    _envelope = _lpFilter.b[0] * absSample + _lpFilter.s[0]; // Calculate output
    _lpFilter.s[0] = (absSample * _lpFilter.b[1]) - (_envelope * _lpFilter.a[0]) + _lpFilter.s[1]; // Update first state
    _lpFilter.s[1] = (absSample * _lpFilter.b[2]) - (_envelope * _lpFilter.a[1]); // Update second state
    // _envelope *= _lpFilter.g;
}

void EMGController::normalize() {
    if (fabs(_normVars.curMax - _normVars.curMin) < 1e-6f) return;

    static bool updated;

    updated = false;

    if (_envelope > _normVars.goalMax*2.0) {// initial overshoot
        _normVars.curMax = _envelope;
        _normVars.goalMax = _envelope;
        _normVars.invRange = 1.0/(_normVars.curMax-_normVars.curMin) * 100;
        updated = true;
    }

    _normVars.counter++;
    if (_normVars.counter == 1) {
        _normVars.tempMax = _envelope;
        _normVars.tempMin = _envelope;
    }
    else if (_normVars.counter < _normVars.updateInterval) {
        if (_envelope > _normVars.tempMax) _normVars.tempMax = _envelope;
        else if (_envelope < _normVars.tempMin) _normVars.tempMin = _envelope;
    }
    else {
        if (_normVars.tempMin < 0) // ignore negative values
            _normVars.tempMin = 0;

        if (_normVars.tempMax > _normVars.goalMax) {
            _normVars.goalMax = _normVars.tempMax;
            _normVars.stepMax = (_normVars.goalMax-_normVars.curMax) * _normVars.invChangeDuration;
            _normVars.counterMax = _normVars.changeDuration;
            updated = true;
        }
        else if (_normVars.tempMax > _normVars.goalMin + 0.75 * (_normVars.goalMax - _normVars.goalMin)) {
            _normVars.goalMax = (_normVars.tempMax + _normVars.goalMax) * 0.5;
            _normVars.stepMax = (_normVars.goalMax-_normVars.curMax) * _normVars.invChangeDuration;
            _normVars.counterMax = _normVars.changeDuration;
            updated = true;
        }

        if (_normVars.tempMin < _normVars.goalMin) {
            _normVars.goalMin = _normVars.tempMin;
            _normVars.stepMin = (_normVars.goalMin - _normVars.curMin) * _normVars.invChangeDuration;
            _normVars.counterMin = _normVars.changeDuration;
            updated = true;
        }
        else if (_normVars.tempMin < _normVars.goalMin + 0.25 * (_normVars.goalMax - _normVars.goalMin)) {
            _normVars.goalMin = (_normVars.tempMin + _normVars.goalMin) * 0.5;
            _normVars.stepMin = (_normVars.goalMin - _normVars.curMin) * _normVars.invChangeDuration;
            _normVars.counterMin = _normVars.changeDuration;
            updated = true;
        }

        _normVars.counter = 0;
    }

    if (updated) {
        _normVars.threshLow = _normVars.goalMin + (_normVars.goalMax - _normVars.goalMin) * 0.25;
        _normVars.threshHigh = _normVars.goalMin + (_normVars.goalMax - _normVars.goalMin) * 0.75;
    }

    if (_normVars.counterMax > 0) {
        _normVars.counterMax--;
        _normVars.curMax += _normVars.stepMax;
        _normVars.invRange = 1.0/(_normVars.curMax-_normVars.curMin) * 100;
    }
    if (_normVars.counterMin > 0) {
        _normVars.counterMin--;
        _normVars.curMin += _normVars.stepMin;
        _normVars.invRange = 1.0/(_normVars.curMax-_normVars.curMin) * 100;
    }
    _envelope = (_envelope-_normVars.curMin)*_normVars.invRange;
    if (_envelope < 0) _envelope = 0;
    if (_squareEnvelope) _envelope = pow(_envelope, 2) * 0.01;
}

void EMGController::update() {
    
    if (_pulseVars.freezeTimer > 0) {
        _pulseVars.freezeTimer--;
        return;
    }

    if (!_active && _envelope >= _threshold) {
        _active = true;
        _pulseVars.onsetCrossingCount++;
    } else if (_active && _envelope < _threshold*_secondThresholdScale) {
        _pulseVars.offsetCrossingCount++;
        _pulseVars.onsetCrossingCount++;
    } else if (_active && _envelope >= _threshold*_secondThresholdScale) {
        _pulseVars.offsetCrossingCount = 0;
        _pulseVars.onsetCrossingCount++;
    }

    if (_pulseVars.offsetCrossingCount > _pulseVars.hysteresis) {
        if (_pulseVars.tooLong) // Too long
            resetVars();
        else {                  // Pulse detected
            resetVars();
            pulse = true;
        }
    } else if (_pulseVars.onsetCrossingCount >= _pulseVars.maxDuration + _pulseVars.hysteresis) {
        _pulseVars.tooLong = true;
    }
}

void EMGController::resetVars() {
    _pulseVars.offsetCrossingCount = 0;
    _pulseVars.onsetCrossingCount = 0;
    _active = false;
    pulse = false;
    _pulseVars.freezeTimer = _pulseVars.freezeDuration;
    _pulseVars.tooLong = false;
}




void EMGController::applyULPFilter() {
    float y1, y2;
    float absSample;

    absSample = abs(_sample);
    // Stage 1
    y1 = _ulpFilter.b1[0] * absSample + _ulpFilter.s1[0]; // Calculate output
    y1 = y1 * _ulpFilter.g[0];
    _ulpFilter.s1[0] = (absSample * _ulpFilter.b1[1]) - (y1 * _ulpFilter.a1[0]) + _ulpFilter.s1[1]; // Update first state
    _ulpFilter.s1[1] = (absSample * _ulpFilter.b1[2]) - (y1 * _ulpFilter.a1[1]); // Update second state

    // Stage 2
    y2 = _ulpFilter.b2[0] * y1 + _ulpFilter.s2[0]; // Calculate output
    // y2 = y2 * _ulpFilter.g[1];
    _ulpFilter.s2[0] = (y1 * _ulpFilter.b2[1]) - (y2 * _ulpFilter.a2[0]) + _ulpFilter.s2[1]; // Update first state
    _ulpFilter.s2[1] = (y1 * _ulpFilter.b2[2]) - (y2 * _ulpFilter.a2[1]); // Update second state

    _smooth = y2;
}

void EMGController::updateSmooth() {
    
    applyULPFilter();
    _smooth *= _scale;
    if (_smooth > 110) {
        _scale *= 100.0/_smooth;
        _smooth = 100.0;
    }
}

