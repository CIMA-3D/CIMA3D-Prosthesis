#pragma once

#include "esp_adc/adc_oneshot.h"

class AdcSensor {
public:
    AdcSensor(uint8_t pin);
    
    int read();

private:
    adc_unit_t _unit;
    adc_channel_t _channel;
    adc_oneshot_unit_handle_t _adc_handle;
    adc_cali_handle_t _cali_handle;

    static const adc_atten_t _attenuation = ADC_ATTEN_DB_12;
    static const adc_bitwidth_t _bitwidth = ADC_BITWIDTH_12;

    static bool _adcsInitialized[2];
    static void initUnitOnce(adc_unit_t unit);
    static adc_oneshot_unit_handle_t _adcHandles[2];
};

