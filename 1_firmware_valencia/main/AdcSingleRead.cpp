#include "AdcSingleRead.h"

bool AdcSensor::_adcsInitialized[2] = { false };
adc_oneshot_unit_handle_t AdcSensor::_adcHandles[2] = { nullptr };


AdcSensor::AdcSensor(uint8_t pin) {
    if (pin < 1 || pin > 20) {
        printf("Invalid ADC pin %d\n", pin);
        return;
    }

    _unit = adc_unit_t((pin-1)/10);
    _channel = adc_channel_t((pin-1)%10);

    initUnitOnce(_unit);

    adc_oneshot_chan_cfg_t config = {
        .atten = _attenuation,
        .bitwidth = _bitwidth,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(_adcHandles[_unit], _channel, &config));


    _cali_handle = NULL;
    esp_err_t ret;

    #if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = _unit,
        .chan = _channel,
        .atten = _attenuation,
        .bitwidth = _bitwidth,
    };
    ret = adc_cali_create_scheme_curve_fitting(&cali_config, &_cali_handle);  // Curve Fitting is supported, line fitting is not supported on ESP32 S3
    
    #elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t cali_config = {
        .unit_id = _unit,
        .atten = _attenuation,
        .bitwidth = _bitwidth,
        #if CONFIG_IDF_TARGET_ESP32
        .default_vref = 0,
        #endif
    };
    ret = adc_cali_create_scheme_line_fitting(&cali_config, &_cali_handle);  // Curve Fitting is not supported, line fitting is supported on ESP32
    
    #else
    ret = ESP_ERR_NOT_SUPPORTED;
    #endif
    
    if (ret == ESP_ERR_NOT_SUPPORTED)
        printf("Pin %d: eFuse not burnt, skip software calibration\n", pin);
    else if (ret != ESP_OK)
        printf("Pin %d not couldn't create calibration scheme\n", pin);
}

void AdcSensor::initUnitOnce(adc_unit_t unit) {
    if (!_adcsInitialized[unit]) {
        adc_oneshot_unit_init_cfg_t init_config = {};
        init_config.unit_id = unit;

        ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &_adcHandles[unit])); 

        _adcsInitialized[unit] = true;
    }
}

int AdcSensor::read() {
    int raw, voltage;
    esp_err_t err;

    err = adc_oneshot_read(_adcHandles[_unit], _channel, &raw);
    if (err != ESP_OK) return -1;

    err = adc_cali_raw_to_voltage(_cali_handle, raw, &voltage);
    if (err != ESP_OK) return -1;

    return voltage;
}