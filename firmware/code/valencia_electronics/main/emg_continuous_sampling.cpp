#include "emg_continuous_sampling.h"
#include "esp_adc/adc_continuous.h"
#include "globals.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring> // Para memset

// --- CONFIGURACIÓN ESPECÍFICA ESP32-S3 ---
#define ADC_UNIT            ADC_UNIT_1
#define ADC_CONV_MODE       ADC_CONV_SINGLE_UNIT_1
#define ADC_BIT_WIDTH       ADC_BITWIDTH_12
#define BUFFER_SIZE         ((int)(numEmgChannels * SOC_ADC_DIGI_RESULT_BYTES * 4)) 
#define FRAME_SIZE          ((int)(numEmgChannels * SOC_ADC_DIGI_RESULT_BYTES))

static adc_continuous_handle_t adcHandle = NULL;
static uint16_t* dataBuffer;
static TaskHandle_t samplingTaskHandle = NULL;

static int8_t channelMap[10]; 
bool ovf = false;

portMUX_TYPE emgMux = portMUX_INITIALIZER_UNLOCKED;

static bool IRAM_ATTR onConversionCallback(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data) {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(samplingTaskHandle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
    return true;
}

static bool IRAM_ATTR onPoolOverflow(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data) {
    ovf = true;
    return true;
}

void initAdc(uint16_t* buffer, TaskHandle_t *callbackTaskHandle) {
    dataBuffer = buffer;
    samplingTaskHandle = *callbackTaskHandle;

    for(int i=0; i<10; i++) channelMap[i] = -1;

    adc_continuous_handle_cfg_t adcBufferConfig = {};
    adcBufferConfig.max_store_buf_size = BUFFER_SIZE * 2;
    adcBufferConfig.conv_frame_size = FRAME_SIZE; 
    
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adcBufferConfig, &adcHandle));

    adc_digi_pattern_config_t adcPattern[numEmgChannels] = {};
    
    for (int i = 0; i < numEmgChannels; i++) {
        uint8_t channel = (emgPins[i] - 1) % 10; 
        uint8_t unit = 0; 

        adcPattern[i].atten = ADC_ATTEN_DB_12;
        adcPattern[i].channel = channel;
        adcPattern[i].unit = unit;    
        adcPattern[i].bit_width = ADC_BIT_WIDTH;

        if(channel < 10) {
            channelMap[channel] = i;
        }
    }

    adc_continuous_config_t adcConfig = {
        .pattern_num = numEmgChannels,
        .adc_pattern = adcPattern,
        .sample_freq_hz = 20 * 1000, 
        .conv_mode = ADC_CONV_MODE,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2, 
    };

    ESP_ERROR_CHECK(adc_continuous_config(adcHandle, &adcConfig));

    adc_continuous_evt_cbs_t callbacks = {};
    callbacks.on_conv_done = onConversionCallback;
    callbacks.on_pool_ovf = onPoolOverflow;

    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(adcHandle, &callbacks, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(adcHandle));
}

void readAdc() {
    static uint32_t out_len;
    static uint8_t result[FRAME_SIZE]; // Buffer temporal de lectura
    
    // Leemos el hardware FUERA del cerrojo para no bloquear el chip
    esp_err_t ret = adc_continuous_read(adcHandle, result, FRAME_SIZE, &out_len, 0);

    if (ret == ESP_OK) {
        // BLOQUEAMOS LA MEMORIA
        portENTER_CRITICAL(&emgMux);
        
        for (int i = 0; i < out_len; i += SOC_ADC_DIGI_RESULT_BYTES) {
            adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
            
            uint32_t hwChannel = p->type2.channel;
            uint32_t data = p->type2.data;

            if (hwChannel < 10) {
                int8_t bufferIndex = channelMap[hwChannel];
                if (bufferIndex >= 0 && bufferIndex < numEmgChannels) {
                    dataBuffer[bufferIndex] = data; // Escribimos rápido y seguro
                }
            }
        }
        
        // DESBLOQUEAMOS LA MEMORIA
        portEXIT_CRITICAL(&emgMux);
    }
}