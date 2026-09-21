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
#define BUFFER_SIZE         ((int)(numEmgChannels * SOC_ADC_DIGI_RESULT_BYTES * 4)) // Buffer un poco más grande por seguridad
#define FRAME_SIZE          ((int)(numEmgChannels * SOC_ADC_DIGI_RESULT_BYTES))

static adc_continuous_handle_t adcHandle = NULL;
static uint16_t* dataBuffer;
static TaskHandle_t samplingTaskHandle = NULL;

// ESTA ES LA CLAVE: Un mapa simple para traducir HW Channel -> Buffer Index
// El ESP32 tiene canales del 0 al 9 en ADC1.
static int8_t channelMap[10]; 

bool ovf = false;

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

    // 1. Inicializamos el mapa a -1 (para saber qué canales NO usamos)
    for(int i=0; i<10; i++) channelMap[i] = -1;

    adc_continuous_handle_cfg_t adcBufferConfig = {};
    adcBufferConfig.max_store_buf_size = BUFFER_SIZE * 2;
    adcBufferConfig.conv_frame_size = FRAME_SIZE; 
    
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adcBufferConfig, &adcHandle));

    adc_digi_pattern_config_t adcPattern[numEmgChannels] = {};
    
    for (int i = 0; i < numEmgChannels; i++) {
        // Lógica para ESP32-S3 (Pin 4 -> Canal 3, Pin 5 -> Canal 4)
        // Usamos tu fórmula original que funciona bien en S3 para pines bajos
        uint8_t channel = (emgPins[i] - 1) % 10; 
        uint8_t unit = 0; // ADC_UNIT_1 es unit 0

        adcPattern[i].atten = ADC_ATTEN_DB_12;
        adcPattern[i].channel = channel;
        adcPattern[i].unit = unit;    
        adcPattern[i].bit_width = ADC_BIT_WIDTH;

        // --- LA SOLUCIÓN MÁGICA ---
        // Guardamos en el mapa que este canal de hardware (ej: 3) 
        // corresponde a la posición 'i' (ej: 0) de tu array.
        if(channel < 10) {
            channelMap[channel] = i;
        }
    }

    adc_continuous_config_t adcConfig = {
        .pattern_num = numEmgChannels,
        .adc_pattern = adcPattern,
        .sample_freq_hz = 20 * 1000, // 20 kHz es seguro para S3
        .conv_mode = ADC_CONV_MODE,
        .format = ADC_DIGI_OUTPUT_FORMAT_TYPE2, // El S3 SIEMPRE usa Type 2
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
    
    // Leemos los bytes crudos del ADC
    adc_continuous_read(adcHandle, result, FRAME_SIZE, &out_len, 0);

    for (int i = 0; i < out_len; i += SOC_ADC_DIGI_RESULT_BYTES) {
        // Casteamos a la estructura de datos del S3 (Type 2)
        adc_digi_output_data_t *p = (adc_digi_output_data_t*)&result[i];
        
        uint32_t hwChannel = p->type2.channel;
        uint32_t data = p->type2.data;

        // --- PROTECCIÓN CONTRA EL CRASH ---
        if (hwChannel < 10) {
            // Buscamos en el mapa: ¿A qué índice de 'dataBuffer' le toca este canal?
            int8_t bufferIndex = channelMap[hwChannel];

            // Si bufferIndex es válido (ej: 0 o 1), guardamos.
            // Si es -1, ignoramos el dato (ruido o error).
            if (bufferIndex >= 0 && bufferIndex < numEmgChannels) {
                dataBuffer[bufferIndex] = data;
            }
        }
    }
}