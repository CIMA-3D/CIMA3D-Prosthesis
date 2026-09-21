#pragma once

#include "nn_model.h"
#include "EMGController.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/micro/micro_interpreter.h"


// In order to use optimized tensorflow lite kernels, a signed int8_t quantized
// model is preferred over the legacy unsigned model format.

// pio run -t menuconfig

class NN {
public:
    NN(const unsigned char* _modeldata);
    int run(float **inputData);
    
    float* getOutput() { return _outputVector; }


private:
    const tflite::Model* _model;
    tflite::MicroInterpreter* _interpreter;
    TfLiteTensor* _input;
    TfLiteTensor* _output;

    static constexpr uint32_t _kTensorArenaSize = 25 * 1024;  // 25 KB arena fija
    
    alignas(8) uint8_t _tensorArena[_kTensorArenaSize]; // or better heap_caps_malloc -> SRAM (faster, default) / PSRAM (slower, bigger) ?
    // interpreter->arena_used_bytes()

    struct InputScaler {
        float multiplier[nn_num_features+nn_one_hot_size];
        float offset[nn_num_features+nn_one_hot_size];
    };
    InputScaler _inputScaler;

    float _outputVector[nn_output_size[0]] = {};
};



constexpr int nnMinRequiredNumBuffers = ceil(nn_window_size * 1.0f / nn_step_size);
constexpr int nnNumChannels = 2;
constexpr int nnHistoryBufferSize = nnMinRequiredNumBuffers + 2;
constexpr int nnOffWindowSize = nnHistoryBufferSize * nn_step_size - nn_window_size;

struct NnInput {
    float featureHistoryBuffer[nnHistoryBufferSize][nn_num_features+nn_one_hot_size];
    float* mean;
    float* absSlopeMean;
    float* globalMean;
    float* lastPredictions;

    float lastSample[nnNumChannels];
    int clock[nnHistoryBufferSize];

    bool initialized;
    int nnFilledInputBufferIndex = -1;

    void update(EMGController** emgs);
    void updatePointers(int &index);
    float** getFeatures(int &index);
    void updateLastPredictions(int &index, float* probs);
};


struct NnOutputPostProcess {
    float alpha;
    float thetaOn;
    float thetaOff;
    int nOn;
    int mOff;

    float pSmooth[nn_output_size[0]];     // EMA state
    int classCounterOn[nn_output_size[0]];
    int classCounterOff[nn_output_size[0]];

    int currentClass;

    bool initialized;


    void init(float alpha = 0.7f, float thetaOn = 0.8f, float thetaOff = 0.3f, int nOn = 2, int mOff = 2);
    int process(const float *probs);
};