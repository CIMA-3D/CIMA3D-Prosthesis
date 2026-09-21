#include "NeuralNetwork.h"

#include "nn_model.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"

// In order to use optimized tensorflow lite kernels, a signed int8_t quantized
// model is preferred over the legacy unsigned model format.

// pio run -t menuconfig

#if ESP_NN
int espNN = 1;
#else
int espNN = 0;
#endif

NN::NN(const unsigned char* _modeldata)
    : _model(tflite::GetModel(_modeldata)) {

    if (!_model || _model->version() != TFLITE_SCHEMA_VERSION) {
        printf("Model provided is schema version %lu not equal to supported version %d.", _model->version(), TFLITE_SCHEMA_VERSION);
        return;
    }

    // _tensorArena = (uint8_t*) heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    // if (!_tensorArena) {
    //     printf("Failed to allocate tensor arena\n");
    //     return;
    // }

    static tflite::MicroMutableOpResolver<7> resolver;
    resolver.AddFullyConnected();
    resolver.AddRelu();
    resolver.AddSoftmax();
    resolver.AddReshape();
    resolver.AddShape();
    resolver.AddPack();
    resolver.AddStridedSlice();

    static tflite::MicroInterpreter static_interpreter(_model, resolver, _tensorArena, _kTensorArenaSize);
    _interpreter = &static_interpreter;

    if (_interpreter->AllocateTensors() != kTfLiteOk) {
        printf("AllocateTensors() failed\n");
        return;
    }

    _input = _interpreter->input(0);
    _output = _interpreter->output(0);

    for (int i = 0; i < nn_num_features; i++) {
        _inputScaler.multiplier[i] = 1.0 / (nn_scaler_std[i] * _input->params.scale);
        _inputScaler.offset[i] = _input->params.zero_point - nn_scaler_mean[i] / (_input->params.scale * nn_scaler_std[i]);
    }
    for (int i = 0; i < nn_one_hot_size; i++) {
        _inputScaler.multiplier[nn_num_features+i] = 1.0 / (_input->params.scale);
        _inputScaler.offset[nn_num_features+i] = _input->params.zero_point;
    }

    printf("NN initialized. With esp_nn: %d\n", espNN);
}



int NN::run(float **inputData) {
    int index = 0;
    for (int i = 0; i < nn_history_length; i++) {
        for (int j = 0; j < nn_num_features + nn_one_hot_size; j++) {
            float val = inputData[i][j];
            
            // Replicamos el StandardScaler de Python SOLO en las primeras 5 features
            if (j < nn_num_features) {
                val = (val - nn_scaler_mean[j]) / nn_scaler_std[j];
            }
            
            // Le pasamos el float directo a la red (nada de int8)
            _input->data.f[index++] = val;
        }
    }

    // Run inference
    if (_interpreter->Invoke() != kTfLiteOk) {
        printf("Inference failed\n");
        return -1;
    }

    // Copiamos la salida (directamente como float)
    for (int i = 0; i < nn_output_size[0]; i++) {
        _outputVector[i] = _output->data.f[i];
    }

    // Compute argmax to get the predicted label
    int pred = std::distance(_outputVector, std::max_element(_outputVector, _outputVector + nn_output_size[0]));
    return pred;
}







void NnInput::update(EMGController** emgs) {
    if (!initialized) {
        for (int i = 0; i < nnHistoryBufferSize; i++) {
            clock[i] = -i * nn_step_size;
            for (int j = 0; j < nn_num_features+nn_one_hot_size; j++)
                featureHistoryBuffer[i][j] = 0;
        }
        initialized = true;
        return;
    }

    for (int i = 0; i < nnHistoryBufferSize; i++) {
        clock[i]++;

        if (clock[i] == 0)
            for (int j = 0; j < nn_num_features+nn_one_hot_size; j++)
                featureHistoryBuffer[i][j] = 0;

        else if (clock[i] > 0) {
            updatePointers(i);
            for (int j = 0; j < nnNumChannels; j++) {
                mean[j]         += emgs[j]->getEnvelope();
                absSlopeMean[j] += abs(emgs[j]->getEnvelope() - lastSample[j]);
            }
        }

        if (clock[i] >= nn_window_size) {
            clock[i] = -nnOffWindowSize;
            
            // 1. Primero calculamos la media real de los 128 datos (como pedía Python)
            float media_ch0 = mean[0] / nn_window_size;
            float media_ch1 = mean[1] / nn_window_size;

            // 2. Ahora reordenamos los datos para que hablen "el idioma de Python"
            
            // F0 y F1: Python quiere el valor instantáneo actual
            mean[0] = emgs[0]->getEnvelope();
            mean[1] = emgs[1]->getEnvelope();

            // F2 y F3: Python quiere la media de los canales
            absSlopeMean[0] = media_ch0;
            absSlopeMean[1] = media_ch1;

            // F4: Python quiere la diferencia absoluta (resta) entre ambos canales
            globalMean[0] = abs(mean[0] - mean[1]);

            nnFilledInputBufferIndex = i;
        }
    }

    for (int i = 0; i < nnNumChannels; i++)
        lastSample[i] = emgs[i]->getEnvelope();
}

void NnInput::updatePointers(int &index) {
    mean            = &featureHistoryBuffer[index][0];
    absSlopeMean    = &featureHistoryBuffer[index][nnNumChannels];
    globalMean      = &featureHistoryBuffer[index][nnNumChannels*2];
    lastPredictions = globalMean + 1;
}

float** NnInput::getFeatures(int &index) {
    static float* x[nn_history_length];
    for (int i = 0; i < nn_history_length; i++)
        x[i] = featureHistoryBuffer[(index - nn_history_length + i + nnHistoryBufferSize) % nnHistoryBufferSize];
    return x;
}

void NnInput::updateLastPredictions(int &index, float* probs) {
    for (int i = 0; i < nn_one_hot_size; i++)
        featureHistoryBuffer[(index + 1) % nnHistoryBufferSize][nnNumChannels*2 + 1] = probs[i];
}



void NnOutputPostProcess::init(float alpha, float thetaOn, float thetaOff, int nOn, int mOff) {
    this->alpha = alpha;
    this->thetaOn = thetaOn;
    this->thetaOff = thetaOff;
    this->nOn = nOn;
    this->mOff = mOff;

    for (int c = 0; c < nn_output_size[0]; c++) {
        pSmooth[c] = 0.0f;
        classCounterOn[c] = 0;
        classCounterOff[c] = 0;
    }
    currentClass = 0;
    initialized = false;
}


// Process one new sample of probabilities
int NnOutputPostProcess::process(const float *probs) {
    // Initialize EMA with first sample
    if (!initialized) {
        for (int c = 0; c < nn_output_size[0]; c++) {
            pSmooth[c] = probs[c];
        }
        initialized = true;
    } else {
        // Update EMA
        for (int c = 0; c < nn_output_size[0]; c++) {
            pSmooth[c] = alpha * probs[c] + (1.0f - alpha) * pSmooth[c];
        }
    }

    // Update counters
    for (int c = 0; c < nn_output_size[0]; c++) {
        if (c == currentClass) {
            if (pSmooth[c] < thetaOff) {
                classCounterOff[c]++;
            } else {
                classCounterOff[c] = 0;
            }
        } else {
            if (pSmooth[c] > thetaOn) {
                classCounterOn[c]++;
            } else {
                classCounterOn[c] = 0;
            }
        }
    }

    // Check switch condition
    int newClass = currentClass;
    float bestProb = -1.0f;
    for (int c = 0; c < nn_output_size[0]; c++) {
        if (classCounterOn[c] >= nOn) {
            if (pSmooth[c] > bestProb) {
                bestProb = pSmooth[c];
                newClass = c;
            }
        }
    }
    if (newClass != currentClass) {
        currentClass = newClass;
        for (int c = 0; c < nn_output_size[0]; c++) {
            classCounterOn[c] = 0;
            classCounterOff[c] = 0;
        }
    }

    // Check off condition
    if (classCounterOff[currentClass] >= mOff) {
        currentClass = 0; // back to rest
        for (int c = 0; c < nn_output_size[0]; c++) {
            classCounterOff[c] = 0;
        }
    }

    return currentClass; // output class id
}