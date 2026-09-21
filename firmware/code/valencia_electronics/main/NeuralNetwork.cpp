#include "NeuralNetwork.h"

#include "nn_model.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"

// ---------------------------------------------------------------------------
// NN::NN
// ---------------------------------------------------------------------------
NN::NN(const unsigned char* modeldata)
    : _model(tflite::GetModel(modeldata))
{
    if (!_model || _model->version() != TFLITE_SCHEMA_VERSION) {
        printf("[NN] ERROR: version del modelo incompatible (%lu vs %d)\n",
               _model ? _model->version() : 0,
               TFLITE_SCHEMA_VERSION);
        return;
    }

    // Registrar solo los operadores que usa el modelo (Dense → FullyConnected,
    // ReLU, Softmax, Reshape, Shape, Pack, StridedSlice).
    static tflite::MicroMutableOpResolver<7> resolver;
    resolver.AddFullyConnected();
    resolver.AddRelu();
    resolver.AddSoftmax();
    resolver.AddReshape();
    resolver.AddShape();
    resolver.AddPack();
    resolver.AddStridedSlice();

    static tflite::MicroInterpreter static_interpreter(
        _model, resolver, _tensorArena, _kTensorArenaSize);
    _interpreter = &static_interpreter;

    if (_interpreter->AllocateTensors() != kTfLiteOk) {
        printf("[NN] ERROR: AllocateTensors() fallo.\n");
        printf("[NN] Arena size configurada: %u bytes\n", (unsigned)_kTensorArenaSize);
        return;
    }

    _input  = _interpreter->input(0);
    _output = _interpreter->output(0);

    // --- Verificaciones de tipo en runtime ---
    // Estas comprobaciones detectan inmediatamente si el modelo exportado
    // es int8 en lugar de float32, evitando el fallo silencioso.
    if (_input->type != kTfLiteFloat32) {
        printf("[NN] ERROR CRITICO: _input->type = %d, esperado %d (float32).\n",
               _input->type, kTfLiteFloat32);
        printf("[NN] El modelo parece estar cuantizado. Regenera nn_model con\n");
        printf("[NN] entrenar_modelo.py (version corregida, sin cuantizacion).\n");
        _interpreter = nullptr;
        return;
    }
    if (_output->type != kTfLiteFloat32) {
        printf("[NN] ERROR CRITICO: _output->type = %d, esperado %d (float32).\n",
               _output->type, kTfLiteFloat32);
        _interpreter = nullptr;
        return;
    }
    if (_input->dims->size != 2 || _input->dims->data[1] != nn_num_features) {
        printf("[NN] ERROR: shape de entrada [%d, %d], esperado [1, %d].\n",
               _input->dims->data[0], _input->dims->data[1], nn_num_features);
        _interpreter = nullptr;
        return;
    }

    printf("[NN] Inicializado correctamente.\n");
    printf("[NN]   Arena usada:  %u / %u bytes\n",
           (unsigned)_interpreter->arena_used_bytes(),
           (unsigned)_kTensorArenaSize);
    printf("[NN]   Input shape:  [1, %d] float32\n", nn_num_features);
    printf("[NN]   Output shape: [1, %d] float32\n", nn_output_size[0]);
}

// ---------------------------------------------------------------------------
// NN::run
// Recibe inputData[nn_history_length][nn_num_features] sin normalizar.
// Aplica el scaler y ejecuta la inferencia.
// Devuelve la clase predicha (argmax), o -1 si el intérprete no está listo.
// ---------------------------------------------------------------------------
int NN::run(float** inputData) {
    if (_interpreter == nullptr) {
        printf("[NN] run() llamado con interprete no inicializado.\n");
        return -1;
    }

    // --- Normalización (StandardScaler) ---
    // val_norm = (val_crudo - mean[j]) / std[j]
    // Exactamente igual que scaler.transform() en Python.
    int index = 0;
    for (int i = 0; i < nn_history_length; i++) {
        for (int j = 0; j < nn_num_features; j++) {
            _input->data.f[index++] =
                (inputData[i][j] - nn_scaler_mean[j]) / nn_scaler_std[j];
        }
    }
    // nn_one_hot_size == 0, verificado por static_assert en el header.
    // No hay nada más que escribir en el buffer de entrada.

    // --- Inferencia ---
    if (_interpreter->Invoke() != kTfLiteOk) {
        printf("[NN] ERROR: Invoke() fallo.\n");
        return -1;
    }

    // --- Copia de salida ---
    // _output->data.f[] es válido porque verificamos kTfLiteFloat32 en el constructor.
    for (int i = 0; i < nn_output_size[0]; i++) {
        _outputVector[i] = _output->data.f[i];
    }

    // Argmax
    int pred = 0;
    float best = _outputVector[0];
    for (int i = 1; i < nn_output_size[0]; i++) {
        if (_outputVector[i] > best) {
            best = _outputVector[i];
            pred = i;
        }
    }
    return pred;
}

// ---------------------------------------------------------------------------
// NnInput::update
//
// Lógica de features — correspondencia exacta con procesar_features.py:
//
//   Python                              C++ (este método)
//   f1 = ch0 (instantáneo)             instant[0] = emgs[0]->getEnvelope()
//   f2 = ch1 (instantáneo)             instant[1] = emgs[1]->getEnvelope()
//   f3 = rolling(128).mean() ch0       meanWin[0]  = acum_ch0 / window_size
//   f4 = rolling(128).mean() ch1       meanWin[1]  = acum_ch1 / window_size
//   f5 = |media_ch0 - media_ch1|       diffMean[0] = |meanWin[0] - meanWin[1]|
//
// El scaler se aplica en NN::run(), NO aquí, igual que en Python donde
// scaler.transform() se llama sobre el dataset completo antes de entrenar.
// ---------------------------------------------------------------------------
void NnInput::update(EMGController** emgs) {
    if (!initialized) {
        for (int i = 0; i < nnHistoryBufferSize; i++) {
            clock[i] = -i * nn_step_size;
            for (int j = 0; j < nn_num_features; j++)
                featureHistoryBuffer[i][j] = 0.0f;
        }
        nnFilledInputBufferIndex = -1;
        initialized = true;
        return;
    }

    for (int i = 0; i < nnHistoryBufferSize; i++) {
        clock[i]++;

        if (clock[i] == 0) {
            // Inicio de una nueva ventana: reiniciar acumuladores
            for (int j = 0; j < nn_num_features; j++)
                featureHistoryBuffer[i][j] = 0.0f;

        } else if (clock[i] > 0 && clock[i] <= nn_window_size) {
            // Dentro de la ventana: acumular en F2 y F3
            updatePointers(i);

            // F0, F1: se actualizan con el valor más reciente en cada paso.
            // Al final de la ventana tendrán el valor instantáneo final.
            instant[0] = emgs[0]->getEnvelope();
            instant[1] = emgs[1]->getEnvelope();

            // F2, F3: acumuladores. Se dividen al cerrar la ventana.
            meanWin[0] += emgs[0]->getEnvelope();
            meanWin[1] += emgs[1]->getEnvelope();
        }

        if (clock[i] >= nn_window_size) {
            // Ventana completa: cerrar y calcular features finales
            updatePointers(i);

            // Dividir acumuladores para obtener la media real
            meanWin[0] /= static_cast<float>(nn_window_size);
            meanWin[1] /= static_cast<float>(nn_window_size);

            // F4: diferencia absoluta entre las medias de ventana
            diffMean[0] = fabsf(meanWin[0] - meanWin[1]);

            // Señalar que este slot está listo para inferencia
            nnFilledInputBufferIndex = i;

            // Reiniciar clock para la siguiente ventana (con overlap = step_size)
            clock[i] = -nnOffWindowSize;
        }
    }

    // Guardar la última muestra (no se usa en esta versión sin absSlopeMean,
    // pero se mantiene por si se añaden features de pendiente en el futuro)
    for (int i = 0; i < nnNumChannels; i++)
        lastSample[i] = emgs[i]->getEnvelope();
}

void NnInput::updatePointers(int index) {
    // Layout del featureHistoryBuffer[index]:
    //   [0]  instant[0]  = F0 (instantáneo ch0)
    //   [1]  instant[1]  = F1 (instantáneo ch1)
    //   [2]  meanWin[0]  = F2 (media ventana ch0)
    //   [3]  meanWin[1]  = F3 (media ventana ch1)
    //   [4]  diffMean[0] = F4 (|F2 - F3|)
    instant  = &featureHistoryBuffer[index][0];
    meanWin  = &featureHistoryBuffer[index][2];
    diffMean = &featureHistoryBuffer[index][4];
}

float** NnInput::getFeatures(int index) {
    static float* x[nn_history_length];
    for (int i = 0; i < nn_history_length; i++) {
        int slot = (index - nn_history_length + 1 + i + nnHistoryBufferSize)
                   % nnHistoryBufferSize;
        x[i] = featureHistoryBuffer[slot];
    }
    return x;
}
