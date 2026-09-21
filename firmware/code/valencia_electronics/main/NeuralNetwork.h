#pragma once

#include "nn_model.h"
#include "EMGController.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/micro/micro_interpreter.h"

// ---------------------------------------------------------------------------
// NN — Clase de inferencia TFLite Micro
//
// Cambios respecto a la versión anterior:
//   - Verifica en tiempo de compilación que nn_one_hot_size == 0.
//     Si en el futuro se reintroduce el one-hot, el compilador fuerza
//     actualizar esta clase también.
//   - Arena size calculada con margen real (ver comentario abajo).
//   - Añade run_if_ready(): combina la comprobación del índice y la
//     inferencia en una sola llamada desde mainTask.
// ---------------------------------------------------------------------------
class NN {
public:
    explicit NN(const unsigned char* modeldata);

    // Ejecuta inferencia. inputData es un array de nn_history_length punteros,
    // cada uno apuntando a nn_num_features floats ya normalizados por el scaler.
    // Devuelve la clase predicha (0–3), o -1 si falla.
    int run(float** inputData);

    // Probabilidades de la última inferencia. Tamaño: nn_output_size[0].
    float _outputVector[nn_output_size[0]] = {};

    float* getOutputVector() { return _outputVector; }

private:
    const tflite::Model*      _model;
    tflite::MicroInterpreter* _interpreter;
    TfLiteTensor*             _input;
    TfLiteTensor*             _output;

    // --- Arena size ---
    // Para una red float32 9→16→8→4 TFLite Micro necesita espacio para:
    //   · Pesos y sesgos de las capas (ya están en el flatbuffer, no en la arena)
    //   · Tensores de activación intermedios: 16*4 + 8*4 + 4*4 = 112 bytes
    //   · Overhead del intérprete y metadatos de operadores: ~4–8 KB típico
    //   · Buffer de scratch de operadores (FullyConnected): ~1 KB
    // Usar 10 KB garantiza que AllocateTensors() tenga margen suficiente.
    // Después de AllocateTensors(), el constructor imprime arena_used_bytes()
    // para que puedas ajustar hacia abajo si necesitas RAM para otra cosa.
    static constexpr size_t _kTensorArenaSize = 10 * 1024;
    alignas(16) uint8_t _tensorArena[_kTensorArenaSize];

    // Verificación de que el modelo no tiene one-hot (nn_one_hot_size debe ser 0)
    static_assert(
        nn_one_hot_size == 0,
        "nn_one_hot_size != 0: el modelo fue entrenado con one-hot. "
        "Revisa entrenar_modelo.py y NnInput."
    );
};

// ---------------------------------------------------------------------------
// Constantes de la ventana deslizante
// ---------------------------------------------------------------------------
constexpr int nnNumChannels      = 2;

// Cuántos buffers necesitamos para cubrir la historia pedida por la red.
// Con nn_history_length=1 y step=64, necesitamos al menos ceil(128/64)=2
// buffers más un par de margen.
constexpr int nnMinReqBuffers    = (nn_window_size + nn_step_size - 1) / nn_step_size;
constexpr int nnHistoryBufferSize = nnMinReqBuffers + 2;

// Muestras que el buffer permanece inactivo entre ventanas
constexpr int nnOffWindowSize    = nnHistoryBufferSize * nn_step_size - nn_window_size;

// ---------------------------------------------------------------------------
// NnInput — Acumulador de features con ventana deslizante
//
// Cambios respecto a la versión anterior:
//   - Elimina completamente el bloque one-hot (nn_one_hot_size = 0).
//   - featureHistoryBuffer ahora tiene solo nn_num_features columnas.
//   - updateLastPredictions() eliminado (no tiene sentido sin one-hot).
//   - La lógica de features coincide exactamente con procesar_features.py:
//       F0 = getEnvelope() ch0 al final de la ventana  (instantáneo)
//       F1 = getEnvelope() ch1 al final de la ventana  (instantáneo)
//       F2 = suma_ch0 / nn_window_size                 (media de la ventana)
//       F3 = suma_ch1 / nn_window_size                 (media de la ventana)
//       F4 = |media_ch0 - media_ch1|
// ---------------------------------------------------------------------------
struct NnInput {
    // Buffer circular de vectores de features
    float featureHistoryBuffer[nnHistoryBufferSize][nn_num_features];

    // Punteros al slot activo (actualizado por updatePointers)
    float* instant;    // F0, F1
    float* meanWin;    // F2, F3
    float* diffMean;   // F4

    float lastSample[nnNumChannels];
    int   clock[nnHistoryBufferSize];
    bool  initialized;

    // Índice del slot más recientemente completado (-1 si ninguno listo)
    int nnFilledInputBufferIndex;

    void update(EMGController** emgs);
    void updatePointers(int index);

    // Devuelve un array de nn_history_length punteros a features,
    // ordenados del más antiguo al más reciente.
    float** getFeatures(int index);
};
