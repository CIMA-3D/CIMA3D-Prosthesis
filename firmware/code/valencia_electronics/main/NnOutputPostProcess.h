#pragma once

class NnOutputPostProcess {
private:
    float thresholdDeact;
    float thresholdAct;
    float smoothing;
    int debounceWindow;
    int numClasses;
    int currentLabel;
    float* smoothedVals;

public:
    NnOutputPostProcess() : currentLabel(0), smoothedVals(nullptr) {}
    
    ~NnOutputPostProcess() {
        if (smoothedVals) delete[] smoothedVals;
    }

    // Inicializa los umbrales y el tamaño de las clases
    void init(float tDeact, float tAct, float alpha, int window, int classes) {
        thresholdDeact = tDeact;
        thresholdAct = tAct;
        smoothing = alpha;
        debounceWindow = window;
        numClasses = classes;
        currentLabel = 0;
        
        if (smoothedVals) delete[] smoothedVals;
        smoothedVals = new float[numClasses]{0};
    }

    // Procesa las probabilidades crudas de la IA y devuelve el gesto definitivo
    int process(float* nnOutputs) {
        if (!smoothedVals) return 0;
        
        int maxIdx = 0;
        float maxVal = -1.0f;

        // 1. Suavizar las salidas (Filtro de media móvil exponencial)
        for (int i = 0; i < numClasses; i++) {
            smoothedVals[i] = (smoothing * nnOutputs[i]) + ((1.0f - smoothing) * smoothedVals[i]);
            if (smoothedVals[i] > maxVal) {
                maxVal = smoothedVals[i];
                maxIdx = i;
            }
        }

        // 2. Aplicar umbrales (Histéresis para evitar tirones)
        if (maxIdx != currentLabel) {
            // Para cambiar a un gesto nuevo, debe superar el umbral alto (ej. 0.9)
            if (maxVal > thresholdAct) {
                currentLabel = maxIdx;
            }
        } else {
            // Para soltar el gesto y volver a reposo, debe caer del umbral bajo (ej. 0.3)
            if (maxVal < thresholdDeact) {
                currentLabel = 0; // 0 suele ser la etiqueta de reposo
            }
        }

        return currentLabel;
    }
};