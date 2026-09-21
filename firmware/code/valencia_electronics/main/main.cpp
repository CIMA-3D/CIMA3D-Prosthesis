#include <freertos/FreeRTOS.h>
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <math.h>
#include <algorithm>

#include "SmartLeds.h"
#include "Servo.h"
#include "PwmController.h"
#include "EMGController.h"
#include "NonBlockingTimer.h"
#include "nn_model.h"
#include "NeuralNetwork.h"
#include "NnOutputPostProcess.h"

#include "AdcSingleRead.h"

#include "emg_continuous_sampling.h"
#include "globals.h"
#include "movements.h"
#include "secondary_sensors.h"
#include "uart_command_handler.h"

// THE TICK RATE IS SET TO 100 Hz, so 1 tick = 10 ms!!!

// TODO: use fixed-point arithmetic for faster calculations.

// TODO: for decision tree consider enabling -fjump-tables and -ftree-switch-conversion.
// Useful if you have big decision trees or state machines that aren't in IRAM.

// NimBLE-based Host APIs task has high priority (21) and is pinned to Core 0. IN IDF
// Wi-Fi Driver task has high priority (23) and is pinned to Core 0. IN IDF


bool enableCustomPrintingTask = false;
bool enableNeuralNetwork = false;

bool enableThresholdControl = true;

bool samplePrimaryEmgSensors = true;
bool sampleSecondarySensors = false;

bool printT = false;
bool printRawEmg = false;
bool printFilteredEmg = false;
bool printEmgEnvelope = true;
bool squareEmgEnvelope = false;
bool printNN = false;


bool printTrigger = false;
bool triggerControl = false;


bool printBattery = true;
bool printTemperature = false;

const uint16_t calibrationTime = 500; // in millis. FIX EMG CALIBRATION (INIT) ROUTINE
const int downsamplingWindow = 200;


void updateEmgTrigger();
void resetEMG();
void emgControl();
void smoothControl();
void moveset1();
void moveset2();
void parseCmd(const char* cmd);
void buzzMelody();
TaskHandle_t printingTaskHandle = NULL;
TaskHandle_t samplingTaskHandle = NULL;
TaskHandle_t mainTaskHandle = NULL;
TaskHandle_t timeCheckingTaskHandle = NULL;
TaskHandle_t secondarySensorsSamplingTaskHandle = NULL;
TaskHandle_t customPrintingTaskHandle = NULL;
void secondarySensorsSamplingTask(void *arg);
void printingTask(void *arg);
void timeCheckingTask(void *param);
void samplingTask(void *arg);


enum class TriggerEMG { NONE, CLOSE, OPEN, CLOSE_DOUBLE, OPEN_DOUBLE };
TriggerEMG triggerEMG = TriggerEMG::NONE;

SmartLed leds(LED_WS2812B, 1, 48, 0);

NN* nn;
NnInput nnInput{};
NnOutputPostProcess nnOutPp;
volatile bool adcToNnFlag = false;
volatile int nnFilledInputBufferIndex = -1;

void customPrintingTask(void *arg) {
    NonBlockingTimer timer;
    static uint16_t printingInterval = 100; // millis
    while(1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // printf("%f", emgs[0]->getEnvelope());
        // for (int i = 1; i < numEmgChannels; i++)
        //     printf(",%f", emgs[i]->getEnvelope());
        // printf("\n");

        if (timer.check()) {
            for (int i = 0; i < numEmgChannels; i++)
                printf(">Raw%i:%d\n", i, rawEmgSampleBuffer[i]);
            timer.setMillis(printingInterval);
        }
        vTaskDelay(pdMS_TO_TICKS(1));  // 10 ms delay for CPU

    }
}


// -----------------------------------------------------------------------
// Control por umbrales tradicional (sin IA).
// Canal 0 = flexor principal (cierra/pinch)
// Canal 1 = extensor/antagonista (abre/unpinch)
// Co-contracción (ambos altos) = cambio de modo cíclico
// -----------------------------------------------------------------------
void evaluateThresholdControl(EMGController** emgs)
{
    static int mode = 0; // 0=mano, 1=pinza, 2=muñeca
    static NonBlockingTimer cooldown; 
    static int wristPos = 50;

    // Obtener señales (Mav0: canal cerrar, Mav1: canal abrir)
    float mav0 = emgs[0]->getEnvelope();
    float mav1 = emgs[1]->getEnvelope();

    // 1. Verificamos si estamos en periodo de espera (cooldown)
    if (!cooldown.check()) return;

    // 2. DEFINICIÓN DE CONDICIONES
    // Cambio de modo: Ambos canales activos y suma robusta > 16
   // Cocontracción: ambos músculos activos a la vez
    bool isCoContraction = (
        mav0 > 3.5 &&
        mav1 > 5.0 &&
        (mav0 + mav1) > 10.0
    );

    // Abrir / unpinch / derecha
    // Dominancia clara del sensor superior (mav1)
    bool upperPeak = (
        mav1 > 6.0 &&
        mav1 > mav0 * 2.0
    );

    // Cerrar / pinch / izquierda
    // Dominancia clara del sensor inferior (mav0)
    bool lowerPeak = (
        mav0 > 4.0 &&
        mav0 > mav1 * 1.6
    );

    // 3. JERARQUÍA DE EJECUCIÓN (Prioridad absoluta al cambio de modo)
    if (isCoContraction) 
    {
        mode = (mode + 1) % 3;
        // Cooldown largo (1.2s) para evitar que el "suelte" de la contracción 
        // se detecte como un abrir/cerrar accidental
        cooldown.setMillis(1200); 
        return; 
    }
    else if (upperPeak) 
    {
        // Acción de Extensión / Abrir
        switch(mode) {
            case 0: openHand(); break;
            case 1: unpinch();  break;
            case 2: 
                wristPos = std::min(wristPos + 15, 100);
                rotateWrist(wristPos);
                break;
        }
        cooldown.setMillis(450);
    }
    else if (lowerPeak) 
    {
        // Acción de Flexión / Cerrar
        switch(mode) {
            case 0: closeHand(); break;
            case 1: pinch();     break;
            case 2: 
                wristPos = std::max(wristPos - 15, 0);
                rotateWrist(wristPos);
                break;
        }
        cooldown.setMillis(450);
    }
}


// Runs on Core 1 (Main Task) - Print, logic
void mainTask(void *pvParameters) {
    NonBlockingTimer emgTriggerPrintTimer;
    NonBlockingTimer otherSensorsPrintTimer;

    while (1) {
        updateMovements(); 

        if (otherSensorsPrintTimer.check()) {
            if (printBattery)
                printf(">Bat:%.3g\n", batteryLevel);
            if (printTemperature)
                printf(">Temp:%.3g\n", temperature);
            otherSensorsPrintTimer.setMillis(50);
        }
        // checkBatteryLevel();
        // smoothControl();LEDC_TIMER_1

        // moveset1();
        pollUart(parseCmd);

            if (enableNeuralNetwork && !triggerControl && !enableThresholdControl) { 
            static int lastLabel = -1; 
            static int predictedLabel = 0;
            
            
            // --- NUEVAS VARIABLES PARA LA MUÑECA ---
            static int wristPosition = 50; 
            static int wristDirection = 1; 
            static NonBlockingTimer wristTimer; 
            // ---------------------------------------

            
            if (nnInput.nnFilledInputBufferIndex != -1) {
                int tempIndex = nnInput.nnFilledInputBufferIndex;
                
                // 1. Obtenemos las características calculadas de los músculos
                float** features = nnInput.getFeatures(tempIndex);
                
                // 2. Ejecutamos la red neuronal (eliminamos 'int rawPrediction =' para quitar el warning)
                nn->run(features);
                
                // 3. Pasamos la predicción por el filtro suavizador (PostProcess)
                predictedLabel = nnOutPp.process(nn->getOutputVector());

                // 4. Reiniciamos la bandera para esperar más datos
                nnInput.nnFilledInputBufferIndex = -1; 
                
                if (printNN) {
                    printf(">NN:%d\n", predictedLabel);
                }
            }


            if (predictedLabel != lastLabel) {

                if (lastLabel == 3) {
                    wristDirection = -wristDirection;
                }

                switch (predictedLabel) {
                    case 0: // REPOSO
                        stopFingers();
                        break;
                    case 1: // ABRIR
                        openHand();
                        break;
                    case 2: // CERRAR
                        closeHand();
                        break;
                    case 3: // GIRAR — acción continua, el timer se activa abajo
                        wristTimer.setMillis(0);
                        break;
                    case 4: // PINCH
                        pinch();
                        break;
                    case 5: // UNPINCH
                        unpinch();
                        break;
                }
                lastLabel = predictedLabel;
            }

            // Acción continua para GIRAR (fuera del if de cambio de label)
            if (predictedLabel == 3) {
                if (wristTimer.check()) {
                    wristPosition += (5 * wristDirection);
                    wristPosition = std::clamp(wristPosition, 0, 100);
                    rotateWrist(wristPosition);
                    wristTimer.setMillis(50);
                }
            }
        }

        if (enableThresholdControl && !triggerControl) {
            evaluateThresholdControl(emgs); // <--- ¡AQUÍ ESTÁ!
        }


        updateEmgTrigger();
        if (printTrigger && emgTriggerPrintTimer.check()) {
            printf(">Trigger:%d\n", static_cast<int>(triggerEMG));
            if (!triggerControl)
                triggerEMG = TriggerEMG::NONE;  
            emgTriggerPrintTimer.setMillis(500);
        }
        if (triggerControl) emgControl();

        vTaskDelay(1);  // 10 ms delay for CPU
    }
}


extern "C" void app_main(void) {
    esp_log_level_set("*", ESP_LOG_WARN); // hide all info, show only warnings/errors

    beginUart(115200);
    printf("\n\n");

    leds[0] = Rgb {0,10,0};
    leds.show();

    initMovements();
    initSecondarySensors();

    nn = new NN(nn_model);
    nnOutPp.init(0.3f, 0.9f, 0.3f, 10, 4);

    for (int i = 0; i < numEmgChannels; i++) {
        emgs[i] = new EMGController(i, &rawEmgSampleBuffer[i]);
        emgs[i]->setMaxDuration(1000);
        emgs[i]->setHysteresis(300);
        emgs[i]->setFreezeDuration(300);
        emgs[i]->setSquareEnvelope(squareEmgEnvelope);
        emgs[i]->setThreshold(30);
    }
    // emgs[0]->setThreshold(40);
    // emgs[1]->setThreshold(20);

    xTaskCreatePinnedToCore(secondarySensorsSamplingTask, "secondarySensorsSamplingTask", 4096, NULL, 10, &secondarySensorsSamplingTaskHandle, 0);

    emgSampleQueue = xQueueCreate(16, sizeof(Sample));
    

    xTaskCreatePinnedToCore(samplingTask, "samplingTask", 8192, NULL, 24, &samplingTaskHandle, 1);
    xTaskCreatePinnedToCore(timeCheckingTask, "timeCheckingTask", 8192, NULL, 23, &timeCheckingTaskHandle, 0);
    
    initAdc(rawEmgSampleBuffer, &samplingTaskHandle);

    if (samplePrimaryEmgSensors)
        while (!emgCalibrated) vTaskDelay(1);

    xTaskCreatePinnedToCore(printingTask, "printingTask", 4096, NULL, 5, &printingTaskHandle, 0);
    xTaskCreatePinnedToCore(mainTask, "mainTask", 16384, NULL, 1, &mainTaskHandle, 0);
    
    if (enableCustomPrintingTask)
        xTaskCreatePinnedToCore(customPrintingTask, "customPrintingTask", 4096, NULL, 5, &customPrintingTaskHandle, 0);
}


extern portMUX_TYPE emgMux;

void samplingTask(void *arg) {
    int counter = 0;
    // FIX P5: Temporizador en uint64_t para que no crashee a los 71 minutos
    uint64_t lastTime = esp_timer_get_time(); 
    uint64_t now;
    uint16_t calibrationCounter = 0;
    
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (samplePrimaryEmgSensors)
            readAdc();

        if (sampleSecondarySensors)
            xTaskNotifyGive(secondarySensorsSamplingTaskHandle);

        if (!samplePrimaryEmgSensors)
            continue;

        if (!emgCalibrated) {
            calibrationCounter++;
            if (calibrationCounter >= calibrationTime) 
                emgCalibrated = true;
            continue;
        }

        now = esp_timer_get_time(); 
        uint32_t T = (uint32_t)(now - lastTime); // Cast seguro
        lastTime = now;
        xTaskNotify(timeCheckingTaskHandle, T, eSetValueWithOverwrite);
        adcToNnFlag = true;

        for (int i = 0; i < numEmgChannels; i++) {
            emgs[i]->applyPowerLineFilter();
            emgs[i]->applyLPFilter();
            emgs[i]->normalize();
            emgs[i]->update();
            if (enableCustomPrintingTask)
                emgs[i]->updateSmooth();
        }
        
        // FIX P2: ¡Solo UNA llamada! La propia red ya sabe cuándo hacer downsampling
        if (enableNeuralNetwork) {
            nnInput.update(emgs);
        }
        
        if (printFilteredEmg || printRawEmg) {
            Sample s;
            if (printRawEmg) {
                // MANTENEMOS EL CERROJO QUE HICIMOS ANTES AQUÍ
                portENTER_CRITICAL(&emgMux);
                for (int i = 0; i < numEmgChannels; i++)
                    s.rawSamples[i] = rawEmgSampleBuffer[i];
                portEXIT_CRITICAL(&emgMux);
            }
            if (printFilteredEmg) {
                for (int i = 0; i < numEmgChannels; i++)
                    s.samples[i] = emgs[i]->getFilteredSample();
            }
            if (uxQueueSpacesAvailable(emgSampleQueue) == 0) {
                emgSampleQueueFull = true;
                Sample tmp;
                while (xQueueReceive(emgSampleQueue, &tmp, 0) == pdTRUE); 
            }
            xQueueSend(emgSampleQueue, &s, 0);
        }

        if (printEmgEnvelope) {
            counter++;
            // El downsamplingWindow ahora SOLO se usa para no saturar la pantalla/serial
            if (counter >= downsamplingWindow) {
                counter = 0;
                emgEnvelopeBuffer.index = (emgEnvelopeBuffer.index + 1) % EMG_ENVELOPE_BUFFER_SIZE;
                for (int i = 0; i < numEmgChannels; i++)
                    emgEnvelopeBuffer.values[i][emgEnvelopeBuffer.index] = emgs[i]->getEnvelope();               
                emgEnvelopeBuffer.flag = true;
            }
        }

        if (customPrintingTaskHandle != NULL)
            xTaskNotifyGive(customPrintingTaskHandle);
    }
}







// ----------------------------------------------------------------------------------------------------
void updateEmgTrigger() {
    // static bool calibrated = false;
    static int counterC = 0;
    static int counterA = 0;

    if (emgs[0]->pulse) {
        emgs[0]->pulse = false;
        emgs[1]->pulse = false;
        if (counterC++ >= 2) 
            triggerEMG = TriggerEMG::CLOSE;
    } else if (emgs[1]->pulse) {
        emgs[1]->pulse = false;
        if (counterA++ >= 2) 
            triggerEMG = TriggerEMG::OPEN;
    }

    // if (emgs[0]->pulse) {
    //     emgs[0]->pulse = false;
    //     emgs[1]->pulse = false;
    //     if (!calibrated) 
    //         calibrated = (++counterC >= 3) && (counterA >= 3);
    //     else
    //         triggerEMG = TriggerEMG::CLOSE;
    // } else if (emgs[1]->pulse) {
    //     emgs[1]->pulse = false;
    //     if (!calibrated)
    //         calibrated = (++counterA >= 3) && (counterC >= 3);
    //     else
    //         triggerEMG = TriggerEMG::OPEN;
    // }
}

void emgControl() {
    if (triggerEMG != TriggerEMG::NONE) {
        if (printTrigger)
            printf(">Trigger:%d\n", static_cast<int>(triggerEMG));
        if (triggerEMG == TriggerEMG::CLOSE) {
            if (handState == HandState::PINCHED)
                unpinch();
            else if (handState == HandState::OPEN)
                unpinch();
            // else if (handState == UNPINCHED) pinch();
            else if (handState == HandState::UNPINCHED) {
                controlledPinch(100);
                unsigned long tiempo = esp_timer_get_time();
                while (!emgs[0]->getActive() && !emgs[1]->getActive() && esp_timer_get_time() - tiempo < 1000*timePinching*2)
                    vTaskDelay(1);
                stopFingers();
                // delay(100);
            }

        } else if (triggerEMG == TriggerEMG::OPEN) {
            if (handState == HandState::UNPINCHED)
                openHand();
            else if (handState == HandState::CLOSED)
                openHand();
            else if (handState == HandState::PINCHED)
                unpinch();
        }

        resetEMG();
        triggerEMG = TriggerEMG::NONE;
        // delay(200);
    }
}

void resetEMG() {
    // emgs[0]->getEnvelope() = 0;
    // emgs[1]->getEnvelope() = 0;
    // emgs[0]->pulse = false;
    // emgs[1]->pulse = false;
    // emgs[0]->getActive() = false;
    // emgs[1]->getActive() = false;
    for (int i = 0; i < numEmgChannels; i++) emgs[i]->resetVars();
}

void parseCmd(const char* cmd) {
    printf("Received: '%s'\n", cmd);

    int val = atoi(cmd + 1);

    if (strcmp(cmd, "o") == 0) openHand();
    else if (strcmp(cmd, "of") == 0) openFingers();
    else if (strcmp(cmd, "c") == 0) closeHand();
    else if (strcmp(cmd, "cf") == 0) closeFingers(255);
    else if (strcmp(cmd, "s") == 0) stopFingers();
    else if (strcmp(cmd, "p") == 0) pinch();
    else if (strcmp(cmd, "u") == 0) unpinch();
    else if (strcmp(cmd, "uw") == 0) unblockWrist();
    else if (strcmp(cmd, "bw") == 0) blockWrist();
    else if (strcmp(cmd, "pm") == 0) printEmgEnvelope = !printEmgEnvelope;
    else if (strcmp(cmd, "pr") == 0) printRawEmg = !printRawEmg;
    else if (strcmp(cmd, "ps") == 0) printFilteredEmg = !printFilteredEmg;
    else if (strcmp(cmd, "ptime") == 0) printT = !printT;
    else if (strcmp(cmd, "ptrigger") == 0) printTrigger = !printTrigger;
    else if (strcmp(cmd, "control") == 0) triggerControl = !triggerControl;
    else if (strcmp(cmd, "ptemp") == 0) printTemperature = !printTemperature;
    else if (strcmp(cmd, "pbat") == 0) printBattery = !printBattery;
    else if (strcmp(cmd, "pnn") == 0) printNN = !printNN;
    else if (strcmp(cmd, "nn") == 0) enableNeuralNetwork = !enableNeuralNetwork;
    else if (cmd[0] == 'w' && val >= 0 && val <= 100) rotateWrist(val);
    else if (cmd[0] == 'c' && val >= 0 && val <= 255) closeFingers(val);
    else if (strcmp(cmd, "th") == 0) {
        enableThresholdControl = !enableThresholdControl;
        printf("Control por umbrales: %s\n", enableThresholdControl ? "ON" : "OFF");
    }
}

void moveset1() {
    static NonBlockingTimer movesetTimer;
    static int moveCounter = -1;
    if (movesetTimer.check()) {
        moveCounter++;
        switch (moveCounter) {
            case 1:
                closeHand();
                movesetTimer.setMillis(500);
                break;
            case 2:
                openHand();
                movesetTimer.setMillis(500);
                break;
            case 3:
                unpinch();
                break;
            case 4:
                pinch();
                break;
            case 5:
                rotateWrist(100);
                movesetTimer.setMillis(500);
                break;
            case 6:
                fingers[0]->forward(255);
                fingers[1]->reverse(255);
                break;
            case 7:
                rotateWrist(0);
                movesetTimer.setMillis(500);
                break;
            case 8:
                openHand();
                break;
            case 9:
                rotateWrist(50);
                break;
            default:
                moveCounter = 0;
                movesetTimer.setMillis(5000);
                break;
        }
    }
}

void moveset2() {
    static NonBlockingTimer movesetTimer;
    static int moveCounter = -1;
    if (movesetTimer.check()) {
        moveCounter++;
        switch (moveCounter) {
            case 1:
                rotateWrist(0);
                movesetTimer.setMillis(1000);
                break;
            case 2:
                rotateWrist(50);
                movesetTimer.setMillis(1000);
                break;
            case 3:
                rotateWrist(100);
                movesetTimer.setMillis(1000);
                break;
            default:
                moveCounter = 0;
                break;
        }
    }
}

void smoothControl() {
    if (emgs[0]->getSmooth() > 10 && emgs[0]->getSmooth() > emgs[1]->getSmooth()) {
        for (int i = 0; i < 2; i++)
            fingers[i]->forward(emgs[0]->getSmooth() * 2.55);
    } else if (emgs[1]->getSmooth() > 10 && emgs[1]->getSmooth() > emgs[0]->getSmooth()) {
        for (int i = 0; i < 2; i++)
            fingers[i]->reverse(emgs[0]->getSmooth() * 2.55);
    } else
        stopFingers();
}

void buzzMelody() {
    NonBlockingTimer timer1;
    NonBlockingTimer timer2;
    
    timer1.setMillis(100);    
    while (!timer1.check()) {
        for (int i = 0; i < 2; i++) fingers[i]->forward(70);
        timer2.delayMicros(80);
        for (int i = 0; i < 2; i++) fingers[i]->reverse(70);
        timer2.delayMicros(80);
    }

    for (int i = 0; i < 2; i++) fingers[i]->stop();
    timer1.delayMillis(100);

    timer1.setMillis(100);
    while (timer1.check()) {
        for (int i = 0; i < 2; i++) fingers[i]->forward(70);
        timer2.delayMicros(70);
        for (int i = 0; i < 2; i++) fingers[i]->reverse(70);
        timer2.delayMicros(70);
    }

    for (int i = 0; i < 2; i++) fingers[i]->stop();
    timer1.delayMillis(100);

    timer1.setMillis(100);
    while (timer1.check()) {
        for (int i = 0; i < 2; i++) fingers[i]->forward(70);
        timer2.delayMicros(60);
        for (int i = 0; i < 2; i++) fingers[i]->reverse(70);
        timer2.delayMicros(60);
    }

    for (int i = 0; i < 2; i++) fingers[i]->stop();
}

// ----------------------------------------------------------------------------------------------------
void secondarySensorsSamplingTask(void *arg) {
    while(1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        readSecondarySensors();
    }
}

// TODO: Can send binary or compressed data if jittering
void printingTask(void *arg) {
    Sample s;
    int mavIdx = 0;
    bool mavPrintDone = true;
    while (true) {
        while (xQueueReceive(emgSampleQueue, &s, 0) == pdTRUE) {
            if (emgSampleQueueFull) {
                emgSampleQueueFull = false;
                leds[0] = Rgb { 50, 0, 0 };
                leds.show();
            }
            if (printRawEmg) {
                printf(">");
                for (int i = 0; i < numEmgChannels; i++) {
                    printf("Raw%d:%u", i, s.rawSamples[i]);
                    if (i < numEmgChannels - 1)
                        printf(",");
                }
               
            printf("\n");
            }    
            if (printFilteredEmg)
                for (int i = 0; i < numEmgChannels; i++)
                    printf(">Sample%d:%f\n", i, s.samples[i]);
        }
        vTaskDelay(1);  // Yield 1 tick to other tasks

        if (!mavPrintDone && emgEnvelopeBuffer.flag) {
            leds[0] = Rgb { 0, 0, 50 };
            leds.show();
        }
        if (emgEnvelopeBuffer.flag || !mavPrintDone) {
            emgEnvelopeBuffer.flag = false;
            mavPrintDone = false;
            printf(">Mav%d:%.3g\n", mavIdx, emgEnvelopeBuffer.values[mavIdx][emgEnvelopeBuffer.index]);
            mavIdx++;
            if (mavIdx >= numEmgChannels) {
                mavPrintDone = true;
                mavIdx = 0;
            }
        }

        // printf(">ch:%u\n", rawEmgSampleBuffer[0]);

        // printf(">ch:%f\n", emgs[0]->getEnvelope());

        // printf(">frequency: %f\n", 1./elapsedTime*1e6);
        // printf(">time (ms): %f\n", float(elapsedTime/1000.0));
        
        // for (int i = 0; i < maxInfo; i++) {
        //     printf(">%d:%u\n", i, rawEmgSampleBuffer[i]);
        // }


        // printf("\n");
        // if (ovf) printf(">aaaaaaaa\n");
        // ovf = false;
        
    }
}

void timeCheckingTask(void *param) {
    uint32_t timestamp;
    bool ledOn = false;
    int initialCounter = 5;
    while (true) {
        if (xTaskNotifyWait(0, UINT32_MAX, &timestamp, portMAX_DELAY) == pdTRUE) {
            if (printT)
                printf(">T:%lu\n", timestamp);
            if (timestamp > 1100) {
                if (initialCounter) {
                    initialCounter--;
                } else {
                    ledOn = !ledOn;
                    if (ledOn)
                        leds[0] = Rgb { 10, 10, 10 };
                    else
                        leds[0] = Rgb { 0, 0, 0 };
                    leds.show();
                }
            }
        }
    }
}




// prueva para ver si los sensores funcionan

/*
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"

void adcTestTask(void *arg)
{
    while (1) {
        int raw0 = adc1_get_raw(ADC1_CHANNEL_3); // GPIO4
        int raw1 = adc1_get_raw(ADC1_CHANNEL_4); // GPIO5

        printf("R0:%d R1:%d\n", raw0, raw1);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

extern "C" void app_main(void)
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);

    xTaskCreate(adcTestTask, "adcTest", 4096, NULL, 5, NULL);
}
*/
