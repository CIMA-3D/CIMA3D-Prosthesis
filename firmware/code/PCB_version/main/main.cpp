#include <freertos/FreeRTOS.h>
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <math.h>

#include "SmartLeds.h"
// #include "PwmController.h" 

#include "EMGController.h"
#include "NonBlockingTimer.h"
#include "nn_model.h"
#include "NeuralNetwork.h"
#include "oled_display.h"

#include "AdcSingleRead.h"

#include "emg_continuous_sampling.h"
#include "globals.h"
#include "movements.h"
#include "secondary_sensors.h"
#include "uart_command_handler.h"


// THE TICK RATE IS SET TO 100 Hz, so 1 tick = 10 ms!!!

bool enableCustomPrintingTask = false;
bool enableNeuralNetwork = true;

bool samplePrimaryEmgSensors = true;
bool sampleSecondarySensors = true;

bool printT = false;
bool printRawEmg = false;
bool printFilteredEmg = false;
bool printEmgEnvelope = true;
bool squareEmgEnvelope = false;
bool printNN = true;

bool printTrigger = false;
bool triggerControl = false;

bool printBattery = false;
bool printTemperature = false;

const uint16_t calibrationTime = 0e3; // in millis. FIX EMG CALIBRATION (INIT) ROUTINE
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

void customPrintingTask(void *arg) {
    NonBlockingTimer timer;
    static uint16_t printingInterval = 100; // millis
    while(1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (timer.check()) {
            for (int i = 0; i < numEmgChannels; i++)
                printf(">Raw%i:%d\n", i, rawEmgSampleBuffer[i]);
            timer.setMillis(printingInterval);
        }
        vTaskDelay(pdMS_TO_TICKS(1));  // 10 ms delay for CPU
    }
}

// =====================================================================================================

// Runs on Core 0 (Main Task) - Print, UI (OLED), UART, Lógica general
void mainTask(void *pvParameters) {
    NonBlockingTimer emgTriggerPrintTimer;
    NonBlockingTimer otherSensorsPrintTimer;
    NonBlockingTimer oledUpdateTimer; 

    while (1) {
        if (oledUpdateTimer.check()) {
            updateOLED("ACTIVA", currentBatteryState);
            oledUpdateTimer.setMillis(2000);
        }
        
        if (otherSensorsPrintTimer.check()) {
            if (printBattery) printf(">Bat:%.3g\n", batteryLevel);
            if (printTemperature) printf(">Temp:%.3g\n", temperature);
            otherSensorsPrintTimer.setMillis(50);
        }
        
        checkBatteryLevel();
        pollUart(parseCmd);

        if (nnInput.nnFilledInputBufferIndex+1) {
            float** x = nnInput.getFeatures(nnInput.nnFilledInputBufferIndex);

            adcToNnFlag = false;
            while (!adcToNnFlag);
            int predictedLabel = nn->run(x);
            
            float* probs = nn->getOutput();
            nnInput.updateLastPredictions(nnInput.nnFilledInputBufferIndex, probs);

            // 1. FILTRO ANTI-TEMBLORES ACTIVADO
            predictedLabel = nnOutPp.process(probs);

            if (printNN) printf(">NN:%d\n", predictedLabel);
            
            if (enableNeuralNetwork && !triggerControl) { 
                static int lastLabel = -1; 
                
                static int wristPosition = 50; 
                static int wristDirection = 1; 
                static NonBlockingTimer wristTimer; 

                // A. ACCIONES CONTINUAS 
                if (predictedLabel == 3) { // 3 = GIRAR
                    if (wristTimer.check()) {
                        wristPosition += (5 * wristDirection); 
                        if (wristPosition > 100) wristPosition = 100;
                        if (wristPosition < 0) wristPosition = 0;
                        
                        rotateWrist(wristPosition);
                        wristTimer.setMillis(50); 
                    }
                }

                // B. ACCIONES DE UN SOLO TOQUE 
                if (predictedLabel != lastLabel) {
                    if (lastLabel == 3) {
                        wristDirection = -wristDirection; // Invierte el sentido
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
                        case 3: // GIRAR 
                            wristTimer.setMillis(0); 
                            break;
                    }
                    lastLabel = predictedLabel; 
                }
            }

            if (printTrigger && emgTriggerPrintTimer.check()) {
                printf(">Trigger:%d\n", static_cast<int>(triggerEMG));
                if (!triggerControl)
                    triggerEMG = TriggerEMG::NONE;  
                emgTriggerPrintTimer.setMillis(500);
            }
            if (triggerControl) emgControl();

            // checkTouchFeedback(); // TODO: Descomentar cuando implementemos el tacto I2C

            vTaskDelay(1);  // 10 ms delay for CPU
        }
    }
}

extern "C" void app_main(void) {
    esp_log_level_set("*", ESP_LOG_WARN); 

    beginUart(115200);
    printf("\n\n");

    leds[0] = Rgb {0,10,0};
    leds.show();

    initMovements();
    initSecondarySensors();

    nn = new NN(nn_model);
    nnOutPp.init(0.3f, 0.9f, 0.3f, 2, 2);

    for (int i = 0; i < numEmgChannels; i++) {
        emgs[i] = new EMGController(i, &rawEmgSampleBuffer[i]);
        emgs[i]->setMaxDuration(1000);
        emgs[i]->setHysteresis(300);
        emgs[i]->setFreezeDuration(300);
        emgs[i]->setSquareEnvelope(squareEmgEnvelope);
        emgs[i]->setThreshold(30);
    }

    initOLED();
    updateOLED("LISTA", 100);

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


void samplingTask(void *arg) {
    int counter = 0;
    uint64_t lastTime = esp_timer_get_time();
    uint64_t now;
    uint16_t calibrationCounter = 0;
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (samplePrimaryEmgSensors)
            readAdc();

        if (sampleSecondarySensors) {
            static int secondarySensorCounter = 0;
            if (++secondarySensorCounter >= 10) { 
                secondarySensorCounter = 0;
                xTaskNotifyGive(secondarySensorsSamplingTaskHandle);
            }
        }

        if (!samplePrimaryEmgSensors) continue;

        if (!emgCalibrated) {
            if (calibrationCounter >= calibrationTime) calibrationCounter = 0;
            calibrationCounter++;
            if (calibrationCounter >= calibrationTime) emgCalibrated = true;
            continue;
        }

        now = esp_timer_get_time(); 
        uint32_t T = now - lastTime;
        lastTime = now;
        xTaskNotify(timeCheckingTaskHandle, T, eSetValueWithOverwrite);
        adcToNnFlag = true;

        for (int i = 0; i < numEmgChannels; i++) {
            emgs[i]->applyPowerLineFilter();
            emgs[i]->applyLPFilter();
            emgs[i]->normalize();
            emgs[i]->update();
            if (enableCustomPrintingTask) emgs[i]->updateSmooth();
        }

        if (enableNeuralNetwork) nnInput.update(emgs);

        if (printFilteredEmg || printRawEmg) {
            Sample s;
            if (printRawEmg)
                for (int i = 0; i < numEmgChannels; i++) s.rawSamples[i] = rawEmgSampleBuffer[i];
            if (printFilteredEmg)
                for (int i = 0; i < numEmgChannels; i++) s.samples[i] = emgs[i]->getFilteredSample();
            if (uxQueueSpacesAvailable(emgSampleQueue) == 0) {
                emgSampleQueueFull = true;
                Sample tmp;
                while (xQueueReceive(emgSampleQueue, &tmp, 0) == pdTRUE); 
            }
            xQueueSend(emgSampleQueue, &s, 0);
        }

        if (printEmgEnvelope) {
            counter++;
            if (counter >= downsamplingWindow) {
                counter = 0;
                emgEnvelopeBuffer.index = (emgEnvelopeBuffer.index + 1) % EMG_ENVELOPE_BUFFER_SIZE;
                for (int i = 0; i < numEmgChannels; i++)
                    emgEnvelopeBuffer.values[i][emgEnvelopeBuffer.index] = emgs[i]->getEnvelope();               
                emgEnvelopeBuffer.flag = true;
            }
        }

        if (customPrintingTaskHandle != NULL) xTaskNotifyGive(customPrintingTaskHandle);
    }
}

void updateEmgTrigger() {
    static int counterC = 0;
    static int counterA = 0;

    if (emgs[0]->pulse) {
        emgs[0]->pulse = false;
        emgs[1]->pulse = false;
        if (counterC++ >= 2) triggerEMG = TriggerEMG::CLOSE;
    } else if (emgs[1]->pulse) {
        emgs[1]->pulse = false;
        if (counterA++ >= 2) triggerEMG = TriggerEMG::OPEN;
    }
}

void emgControl() {
    if (triggerEMG != TriggerEMG::NONE) {
        if (printTrigger) printf(">Trigger:%d\n", static_cast<int>(triggerEMG));
        
        if (triggerEMG == TriggerEMG::CLOSE) {
            if (handState == HandState::PINCHED || handState == HandState::OPEN) {
                unpinch();
            } else if (handState == HandState::UNPINCHED) {
                controlledPinch(100);
                unsigned long tiempo = esp_timer_get_time();
                while (!emgs[0]->getActive() && !emgs[1]->getActive() && esp_timer_get_time() - tiempo < 1000*timePinching*2)
                    vTaskDelay(1);
                stopFingers();
            }
        } else if (triggerEMG == TriggerEMG::OPEN) {
            if (handState == HandState::UNPINCHED || handState == HandState::CLOSED) {
                openHand();
            } else if (handState == HandState::PINCHED) {
                unpinch();
            }
        }

        resetEMG();
        triggerEMG = TriggerEMG::NONE;
    }
}

void resetEMG() {
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
    // else if (strcmp(cmd, "uw") == 0) unblockWrist(); // Descomentar cuando añadamos los frenos
    // else if (strcmp(cmd, "bw") == 0) blockWrist();   // Descomentar cuando añadamos los frenos
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
    else if (cmd[0] == 'b' && val >= 0 && val <= 100) moveThumbBase(val);
    else if (cmd[0] == 't' && val >= 0 && val <= 100) moveThumbTip(val);
    else if (cmd[0] == 'w' && val >= 0 && val <= 100) rotateWrist(val);
    else if (cmd[0] == 'c' && val >= 0 && val <= 255) closeFingers(val);
}

void moveset1() {
    static NonBlockingTimer movesetTimer;
    static int moveCounter = -1;
    if (movesetTimer.check()) {
        moveCounter++;
        switch (moveCounter) {
            case 1: closeHand(); movesetTimer.setMillis(500); break;
            case 2: openHand(); movesetTimer.setMillis(500); break;
            case 3: unpinch(); break;
            case 4: pinch(); break;
            case 5: rotateWrist(100); movesetTimer.setMillis(500); break;
            case 6: fingers[0]->forward(255); fingers[1]->reverse(255); break;
            case 7: rotateWrist(0); movesetTimer.setMillis(500); break;
            case 8: openHand(); break;
            case 9: rotateWrist(50); break;
            default: moveCounter = 0; movesetTimer.setMillis(5000); break;
        }
    }
}

void moveset2() {
    static NonBlockingTimer movesetTimer;
    static int moveCounter = -1;
    if (movesetTimer.check()) {
        moveCounter++;
        switch (moveCounter) {
            case 1: rotateWrist(0); movesetTimer.setMillis(1000); break;
            case 2: rotateWrist(50); movesetTimer.setMillis(1000); break;
            case 3: rotateWrist(100); movesetTimer.setMillis(1000); break;
            default: moveCounter = 0; break;
        }
    }
}

void smoothControl() {
    if (emgs[0]->getSmooth() > 10 && emgs[0]->getSmooth() > emgs[1]->getSmooth()) {
        for (int i = 0; i < 2; i++) fingers[i]->forward(emgs[0]->getSmooth() * 2.55);
    } else if (emgs[1]->getSmooth() > 10 && emgs[1]->getSmooth() > emgs[0]->getSmooth()) {
        for (int i = 0; i < 2; i++) fingers[i]->reverse(emgs[0]->getSmooth() * 2.55);
    } else {
        stopFingers();
    }
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

void secondarySensorsSamplingTask(void *arg) {
    while(1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        // readSecondarySensors(); // TODO: Descomentar cuando ajustemos el I2C del tacto
    }
}

void printingTask(void *arg) {
    Sample s;
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
                    if (i < numEmgChannels - 1) printf(",");
                }
                printf("\n");
            }  
            if (printFilteredEmg) {
                for (int i = 0; i < numEmgChannels; i++)
                    printf(">Sample%d:%f\n", i, s.samples[i]);
            }
        }
        vTaskDelay(1);  // Yield 1 tick to other tasks

        if (!mavPrintDone && emgEnvelopeBuffer.flag) {
            leds[0] = Rgb { 0, 0, 50 };
            leds.show();
        }
        if (emgEnvelopeBuffer.flag) {
            emgEnvelopeBuffer.flag = false;
            printf(">Envelope0:%f\n", emgEnvelopeBuffer.values[0][emgEnvelopeBuffer.index]);
            printf(">Envelope1:%f\n", emgEnvelopeBuffer.values[1][emgEnvelopeBuffer.index]);
        }
    }
}

void timeCheckingTask(void *param) {
    uint32_t timestamp;
    bool ledOn = false;
    int initialCounter = 5;
    while (true) {
        if (xTaskNotifyWait(0, UINT32_MAX, &timestamp, portMAX_DELAY) == pdTRUE) {
            if (printT) printf(">T:%lu\n", timestamp);
            if (timestamp > 1100) {
                if (initialCounter) {
                    initialCounter--;
                } else {
                    ledOn = !ledOn;
                    if (ledOn) leds[0] = Rgb { 10, 10, 10 };
                    else leds[0] = Rgb { 0, 0, 0 };
                    leds.show();
                }
            }
        }
    }
}