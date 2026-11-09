#include "movements.h"
#include "globals.h"
#include "esp_timer.h" 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"

// --- CLASES ORIGINALES INTACTAS ---
DcDriver::DcDriver(uint8_t pwmPin, uint8_t dir1Pin, uint8_t dir2Pin, ledc_channel_t pwmChannel, ledc_timer_t timer)
        : _pwm(pwmPin, pwmChannel, timer), _dir1Pin((gpio_num_t)dir1Pin), _dir2Pin((gpio_num_t)dir2Pin) { 
        gpio_set_direction(_dir1Pin, GPIO_MODE_OUTPUT);
        gpio_set_direction(_dir2Pin, GPIO_MODE_OUTPUT);
        stop(); 
    }

void DcDriver::forward(uint8_t speed) {
    gpio_set_level(_dir1Pin, 1);
    gpio_set_level(_dir2Pin, 0);
    _pwm.write((uint32_t)speed);
}

void DcDriver::reverse(uint8_t speed) {
    gpio_set_level(_dir1Pin, 0);
    gpio_set_level(_dir2Pin, 1);
    _pwm.write((uint32_t)speed);
}

void DcDriver::stop() {
    gpio_set_level(_dir1Pin, 0);
    gpio_set_level(_dir2Pin, 0);
    _pwm.write(0);
}

class TwoPinLinearActuator {
private:
    ledc_channel_t _ch1;
    ledc_channel_t _ch2;
public:
    TwoPinLinearActuator(uint8_t pin1, uint8_t pin2, ledc_channel_t ch1, ledc_channel_t ch2, ledc_timer_t timer) {
        _ch1 = ch1; _ch2 = ch2;
        ledc_channel_config_t conf1 = { .gpio_num = pin1, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = ch1, .intr_type = LEDC_INTR_DISABLE, .timer_sel = timer, .duty = 0, .hpoint = 0 };
        ledc_channel_config(&conf1);
        ledc_channel_config_t conf2 = { .gpio_num = pin2, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = ch2, .intr_type = LEDC_INTR_DISABLE, .timer_sel = timer, .duty = 0, .hpoint = 0 };
        ledc_channel_config(&conf2);
    }
    void extend(uint32_t speed) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, _ch2, 0); ledc_update_duty(LEDC_LOW_SPEED_MODE, _ch2);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, _ch1, speed); ledc_update_duty(LEDC_LOW_SPEED_MODE, _ch1);
    }
    void retract(uint32_t speed) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, _ch1, 0); ledc_update_duty(LEDC_LOW_SPEED_MODE, _ch1);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, _ch2, speed); ledc_update_duty(LEDC_LOW_SPEED_MODE, _ch2);
    }
    void stop() {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, _ch1, 0); ledc_update_duty(LEDC_LOW_SPEED_MODE, _ch1);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, _ch2, 0); ledc_update_duty(LEDC_LOW_SPEED_MODE, _ch2);
    }
};

static Servo wristBrake1(wristBrake1Pin, LEDC_CHANNEL_0, LEDC_TIMER_0);
static Servo wristBrake2(wristBrake2Pin, LEDC_CHANNEL_1, LEDC_TIMER_0);
static Servo wrist(wristPin, LEDC_CHANNEL_2, LEDC_TIMER_1);
static TwoPinLinearActuator thumbActuator(thumbIn1Pin, thumbIn2Pin, LEDC_CHANNEL_3, LEDC_CHANNEL_4, LEDC_TIMER_2);

static DcDriver finger1(fingersSpeedPins[0], fingersDirPins1[0], fingersDirPins2[0], LEDC_CHANNEL_5, LEDC_TIMER_3);
static DcDriver finger2(fingersSpeedPins[1], fingersDirPins1[1], fingersDirPins2[1], LEDC_CHANNEL_6, LEDC_TIMER_3);

DcDriver* fingers[] = { &finger1, &finger2 };
const int numFingers = sizeof(fingers) / sizeof(fingers[0]);
static int wristPosition;

// =========================================================================
// NUEVA ARQUITECTURA NO BLOQUEANTE (MÁQUINA DE ESTADOS)
// =========================================================================

// Temporizadores independientes
uint64_t thumbStopTime = 0;
uint64_t fingersStopTime = 0;

// Máquina de estados para la muñeca
enum class WristState { IDLE, UNBLOCKING, ROTATING, BLOCKING };
WristState wristState = WristState::IDLE;
uint64_t wristTimer = 0;
int targetWristPos = 0;

// Máquina de estados para secuencia especial del Unpinch
enum class HandSeqState { IDLE, UNPINCH_THUMB };
HandSeqState handSeqState = HandSeqState::IDLE;


void initMovements() {
    blockWrist();
    openHand();
}

// ---------------- FUNCIONES BASE DE DEDOS ----------------
void openFingers() {
    for (int i = 0; i < numFingers; i++) {
        fingers[i]->forward(255);
    }
}

void closeFingers(int v) {
    for (int i = 0; i < numFingers; i++) {
        fingers[i]->reverse(v);
    }
}

void stopFingers() {
  for (int i = 0; i < numFingers; i++)
    fingers[i]->stop();
}

void closePinchFingers(int v) {
    finger1.reverse(v);
}

void openPinchFingers(int v) {
    finger1.forward(v);
}

void stopPinchFingers() {
    finger1.stop();
}

void blockWrist() {
    wristBrake1.writeMicroseconds(2000 - wristBrakeBlocked * 10);
    wristBrake2.writeMicroseconds(2000 - wristBrakeBlocked * 10);
}

void unblockWrist() {
    wristBrake1.writeMicroseconds(2000 - wristBrakeUnblocked * 10);
    wristBrake2.writeMicroseconds(2000 - wristBrakeUnblocked * 10);
}

// ---------------- FUNCIONES DE ALTO NIVEL (¡Lanzar y olvidar!) ----------------

void openHand() {
    if (handState != HandState::OPEN) {
        printf("Opening Hand\n");
        openFingers();
        thumbActuator.retract(255);
        
        uint64_t now = esp_timer_get_time();
        fingersStopTime = now + (timeFingersOpening * 1000ULL);
        thumbStopTime = now + (timeThumbRetracting * 1000ULL);
        handSeqState = HandSeqState::IDLE; // Cancelamos secuencias pendientes
    }
    handState = HandState::OPEN;
}

void closeHand() {
    if (handState != HandState::CLOSED) {
        printf("Closing Hand\n");
        closeFingers(255);
        thumbActuator.extend(255); 
        
        uint64_t now = esp_timer_get_time();
        fingersStopTime = now + (timeFingersClosing * 1000ULL);
        thumbStopTime = now + (timeThumbExtending * 1000ULL);
        handSeqState = HandSeqState::IDLE;
    }
    handState = HandState::CLOSED;
}

void pinch() {

    if (handState != HandState::PINCHED) {

        printf("Pinching\n");

        closePinchFingers(255);
        thumbActuator.extend(255);

        uint64_t now = esp_timer_get_time();

        fingersStopTime = now + (timePinch * 1000ULL);
        thumbStopTime = now + (timeThumbExtending * 1000ULL);

        handSeqState = HandSeqState::IDLE;
    }

    handState = HandState::PINCHED;
}

void unpinch() {

    uint64_t now = esp_timer_get_time();

    if (handState != HandState::UNPINCHED) {

        printf("Unpinching\n");

        openPinchFingers(255);

        thumbActuator.retract(255);

        fingersStopTime = now + (timeFingersOpening * 1000ULL);
        thumbStopTime = now + (timeThumbRetracting * 1000ULL);

        handSeqState = HandSeqState::IDLE;
    }

    handState = HandState::UNPINCHED;
}

void controlledPinch(int v) {
    printf("Performing Controlled Pinch\n");
    thumbActuator.extend(v);
    closeFingers(v);
    handState = HandState::PINCHED;
    // Como es controlado desde fuera, no seteamos stopTime.
    fingersStopTime = 0; 
    thumbStopTime = 0;
}

void rotateWrist(int pos) {
    if (pos < 15) pos = 5;
    else if (pos < 43) pos = 30;
    else if (pos < 68) pos = 54;
    else if (pos < 89) pos = 78;
    else pos = 99;

    if (wristPosition != pos) {
        targetWristPos = pos;
        printf("Rotating wrist. Target: %d\n", pos);
        
        unblockWrist(); // Iniciamos Fase 1
        wristState = WristState::UNBLOCKING;
        wristTimer = esp_timer_get_time() + (timeWristUnblocking * 1000ULL);
    }
}

// =========================================================================
// EL "CORAZÓN" DE LOS MOTORES - DEBE LLAMARSE CONTINUAMENTE DESDE EL MAIN
// =========================================================================
void updateMovements() {
    uint64_t now = esp_timer_get_time();

    // 1. Apagar Pulgar si su tiempo terminó
    if (thumbStopTime > 0 && now >= thumbStopTime) {
        thumbActuator.stop();
        thumbStopTime = 0;
    }

    // 2. Apagar Dedos si su tiempo terminó
    if (fingersStopTime > 0 && now >= fingersStopTime) {
        stopFingers();
        fingersStopTime = 0;
    }

    // 3. Evaluar secuencias especiales (Unpinch Secuencial)
    if (handSeqState == HandSeqState::UNPINCH_THUMB && thumbStopTime == 0) {
        // El pulgar ya terminó de retraerse. Ahora abrimos dedos.
        openFingers();
        fingersStopTime = now + (timeFingersOpening * 1000ULL);
        handSeqState = HandSeqState::IDLE;
    }

    // 4. Máquina de estados de la MUÑECA (Fases automáticas sin delay)
    if (wristState == WristState::UNBLOCKING && now >= wristTimer) {
        // Fase 1 terminada: Freno quitado. Procedemos a Fase 2: Girar.
        int angleDifference = abs(wristPosition - targetWristPos);
        wristPosition = targetWristPos;
        wrist.writeMicroseconds(2500 - targetWristPos * 18);
        
        wristState = WristState::ROTATING;
        wristTimer = now + (angleDifference * timeWristUnitRotation * 1000ULL);
    }
    else if (wristState == WristState::ROTATING && now >= wristTimer) {
        // Fase 2 terminada: Giro completado. Procedemos a Fase 3: Poner Freno.
        blockWrist();
        wristState = WristState::BLOCKING;
        wristTimer = now + (timeWristBlocking * 1000ULL);
    }
    else if (wristState == WristState::BLOCKING && now >= wristTimer) {
        // Fase 3 terminada: Freno puesto. Volvemos a reposo.
        wristState = WristState::IDLE;
    }
}