#include "globals.h"

float temperature = 0;
float batteryLevel = 0;

extern uint16_t rawEmgSampleBuffer[numEmgChannels];

HandState handState = HandState::OPEN;
EMGController *emgs[numEmgChannels];
uint16_t rawEmgSampleBuffer[numEmgChannels];
QueueHandle_t emgSampleQueue;
EmgEnvelopeBuffer emgEnvelopeBuffer;


bool emgCalibrated = false;
volatile bool emgSampleQueueFull = false;