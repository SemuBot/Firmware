#ifndef STEPPERMOTORS_H
#define STEPPERMOTORS_H
#include <Arduino.h>
#include "pins.h"
#define STEPS_PER_ROTATION 200
#define CURRENT 800
#define DELAY_VALUE 500

extern uint16_t encoderValue;


void motorSetup();
void down(int steps, int speedToSet);
void up(int steps, int speedToSet);
void stop();
void test();
char isMotorMoving();
#endif // STEPPERMOTORS_H