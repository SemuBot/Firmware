#ifndef STEPPERMOTORS_H
#define STEPPERMOTORS_H
#include <Arduino.h>

#define CS 6
#define DIR 4
#define STEP_PIN 5
#define STEPS_PER_ROTATION 200
#define CURRENT 800
#define DELAY_VALUE 500
#define SPEED 20

extern uint16_t encoderValue;


void motorSetup();
void down(int steps, int speedToSet);
void down2(int steps, int speedToSet);
void up(int steps, int speedToSet);
void stop();
void setSpeed(int speed);
char isMotorMoving();
#endif // STEPPERMOTORS_H