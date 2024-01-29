#ifndef STEPPERMOTORS_H
#define STEPPERMOTORS_H
#include <Arduino.h>

#define CS 2
#define DIR 6
#define STEP_PIN 7
#define STEPS_PER_ROTATION 200
#define CURRENT 2000
#define DELAY_VALUE 500
#define SPEED 20
#define UPPER_LIMIT 30000
#define LOWER_LIMIT 100



extern int initialPosition;
extern uint16_t encoderValue;


void motorSetup();
void down(int steps, int speedToSet);
void up(int steps, int speedToSet);
void setSpeed(int speed);

#endif // STEPPERMOTORS_H