/*
 * motors.h
 *
 *  Created on: Mar 14, 2024
 *      Author: kaurk
 */
#ifndef MOTORS_H
#define MOTORS_H

#include "main.h"
#include "stdbool.h"
#define MOVE_DURATION 10 // 5 seconds in milliseconds

struct Motors {
  int DIR_PIN;
  int STEP_PIN;
  int EN_PIN;
  int SPEED;
  int STEPS;
  TIM_TypeDef  *TIMER;
  int CCR_VALUE;
  GPIO_TypeDef *EN_PORT;
  GPIO_TypeDef *DIR_PORT;
  bool moving;
};

void moveMotor(struct Motors* motor);
int clamp(int value, int min, int max);
#endif /* MOTORS_H */
