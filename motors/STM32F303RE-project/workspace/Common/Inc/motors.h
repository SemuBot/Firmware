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

struct Motors{ // Check number sizes
  int8_t DIR_PIN;
  int8_t STEP_PIN;
  int8_t EN_PIN;
  int8_t SPEED;
  int16_t STEPS;
  TIM_TypeDef  *TIMER;
  GPIO_TypeDef *EN_PORT;
  GPIO_TypeDef *DIR_PORT;
  bool moving;
} ;

void moveMotor(struct Motors* motor);
int clamp(int value, int min, int max);
#endif /* MOTORS_H */
