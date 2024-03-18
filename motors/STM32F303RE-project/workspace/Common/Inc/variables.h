/*
 * variables.h
 *
 *  Created on: Mar 14, 2024
 *      Author: kaurk
 */
#include <stdbool.h>
 #include <stdint.h>


#ifndef INC_VARIABLES_H_
#define INC_VARIABLES_H_


#define MOTOR1_STEP_PIN GPIO_PIN_0
#define MOTOR1_STEP_PORT GPIOC

#define MOTOR1_DIR_PIN GPIO_PIN_1
#define MOTOR1_DIR_PORT GPIOC

#define MOTOR1_EN_PIN GPIO_PIN_3
#define MOTOR1_EN_PORT GPIOC





const int NUM_ENCODERS = 5;
const int NUM_MOTORS = 5;

struct Motor {
  int dirPin;
  int stepPin;
  int enPin;
  int speed;
  int steps;
  bool moving;
};

#endif /* INC_VARIABLES_H_ */
