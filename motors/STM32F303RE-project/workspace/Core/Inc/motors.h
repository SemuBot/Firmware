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
#include "usart.h"


typedef struct { // Check number sizes
	// Conf values
	uint16_t en_pin;
	uint16_t dir_pin;
	uint16_t pul_pin;
	GPIO_TypeDef* en_port;
	GPIO_TypeDef* dir_port;
	GPIO_TypeDef* pul_port;

	TIM_HandleTypeDef* timer;

	// Runtime values
	int8_t speed;
	uint8_t cmd_steps;
	uint16_t steps_left;
	volatile bool pulse_finished;
} motor_st;

#define MOTOR_STEPS_PER_REVOLUTION 800
#define MOTOR_CCR1_START_VALUE 75

void MOTOR_init();
void MOTOR_set_all(usart_buffer_st* buf);
void MOTOR_interrupt(TIM_HandleTypeDef* htim);
void MOTOR_main();
void MOTOR_test();


#endif /* MOTORS_H */
