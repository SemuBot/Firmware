/*
 * motor.c
 *
 *  Created on: Nov 18, 2024
 *      Author: medved
 */


#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "motor.h"
#include "tim.h"
#include "stm32l4xx_hal.h"
#include "usart.h"



static uint8_t duty_cycle_limit;



motor_st motor1 = {
	.dir_port = MOTOR1_DIR_GPIO_Port,
	.dir_pin = MOTOR1_DIR_Pin,
    .pwm_timer = &htim1,
    .pwm_port = MOTOR1_PWM_GPIO_Port,
    .pwm_pin = MOTOR1_PWM_Pin,
    .duty_cycle = 0.0,
	.duty_cycle_limit = DUTY_CYCLE_LIMIT

};

motor_st motor2 = {
	.dir_port = MOTOR2_DIR_GPIO_Port,
	.dir_pin = MOTOR2_DIR_Pin,
    .pwm_timer = &htim3,
    .pwm_port = MOTOR2_PWM_GPIO_Port,
    .pwm_pin = MOTOR2_PWM_Pin,
    .duty_cycle = 0.0,
	.duty_cycle_limit = DUTY_CYCLE_LIMIT

};

motor_st motor3 = {
	.dir_port = MOTOR3_DIR_GPIO_Port,
	.dir_pin = MOTOR3_DIR_Pin,
    .pwm_timer = &htim4,
    .pwm_port = MOTOR3_PWM_GPIO_Port,
    .pwm_pin = MOTOR3_PWM_Pin,
    .duty_cycle = 0.0,
	.duty_cycle_limit = DUTY_CYCLE_LIMIT
};



void motor_init(motor_st *motor_data){
    duty_cycle_limit = DUTY_CYCLE_LIMIT_DEFAULT;
    motor_disable(motor_data);
}



void motor_update(motor_st *motor_data)
{
    uint16_t scaled_duty_cycle;
    uint16_t arr_value = __HAL_TIM_GET_AUTORELOAD(motor_data->pwm_timer);

    // Check if the motor is enabled
    if (motor_data->duty_cycle >= 0)
    {
        motor_enable(motor_data);
        HAL_GPIO_WritePin(motor_data->dir_port, motor_data->dir_pin, GPIO_PIN_SET);    }
    else if (motor_data->duty_cycle < 0)
    {
        HAL_GPIO_WritePin(motor_data->dir_port, motor_data->dir_pin, GPIO_PIN_RESET);
        motor_enable(motor_data);
    }
    else
    {
        motor_disable(motor_data);
        return;
    }

    // Scale the duty cycle based on the motor's duty cycle
    scaled_duty_cycle = (uint16_t)((fabs(motor_data->duty_cycle) / 100.0f) * arr_value);
    if (scaled_duty_cycle > arr_value)
    {
        scaled_duty_cycle = arr_value;
    }

    __HAL_TIM_SET_COMPARE(motor_data->pwm_timer, TIM_CHANNEL_1, scaled_duty_cycle);
    //char duty_cycle_str[50];
    //int len = snprintf(duty_cycle_str, sizeof(duty_cycle_str), "Duty Cycle: %.2f\n", motor_data->duty_cycle);
    //HAL_UART_Transmit(&huart2, (uint8_t *)duty_cycle_str, len, HAL_MAX_DELAY);
}



void calculate_motor_duty_cycles(float linear_x, float linear_y, float omega, motor_st *motor1, motor_st *motor2, motor_st *motor3) {

    float duty_cycle_1 = (linear_x * cosf(MOTOR1_ANGLE) + linear_y * sinf(MOTOR1_ANGLE) + (omega * ROBOT_RADIUS)) * PWM_SCALING_FACTOR;
    float duty_cycle_2 = (linear_x * cosf(MOTOR2_ANGLE) + linear_y * sinf(MOTOR2_ANGLE) + (omega * ROBOT_RADIUS)) * PWM_SCALING_FACTOR;
    float duty_cycle_3 = (linear_x * cosf(MOTOR3_ANGLE) + linear_y * sinf(MOTOR3_ANGLE) + (omega * ROBOT_RADIUS)) * PWM_SCALING_FACTOR;

    duty_cycle_1 = fmaxf(-100.0f, fminf(100.0f, duty_cycle_1));
    duty_cycle_2 = fmaxf(-100.0f, fminf(100.0f, duty_cycle_2));
    duty_cycle_3 = fmaxf(-100.0f, fminf(100.0f, duty_cycle_3));

    motor1->duty_cycle = duty_cycle_1;
    motor2->duty_cycle = duty_cycle_2;
    motor3->duty_cycle = duty_cycle_3;

    motor_update(motor1);
    motor_update(motor2);
    motor_update(motor3);
}


void motor_enable(motor_st *motor_data){
    HAL_TIM_PWM_Start(motor_data->pwm_timer, TIM_CHANNEL_1);
}


void motor_disable(motor_st *motor_data){
    HAL_TIM_PWM_Stop(motor_data->pwm_timer, TIM_CHANNEL_1);
}


