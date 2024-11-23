/*
 * motor.h
 *
 *  Created on: Nov 18, 2024
 *      Author: medved
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "stm32l4xx_hal.h"

#define DUTY_CYCLE_LIMIT_DEFAULT 100
#define MAX_PWM_VALUE 1000
#define MAX_LINEAR_VELOCITY 1.0f
#define MIN_LINEAR_VELOCITY -1.0f
#define DUTY_CYCLE_LIMIT 100

#define ROBOT_RADIUS 0.05f // Robot radius (distance from center to each motor, in meters)
#define PWM_SCALING_FACTOR 100.0f

#define MOTOR1_ANGLE 0.0f        // Motor 1 at 0 degrees
#define MOTOR2_ANGLE (2.0f * M_PI / 3.0f) // Motor 2 at 120 degrees
#define MOTOR3_ANGLE (4.0f * M_PI / 3.0f) // Motor 3 at 240 degrees

typedef struct
{
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    TIM_HandleTypeDef *pwm_timer;
    GPIO_TypeDef *pwm_port;
    uint16_t pwm_pin;
    double duty_cycle;
    uint8_t duty_cycle_limit;

} motor_st;

extern motor_st motor1;
extern motor_st motor2;
extern motor_st motor3;

void motor_init(motor_st *motor_handler);
void motor_update(motor_st *motor_handler);
void motor_enable(motor_st *motor_handler);
void motor_disable(motor_st *motor_handler);
void motor_setDutyCycleLimit(motor_st *motor_handler, uint8_t limit);
void calculate_motor_duty_cycles(float linear_x, float linear_y, float omega, motor_st *motor1, motor_st *motor2, motor_st *motor3);

#endif /* INC_MOTOR_H_ */

