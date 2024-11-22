/*
 * motor.h
 *
 *  Created on: Nov 18, 2024
 *      Author: medved
 */

/*
 * motor.h
 *
 *  Created on: Nov 18, 2024
 *      Author: medved
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "stm32l4xx_hal.h"

typedef struct {
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
} Motor_pinout;

typedef struct
{
    Motor_pinout *pinout;
    TIM_HandleTypeDef *pwm_timer;
    GPIO_TypeDef *pwm_port;
    uint16_t pwm_pin;
    double duty_cycle;
} Motor_data;

void motor_init(Motor_data *motor_handler, Motor_pinout *pinout, TIM_HandleTypeDef *pwm_timer);
void motor_update(Motor_data *motor_handler);
void motor_enable(Motor_data *motor_handler);
void motor_disable(Motor_data *motor_handler);
void motor_setDutyCycleLimit(Motor_data *motor_handler, uint8_t limit);

#endif /* INC_MOTOR_H_ */

