/*
 * motor.h
 *
 *  Created on: Nov 18, 2024
 *      Author: medved
 *  Updated: Dec 09, 2024 - Added DRV8353 SPI support
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "stm32f3xx_hal.h"
#include "drv8353.h"

#define DUTY_CYCLE_LIMIT 100

typedef struct {
    GPIO_TypeDef *dir_port;                  // Direction GPIO port
    uint16_t dir_pin;                        // Direction GPIO pin
    TIM_HandleTypeDef *pwm_timer;            // PWM timer handle
    GPIO_TypeDef *pwm_port;                  // PWM GPIO port
    uint16_t pwm_pin;                        // PWM GPIO pin
    float duty_cycle;                        // Current duty cycle (-100 to +100)
    DRV8353_Handle *drv_handle;              // DRV8353 driver handle
} motor_st;

// Global motor instances
extern motor_st motor1;
extern motor_st motor2;
extern motor_st motor3;



#endif /* INC_MOTOR_H_ */
