/*
 * cmd_vel.h
 *
 *  Created on: Nov 22, 2024
 *      Author: medved
 */

#ifndef INC_CMD_VEL_H_
#define INC_CMD_VEL_H_


#include "stm32l4xx_hal.h"

typedef struct {
    float linear_x;
    float linear_y;
    float linear_z;
    float angular_x;
    float angular_y;
    float angular_z;
} CmdVel;

void USART_Receive_CmdVel(UART_HandleTypeDef *huart, CmdVel *cmd_vel);
void Process_CmdVel(CmdVel *cmd_vel);

#endif /* INC_CMD_VEL_H_ */
