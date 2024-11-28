/*
 * cmd_vel.c
 *
 *  Created on: Nov 22, 2024
 *      Author: medved
 */

#include "cmd_vel.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "usart.h"
#include "motor.h"

#define BUFFER_SIZE 256

static char usart_buffer[BUFFER_SIZE];
static uint8_t rx_data;

void USART_Receive_CmdVel(UART_HandleTypeDef *huart, CmdVel *cmd_vel) {
    static uint16_t index = 0;

    if (rx_data == '\n' || index >= BUFFER_SIZE - 1) {
        usart_buffer[index] = '\0';
        index = 0;

        if (strstr(usart_buffer, "linear_x") && strstr(usart_buffer, "angular_z")) {
            sscanf(
                usart_buffer,
                "{linear_x:%f,linear_y:%f,linear_z:%f,angular_x:%f,angular_y:%f,angular_z:%f}",
                &cmd_vel->linear_x,
                &cmd_vel->linear_y,
                &cmd_vel->linear_z,
                &cmd_vel->angular_x,
                &cmd_vel->angular_y,
                &cmd_vel->angular_z
            );

            Process_CmdVel(cmd_vel);

        } else {
            const char *error_msg = "Invalid cmd_vel format\n";
            HAL_UART_Transmit(huart, (uint8_t *)error_msg, strlen(error_msg), HAL_MAX_DELAY);
        }
    } else {
        usart_buffer[index++] = rx_data;
    }

    HAL_UART_Receive_IT(huart, &rx_data, 1);
}

void Process_CmdVel(CmdVel *cmd_vel) {
    char debug_msg[128];


    snprintf(
        debug_msg,
        sizeof(debug_msg),
        "Linear: [X: %.2f, Y: %.2f, Z: %.2f], Angular: [X: %.2f, Y: %.2f, Z: %.2f]\n",
        cmd_vel->linear_x,
        cmd_vel->linear_y,
        cmd_vel->linear_z,
        cmd_vel->angular_x,
        cmd_vel->angular_y,
        cmd_vel->angular_z
    );

    HAL_UART_Transmit(&huart2, (uint8_t *)debug_msg, strlen(debug_msg), HAL_MAX_DELAY);
}

