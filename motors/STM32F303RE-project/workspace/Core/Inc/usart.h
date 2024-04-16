/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */
#include "stdbool.h"

/* USER CODE END Includes */

extern UART_HandleTypeDef huart2;

/* USER CODE BEGIN Private defines */

#define USART_BUFFER_SIZE 11

/* USER CODE END Private defines */

void MX_USART2_UART_Init(void);

/* USER CODE BEGIN Prototypes */

typedef struct { // __attribute__(packed)
	uint8_t delimiter_l;
	uint8_t delimiter_h;
	uint8_t cmd;
	int8_t m1_speed;
	uint8_t m1_steps;
	int8_t m2_speed;
	uint8_t m2_steps;
	int8_t m3_speed;
	uint8_t m3_steps;
	int8_t m4_speed;
	uint8_t m4_steps;
	//uint16_t crc;
}usart_buffer_st;

bool USART_drdy();
void USART_init();
bool USART_drdy();
usart_buffer_st* USART_get_buffer();


/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

