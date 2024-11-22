/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define DIR_Pin GPIO_PIN_9
#define DIR_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define MOTOR1_PUL_Pin GPIO_PIN_8
#define MOTOR1_PUL_GPIO_Port GPIOA
#define MOTOR1_DIR_Pin GPIO_PIN_9
#define MOTOR1_DIR_GPIO_Port GPIOC
#define MOTOR1_EN_Pin GPIO_PIN_2
#define MOTOR1_EN_GPIO_Port GPIOC
#define MOTOR2_PUL_Pin GPIO_PIN_0
#define MOTOR2_PUL_GPIO_Port GPIOA
#define MOTOR2_EN_Pin GPIO_PIN_1
#define MOTOR2_EN_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define MOTOR2_DIR_Pin GPIO_PIN_4
#define MOTOR2_DIR_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define MOTOR3_PUL_Pin GPIO_PIN_6
#define MOTOR3_PUL_GPIO_Port GPIOA
#define MOTOR3_EN_Pin GPIO_PIN_7
#define MOTOR3_EN_GPIO_Port GPIOA
#define CS_ENC_4_Pin GPIO_PIN_10
#define CS_ENC_4_GPIO_Port GPIOB
#define MOTOR4_EN_Pin GPIO_PIN_12
#define MOTOR4_EN_GPIO_Port GPIOB
#define MOTOR5_DIR_Pin GPIO_PIN_14
#define MOTOR5_DIR_GPIO_Port GPIOB
#define MOTOR5_EN_Pin GPIO_PIN_15
#define MOTOR5_EN_GPIO_Port GPIOB
#define MOTOR5_PUL_Pin GPIO_PIN_6
#define MOTOR5_PUL_GPIO_Port GPIOC
#define CS_ENC_2_Pin GPIO_PIN_7
#define CS_ENC_2_GPIO_Port GPIOC
#define CS_ENC_3_Pin GPIO_PIN_1
#define CS_ENC_3_GPIO_Port GPIOA
#define CS_ENC_1_Pin GPIO_PIN_9
#define CS_ENC_1_GPIO_Port GPIOA
#define MOTOR4_PUL_Pin GPIO_PIN_11
#define MOTOR4_PUL_GPIO_Port GPIOA
#define MOTOR4_DIR_Pin GPIO_PIN_12
#define MOTOR4_DIR_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define CS_ENC_5_Pin GPIO_PIN_4
#define CS_ENC_5_GPIO_Port GPIOB
#define MOTOR3_DIR_Pin GPIO_PIN_6
#define MOTOR3_DIR_GPIO_Port GPIOB
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
