/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

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
#define DEBUG_LED_Pin GPIO_PIN_13
#define DEBUG_LED_GPIO_Port GPIOC
#define MOTOR2_DIR_Pin GPIO_PIN_5
#define MOTOR2_DIR_GPIO_Port GPIOA
#define MOTOR3_DIR_Pin GPIO_PIN_2
#define MOTOR3_DIR_GPIO_Port GPIOB
#define nBRAKE_Pin GPIO_PIN_10
#define nBRAKE_GPIO_Port GPIOB
#define MOTOR1_DIR_Pin GPIO_PIN_11
#define MOTOR1_DIR_GPIO_Port GPIOB
#define ENABLE_Pin GPIO_PIN_12
#define ENABLE_GPIO_Port GPIOB
#define CS_ENC_1_Pin GPIO_PIN_15
#define CS_ENC_1_GPIO_Port GPIOB
#define CS_ENC_3_Pin GPIO_PIN_6
#define CS_ENC_3_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
