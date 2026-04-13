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
#define debug_Pin GPIO_PIN_13
#define debug_GPIO_Port GPIOC
#define motor2_DRDY_Pin GPIO_PIN_0
#define motor2_DRDY_GPIO_Port GPIOA
#define cs_enc_2_Pin GPIO_PIN_1
#define cs_enc_2_GPIO_Port GPIOA
#define motor2_adc_Pin GPIO_PIN_2
#define motor2_adc_GPIO_Port GPIOA
#define motor2_nBRAKE_Pin GPIO_PIN_3
#define motor2_nBRAKE_GPIO_Port GPIOA
#define motor2_Cs_Pin GPIO_PIN_4
#define motor2_Cs_GPIO_Port GPIOA
#define motor2_dir_Pin GPIO_PIN_5
#define motor2_dir_GPIO_Port GPIOC
#define motor2_pwm_Pin GPIO_PIN_6
#define motor2_pwm_GPIO_Port GPIOC
#define motor3_nBRAKE_Pin GPIO_PIN_0
#define motor3_nBRAKE_GPIO_Port GPIOB
#define motor3_dir_Pin GPIO_PIN_4
#define motor3_dir_GPIO_Port GPIOC
#define motor3_Cs_Pin GPIO_PIN_1
#define motor3_Cs_GPIO_Port GPIOB
#define motor3_adc_Pin GPIO_PIN_2
#define motor3_adc_GPIO_Port GPIOB
#define motor3_DRDY_Pin GPIO_PIN_10
#define motor3_DRDY_GPIO_Port GPIOB
#define enable_Pin GPIO_PIN_11
#define enable_GPIO_Port GPIOB
#define motor1_nBRAKE_Pin GPIO_PIN_12
#define motor1_nBRAKE_GPIO_Port GPIOB
#define motor1_dir_Pin GPIO_PIN_6
#define motor1_dir_GPIO_Port GPIOC
#define cs_enc_3_Pin GPIO_PIN_7
#define cs_enc_3_GPIO_Port GPIOC
#define cs_enc_1_Pin GPIO_PIN_8
#define cs_enc_1_GPIO_Port GPIOC
#define motor1_DRDY_Pin GPIO_PIN_9
#define motor1_DRDY_GPIO_Port GPIOC
#define motor1_pwm_Pin GPIO_PIN_8
#define motor1_pwm_GPIO_Port GPIOA
#define motor1_Cs_Pin GPIO_PIN_9
#define motor1_Cs_GPIO_Port GPIOA
#define motor1_adc_Pin GPIO_PIN_10
#define motor1_adc_GPIO_Port GPIOA
#define MCU_CS_Pin GPIO_PIN_15
#define MCU_CS_GPIO_Port GPIOA
#define motor2_led_Pin GPIO_PIN_3
#define motor2_led_GPIO_Port GPIOB
#define motor3_led_Pin GPIO_PIN_4
#define motor3_led_GPIO_Port GPIOB
#define motor1_led_Pin GPIO_PIN_5
#define motor1_led_GPIO_Port GPIOB
#define motor3_pwm_Pin GPIO_PIN_6
#define motor3_pwm_GPIO_Port GPIOB
#define SSI_CLK_Pin GPIO_PIN_2
#define SSI_CLK_GPIO_Port GPIOD
#define SSI_DATA_Pin GPIO_PIN_3
#define SSI_DATA_GPIO_Port GPIOB
#define enc1_data_Pin        GPIO_PIN_3
#define enc1_data_GPIO_Port  GPIOB
#define enc2_data_Pin        GPIO_PIN_15
#define enc2_data_GPIO_Port  GPIOC
#define enc3_data_Pin        GPIO_PIN_5
#define enc3_data_GPIO_Port  GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
