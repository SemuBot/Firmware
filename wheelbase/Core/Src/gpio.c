/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    DBGMCU->CR &= ~DBGMCU_CR_TRACE_IOEN;
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    // SSI clock and data
    HAL_GPIO_WritePin(SSI_CLK_GPIO_Port, SSI_CLK_Pin, GPIO_PIN_SET);
    GPIO_InitStruct.Pin   = SSI_CLK_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(SSI_CLK_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin  = SSI_DATA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(SSI_DATA_GPIO_Port, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOA, motor2_nBRAKE_Pin|motor2_Cs_Pin|motor1_Cs_Pin|MCU_CS_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOB, motor3_Cs_Pin|enable_Pin|motor1_nBRAKE_Pin|motor3_nBRAKE_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(GPIOC, debug_Pin|motor3_dir_Pin|motor1_dir_Pin|motor2_dir_Pin, GPIO_PIN_RESET);

    HAL_GPIO_WritePin(motor1_adc_GPIO_Port, motor1_adc_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor2_adc_GPIO_Port, motor2_adc_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor3_adc_GPIO_Port, motor3_adc_Pin, GPIO_PIN_SET);

    GPIO_InitStruct.Pin   = motor2_nBRAKE_Pin|motor2_Cs_Pin|motor1_Cs_Pin|MCU_CS_Pin|motor1_adc_Pin| motor2_adc_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = motor3_Cs_Pin|enable_Pin|motor1_nBRAKE_Pin|motor3_nBRAKE_Pin|motor3_adc_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(motor3_adc_GPIO_Port, motor3_adc_Pin, GPIO_PIN_SET);

    GPIO_InitStruct.Pin   = debug_Pin|motor3_dir_Pin|motor1_dir_Pin|motor2_dir_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = motor2_pwm_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(motor2_pwm_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Pin  = motor2_DRDY_Pin;
    HAL_GPIO_Init(motor2_DRDY_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin  = motor3_DRDY_Pin;
    HAL_GPIO_Init(motor3_DRDY_GPIO_Port, &GPIO_InitStruct);
    GPIO_InitStruct.Pin  = motor1_DRDY_Pin;
    HAL_GPIO_Init(motor1_DRDY_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin   = cs_enc_3_Pin;
    HAL_GPIO_Init(cs_enc_3_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(cs_enc_3_GPIO_Port, cs_enc_3_Pin, GPIO_PIN_SET);
    GPIO_InitStruct.Pin   = cs_enc_1_Pin;
    HAL_GPIO_Init(cs_enc_1_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(cs_enc_1_GPIO_Port, cs_enc_1_Pin, GPIO_PIN_SET);
    GPIO_InitStruct.Pin   = cs_enc_2_Pin;
    HAL_GPIO_Init(cs_enc_2_GPIO_Port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(cs_enc_2_GPIO_Port, cs_enc_2_Pin, GPIO_PIN_SET);

    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Pin  = enc1_data_Pin|enc3_data_Pin;   // PB3, PB5
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    GPIO_InitStruct.Pin  = enc2_data_Pin;                  // PC15
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);



    GPIO_InitStruct.Pin       = motor1_pwm_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
    HAL_GPIO_Init(motor1_pwm_GPIO_Port, &GPIO_InitStruct);

    // Motor 3 PWM: PB6 = TIM4_CH1
    GPIO_InitStruct.Pin       = motor3_pwm_Pin;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(motor3_pwm_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
