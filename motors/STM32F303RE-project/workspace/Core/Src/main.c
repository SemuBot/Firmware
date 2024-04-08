/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motors.h"
#include "variables.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#define MAX_COMMAND_LENGTH 7

#define MOTOR1_COMMAND "m1"
#define MOTOR2_COMMAND "m2"
#define MOTOR3_COMMAND "m3"
#define MOTOR4_COMMAND "m4"
#define MOTOR5_COMMAND "m5"

uint8_t UART2_rxBuffer[MAX_COMMAND_LENGTH] = {0};
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */





Motors motor1;
Motors motor2;
Motors motor3;
Motors motor4;
volatile Motors motor5;
int counter = 0;

void move_motor(Motors* m){
	if (m->STEPS == 0) return;

	m->SPEED_CNT++;

	if (m->SPEED_CNT == 0){
		HAL_GPIO_TogglePin(m->PWM_PORT, m->PWM_PIN);
	}

	if (m->SPEED_CNT == (m->SPEED >> 1)){
		HAL_GPIO_TogglePin(m->PWM_PORT, m->PWM_PIN);
	}
	if (m->SPEED_CNT == m->SPEED){

		m->STEPS--;
		m->SPEED_CNT = 0;

	}


	// Kas peab liikuma steps != 0

	// Ajutine muutuja, mis määrab kiirust. kiirus > speed, siis toggle
	// Kui on 2 korda toggle toimunud, siis steps--


	}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

  move_motor(&motor1);
  move_motor(&motor2);
  move_motor(&motor3);
  move_motor(&motor4);
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

  // Move motor logic
  // If steps not zero, toggle pin
  // decrement steps value
  // if reaches zero, dont move
  // Speed determines when to toggle step pin


}



#define DEFAULT_STEPS_NR 500

void add_motor_val(Motors* m, int8_t val){
	if (val < 0){
		m->DIR_PIN = 0;
		HAL_GPIO_WritePin(m->DIR_PORT, m->DIR_PIN, GPIO_PIN_RESET); // Set EN high to enable the driver
		m->SPEED = -val;
	} else {
		m->DIR_PIN = 1;
		HAL_GPIO_WritePin(m->DIR_PORT, m->DIR_PIN, GPIO_PIN_SET); // Set EN high to enable the driver
		m->SPEED = val;

	}
	m->SPEED_CNT = 0;
	m->STEPS = 200;
}

void receive(){
	uint16_t packet_start = UART2_rxBuffer[0] << 8 || UART2_rxBuffer[1];
	if (packet_start != 0x4994) return;

	// Do cmd byte check

	int8_t m1_val = UART2_rxBuffer[3];
	int8_t m2_val = UART2_rxBuffer[4];
	int8_t m3_val = UART2_rxBuffer[5];
	int8_t m4_val = UART2_rxBuffer[6];

	if (m1_val != 0) add_motor_val(&motor1, m1_val);
	if (m2_val != 0) add_motor_val(&motor2, m2_val);
	if (m3_val != 0) add_motor_val(&motor3, m3_val);
	if (m4_val != 0) add_motor_val(&motor4, m4_val);

	HAL_UART_Receive_IT(&huart2, UART2_rxBuffer, MAX_COMMAND_LENGTH);


}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); // Blue button
  HAL_GPIO_EXTI_Callback(GPIO_PIN_13);

  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); //Start timer
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //Start timer
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1); //Start timer
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); //Start timer
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1); //Start timer
  HAL_UART_Receive_IT(&huart2, UART2_rxBuffer, MAX_COMMAND_LENGTH);
  //HAL_UART_Receive_IT(&huart1, UART2_rxBuffer, MAX_COMMAND_LENGTH);


  // Motor 1 initialization
  motor1.DIR_PIN = MOTOR1_DIR_Pin;
  motor1.STEP_PIN = MOTOR1_PUL_Pin;
  motor1.EN_PIN = MOTOR1_EN_Pin;
  motor1.SPEED = 100;
  motor1.SPEED_CNT = 0;
  motor1.STEPS = 200;
  motor1.TIMER = TIM1;
  motor1.EN_PORT = MOTOR1_EN_GPIO_Port;
  motor1.DIR_PORT = MOTOR1_DIR_GPIO_Port;
  motor1.PWM_PORT = MOTOR1_PUL_GPIO_Port;
  motor1.is_high = false;

  // Motor 2 initialization
  motor2.DIR_PIN = MOTOR2_DIR_Pin;
  motor2.STEP_PIN = MOTOR2_PUL_Pin;
  motor2.EN_PIN = MOTOR2_EN_Pin;
  motor2.SPEED = 100;
  motor2.SPEED_CNT = 0;

  motor2.STEPS = 200;
  motor2.TIMER = TIM2;
  motor2.EN_PORT = MOTOR2_EN_GPIO_Port;
  motor2.DIR_PORT = MOTOR2_DIR_GPIO_Port;
  motor2.PWM_PORT = MOTOR2_PUL_GPIO_Port;
  motor2.is_high = false;


  // Motor 3 initialization
  motor3.DIR_PIN = MOTOR3_DIR_Pin;
  motor3.STEP_PIN = MOTOR3_PUL_Pin;
  motor3.EN_PIN = MOTOR3_EN_Pin;
  motor3.SPEED = 100;
  motor3.SPEED_CNT = 0;
  motor3.STEPS = 200;
  motor3.TIMER = TIM3;
  motor3.EN_PORT = MOTOR3_EN_GPIO_Port;
  motor3.DIR_PORT = MOTOR3_DIR_GPIO_Port;
  motor3.PWM_PORT = MOTOR3_PUL_GPIO_Port;
  motor3.is_high = false;


  // Motor 4 initialization
  motor4.DIR_PIN = MOTOR4_DIR_Pin;
  motor4.STEP_PIN = MOTOR4_PUL_Pin;
  motor4.EN_PIN = MOTOR4_EN_Pin;
  motor4.SPEED = 100;
  motor4.SPEED_CNT = 0;

  motor4.STEPS = 200;
  motor4.TIMER = TIM4;
  motor4.EN_PORT = MOTOR4_EN_GPIO_Port;
  motor4.DIR_PORT = MOTOR4_DIR_GPIO_Port;

  motor4.PWM_PORT = MOTOR4_PUL_GPIO_Port;
  motor4.is_high = false;


  // Motor 5 initialization
  motor5.DIR_PIN = MOTOR5_DIR_Pin;
  motor5.STEP_PIN = MOTOR5_PUL_Pin;
  motor5.EN_PIN = MOTOR5_EN_Pin;
  motor5.PWM_PIN = MOTOR5_PUL_Pin;
  motor5.SPEED = 100;
  motor5.SPEED_CNT = 0;
  motor5.STEPS = 200;
  motor5.TIMER = TIM8;
  motor5.EN_PORT = MOTOR5_EN_GPIO_Port;
  motor5.DIR_PORT = MOTOR5_DIR_GPIO_Port;
  motor5.PWM_PORT = MOTOR5_PUL_GPIO_Port;
  motor5.is_high = false;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_TIM8
                              |RCC_PERIPHCLK_TIM2|RCC_PERIPHCLK_TIM34;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim8ClockSelection = RCC_TIM8CLK_HCLK;
  PeriphClkInit.Tim2ClockSelection = RCC_TIM2CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
    receive();

}





void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // Check if the button (PC13) is pressed
  if (GPIO_Pin == GPIO_PIN_13)
  {
	    //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
	    motor1.STEPS = 200;
	    motor1.SPEED = 50;
	    motor2.STEPS = 200;
	    motor2.SPEED = 50;
	    counter++;
	    if (counter > 2){
	    	HAL_GPIO_TogglePin(motor1.DIR_PORT, motor1.DIR_PIN); // Toggle the direction pin
	    	HAL_GPIO_TogglePin(motor2.DIR_PORT, motor2.DIR_PIN); // Toggle the direction pin

	    }
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
