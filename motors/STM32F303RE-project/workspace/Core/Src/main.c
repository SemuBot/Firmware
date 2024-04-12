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
#define MAX_COMMAND_LENGTH 20

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



struct Motors motor1 = {
    .DIR_PIN = MOTOR1_DIR_Pin,
    .STEP_PIN = MOTOR1_PUL_Pin,
    .EN_PIN = MOTOR1_EN_Pin,
    .SPEED = 100,
    .STEPS = 200,
    .TIMER = TIM1,
    .EN_PORT = MOTOR1_EN_GPIO_Port,
    .DIR_PORT = MOTOR1_DIR_GPIO_Port,
    .moving = false
};

struct Motors motor2 = {
    .DIR_PIN = MOTOR2_DIR_Pin,
    .STEP_PIN = MOTOR2_PUL_Pin,
    .EN_PIN = MOTOR2_EN_Pin,
    .SPEED = 100,
    .STEPS = 200,
    .TIMER = TIM2,
    .EN_PORT = MOTOR2_EN_GPIO_Port,
    .DIR_PORT = MOTOR2_DIR_GPIO_Port,
    .moving = false
};

struct Motors motor3 = {
    .DIR_PIN = MOTOR3_DIR_Pin,
    .STEP_PIN = MOTOR3_PUL_Pin,
    .EN_PIN = MOTOR3_EN_Pin,
    .SPEED = 100,
    .STEPS = 200,
    .TIMER = TIM3,
    .EN_PORT = MOTOR3_EN_GPIO_Port,
    .DIR_PORT = MOTOR3_DIR_GPIO_Port,
    .moving = false
};

struct Motors motor4 = {
    .DIR_PIN = MOTOR4_DIR_Pin,
    .STEP_PIN = MOTOR4_PUL_Pin,
    .EN_PIN = MOTOR4_EN_Pin,
    .SPEED = 100,
    .STEPS = 200,
    .TIMER = TIM4,
    .EN_PORT = MOTOR4_EN_GPIO_Port,
    .DIR_PORT = MOTOR4_DIR_GPIO_Port,
    .moving = false
};

struct Motors motor5 = {
    .DIR_PIN = MOTOR5_DIR_Pin,
    .STEP_PIN = MOTOR5_PUL_Pin,
    .EN_PIN = MOTOR5_EN_Pin,
    .SPEED = 100,
    .STEPS = 200,
    .TIMER = TIM8,
    .EN_PORT = MOTOR5_EN_GPIO_Port,
    .DIR_PORT = MOTOR5_DIR_GPIO_Port,
    .moving = false
};

int counter = 0;
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


	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_GPIO_WritePin(motor1.DIR_PORT, motor1.DIR_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(motor2.DIR_PORT, motor2.DIR_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor3.DIR_PORT, motor3.DIR_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor4.DIR_PORT, motor4.DIR_PIN, GPIO_PIN_SET);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	motor1.STEPS = 2500;
	motor1.SPEED = 5;

	motor2.STEPS = 1500;
	motor2.SPEED = 5;

	motor3.STEPS = 1500;
	motor3.SPEED = 5;

	motor4.STEPS = 1500;
	motor4.SPEED = 60;


	moveMotor(&motor1);
	//moveMotor(&motor2);
	moveMotor(&motor3);
	moveMotor(&motor4);


	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);

	HAL_GPIO_WritePin(motor1.DIR_PORT, motor1.DIR_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor2.DIR_PORT, motor2.DIR_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(motor3.DIR_PORT, motor3.DIR_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(motor4.DIR_PORT, motor4.DIR_PIN, GPIO_PIN_RESET);

	motor1.STEPS = 2500;
	motor1.SPEED = 10;

	motor2.STEPS = 1500;
	motor2.SPEED = 10;

	motor3.STEPS = 1500;
	motor3.SPEED = 10;

	motor4.STEPS = 1500;
	motor4.SPEED = 10;


	moveMotor(&motor4);
	moveMotor(&motor3);
	//moveMotor(&motor2);
	moveMotor(&motor1);
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

    if (huart == &huart2) {
        if (UART2_rxBuffer[0] != '\0') {
            char *token = strtok((char *)UART2_rxBuffer, "_");

            // Check the command for motor 1
            if (strcmp(token, MOTOR1_COMMAND) == 0) {
                token = strtok(NULL, "_"); // Get the next token (direction)
                int direction = (token != NULL && strcmp(token, "cw") == 0) ? 1 : 0; // 1 for clockwise, 0 for counterclockwise

                token = strtok(NULL, "_"); // Get the next token (speed)
                if (token != NULL) {
                    int speed = atoi(token); // Convert speed string to integer
                    motor1.SPEED = speed;
                }

                token = strtok(NULL, "_"); // Get the next token (steps)
                if (token != NULL) {
                    int steps = atoi(token); // Convert steps string to integer
                    motor1.STEPS = steps;
                }

                // Set direction pin based on direction
                HAL_GPIO_WritePin(motor1.DIR_PORT, motor1.DIR_PIN, direction);

                // Send back the updated settings over UART
                char uartTxBuffer[MAX_COMMAND_LENGTH] = {0};
                sprintf(uartTxBuffer, "Motor1 Settings: Direction=%s, Speed=%d, Steps=%d\r\n",
                        (direction == 1) ? "CW" : "CCW", motor1.SPEED, motor1.STEPS);
                HAL_UART_Transmit(&huart2, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), HAL_MAX_DELAY);
            }

            // Check the command for motor 2
            else if (strcmp(token, MOTOR2_COMMAND) == 0) {
                token = strtok(NULL, "_"); // Get the next token (direction)
                int direction = (token != NULL && strcmp(token, "cw") == 0) ? 1 : 0; // 1 for clockwise, 0 for counterclockwise

                token = strtok(NULL, "_"); // Get the next token (speed)
                if (token != NULL) {
                    int speed = atoi(token); // Convert speed string to integer
                    motor2.SPEED = speed;
                }

                token = strtok(NULL, "_"); // Get the next token (steps)
                if (token != NULL) {
                    int steps = atoi(token); // Convert steps string to integer
                    motor2.STEPS = steps;
                }

                // Set direction pin based on direction
                HAL_GPIO_WritePin(motor2.DIR_PORT, motor2.DIR_PIN, direction);

                // Send back the updated settings over UART
                char uartTxBuffer[MAX_COMMAND_LENGTH] = {0};
                sprintf(uartTxBuffer, "Motor2 Settings: Direction=%s, Speed=%d, Steps=%d\r\n",
                        (direction == 1) ? "CW" : "CCW", motor2.SPEED, motor2.STEPS);
                HAL_UART_Transmit(&huart2, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), HAL_MAX_DELAY);
            }

            // Check the command for motor 3
            else if (strcmp(token, MOTOR3_COMMAND) == 0) {
                token = strtok(NULL, "_"); // Get the next token (direction)
                int direction = (token != NULL && strcmp(token, "cw") == 0) ? 1 : 0; // 1 for clockwise, 0 for counterclockwise

                token = strtok(NULL, "_"); // Get the next token (speed)
                if (token != NULL) {
                    int speed = atoi(token); // Convert speed string to integer
                    motor3.SPEED = speed;
                }

                token = strtok(NULL, "_"); // Get the next token (steps)
                if (token != NULL) {
                    int steps = atoi(token); // Convert steps string to integer
                    motor3.STEPS = steps;
                }

                // Set direction pin based on direction
                HAL_GPIO_WritePin(motor3.DIR_PORT, motor3.DIR_PIN, direction);

                // Send back the updated settings over UART
                char uartTxBuffer[MAX_COMMAND_LENGTH] = {0};
                sprintf(uartTxBuffer, "Motor3 Settings: Direction=%s, Speed=%d, Steps=%d\r\n",
                        (direction == 1) ? "CW" : "CCW", motor3.SPEED, motor3.STEPS);
                HAL_UART_Transmit(&huart2, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), HAL_MAX_DELAY);
            }

            // Check the command for motor 4
            else if (strcmp(token, MOTOR4_COMMAND) == 0) {
                token = strtok(NULL, "_"); // Get the next token (direction)
                int direction = (token != NULL && strcmp(token, "cw") == 0) ? 1 : 0; // 1 for clockwise, 0 for counterclockwise

                token = strtok(NULL, "_"); // Get the next token (speed)
                if (token != NULL) {
                    int speed = atoi(token); // Convert speed string to integer
                    motor4.SPEED = speed;
                }

                token = strtok(NULL, "_"); // Get the next token (steps)
                if (token != NULL) {
                    int steps = atoi(token); // Convert steps string to integer
                    motor4.STEPS = steps;
                }

                // Set direction pin based on direction
                HAL_GPIO_WritePin(motor4.DIR_PORT, motor4.DIR_PIN, direction);

                // Send back the updated settings over UART
                char uartTxBuffer[MAX_COMMAND_LENGTH] = {0};
                sprintf(uartTxBuffer, "Motor4 Settings: Direction=%s, Speed=%d, Steps=%d\r\n",
                        (direction == 1) ? "CW" : "CCW", motor4.SPEED, motor4.STEPS);
                HAL_UART_Transmit(&huart2, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), HAL_MAX_DELAY);
            }

            // Check the command for motor 5
            else if (strcmp(token, MOTOR5_COMMAND) == 0) {
                token = strtok(NULL, "_"); // Get the next token (direction)
                int direction = (token != NULL && strcmp(token, "cw") == 0) ? 1 : 0; // 1 for clockwise, 0 for counterclockwise

                token = strtok(NULL, "_"); // Get the next token (speed)
                if (token != NULL) {
                    int speed = atoi(token); // Convert speed string to integer
                    motor5.SPEED = speed;
                }

                token = strtok(NULL, "_"); // Get the next token (steps)
                if (token != NULL) {
                    int steps = atoi(token); // Convert steps string to integer
                    motor5.STEPS = steps;
                }

                // Set direction pin based on direction
                HAL_GPIO_WritePin(motor5.DIR_PORT, motor5.DIR_PIN, direction);

                // Send back the updated settings over UART
                char uartTxBuffer[MAX_COMMAND_LENGTH] = {0};
                sprintf(uartTxBuffer, "Motor5 Settings: Direction=%s, Speed=%d, Steps=%d\r\n",
                        (direction == 1) ? "CW" : "CCW", motor5.SPEED, motor5.STEPS);
                HAL_UART_Transmit(&huart2, (uint8_t *)uartTxBuffer, strlen(uartTxBuffer), HAL_MAX_DELAY);
            }
        }

        // Clear the receive buffer
        memset(UART2_rxBuffer, 0, sizeof(UART2_rxBuffer));

        // Restart UART receive interrupt
        HAL_UART_Receive_IT(&huart2, UART2_rxBuffer, MAX_COMMAND_LENGTH);
    }
}





void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  // Check if the button (PC13) is pressed
  if (GPIO_Pin == GPIO_PIN_13)
  {

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
