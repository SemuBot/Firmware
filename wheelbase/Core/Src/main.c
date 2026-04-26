/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - Switched to MOTOR 2
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "drv8353.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <sys/time.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern motor_st motor1, motor2, motor3;
DRV8353_Handle drv_motor1, drv_motor2, drv_motor3;
motor_st motor1 = {
    .dir_port = motor1_dir_GPIO_Port,
    .dir_pin = motor1_dir_Pin,
    .pwm_timer = &htim1,
    .pwm_port = motor1_pwm_GPIO_Port,
    .pwm_pin = motor1_pwm_Pin,
    .duty_cycle = 0.0,
    .duty_cycle_limit = DUTY_CYCLE_LIMIT,
    .drv_handle = &drv_motor1
};

motor_st motor2 = {
    .dir_port = motor2_dir_GPIO_Port,
    .dir_pin = motor2_dir_Pin,
    .pwm_timer = &htim3,
    .pwm_port = motor2_pwm_GPIO_Port,
    .pwm_pin = motor2_pwm_Pin,
    .duty_cycle = 0.0,
    .duty_cycle_limit = DUTY_CYCLE_LIMIT,
    .drv_handle = &drv_motor2
};

motor_st motor3 = {
    .dir_port = motor3_dir_GPIO_Port,
    .dir_pin = motor3_dir_Pin,
    .pwm_timer = &htim4,
    .pwm_port = motor3_pwm_GPIO_Port,
    .pwm_pin = motor3_pwm_Pin,
    .duty_cycle = 0.0,
    .duty_cycle_limit = DUTY_CYCLE_LIMIT,
    .drv_handle = &drv_motor3
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_CAN_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
  MX_USB_DEVICE_Init();

  HAL_Delay(5000);

  // Motor 1
  drv_motor1.hspi = &hspi2;
  drv_motor1.cs_port = motor1_Cs_GPIO_Port;
  drv_motor1.cs_pin = motor1_Cs_Pin;
  drv_motor1.initialized = false;

  // Motor 2
  drv_motor2.hspi = &hspi2;
  drv_motor2.cs_port = motor2_Cs_GPIO_Port;
  drv_motor2.cs_pin = motor2_Cs_Pin;
  drv_motor2.initialized = false;

  // Motor 3
  drv_motor3.hspi = &hspi2;
  drv_motor3.cs_port = motor3_Cs_GPIO_Port;
  drv_motor3.cs_pin = motor3_Cs_Pin;
  drv_motor3.initialized = false;

  // Deselect all CS pins
  HAL_GPIO_WritePin(motor1_Cs_GPIO_Port, motor1_Cs_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(motor2_Cs_GPIO_Port, motor2_Cs_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(motor3_Cs_GPIO_Port, motor3_Cs_Pin, GPIO_PIN_SET);
  HAL_Delay(100);


  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

  // Motor 1 (TIM1 CH1)
  HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  __HAL_TIM_MOE_ENABLE(&htim1);

  // Motor 2 (TIM3 CH1)
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);

  // Motor 3 (TIM4 CH1)
  HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1);
  HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);

  HAL_GPIO_WritePin(enable_GPIO_Port, enable_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(motor1_nBRAKE_GPIO_Port, motor1_nBRAKE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(motor2_nBRAKE_GPIO_Port, motor2_nBRAKE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(motor3_nBRAKE_GPIO_Port, motor3_nBRAKE_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);


  HAL_GPIO_WritePin(enable_GPIO_Port, enable_Pin, GPIO_PIN_SET);
  HAL_Delay(100);

  // Test SPI

  DRV8353_RawSPITest(&drv_motor1);
  DRV8353_RawSPITest(&drv_motor2);
  DRV8353_RawSPITest(&drv_motor3);


  uint16_t driver_ctrl = 0x0040;
  uint16_t gate_hs_ctrl = 0x0300;
  uint16_t gate_ls_ctrl = 0x0300;
  uint16_t ocp_ctrl = 0x0129;
  uint16_t csa_ctrl = 0x0123;

  // M1
  DRV8353_WriteRegister(&drv_motor1, DRV8353_REG_DRIVER_CONTROL, driver_ctrl);
  DRV8353_WriteRegister(&drv_motor1, DRV8353_REG_GATE_DRIVE_HS, gate_hs_ctrl);
  DRV8353_WriteRegister(&drv_motor1, DRV8353_REG_GATE_DRIVE_LS, gate_ls_ctrl);
  DRV8353_WriteRegister(&drv_motor1, DRV8353_REG_OCP_CONTROL, ocp_ctrl);
  DRV8353_WriteRegister(&drv_motor1, DRV8353_REG_CSA_CONTROL, csa_ctrl);

  // M2
  DRV8353_WriteRegister(&drv_motor2, DRV8353_REG_DRIVER_CONTROL, driver_ctrl);
  DRV8353_WriteRegister(&drv_motor2, DRV8353_REG_GATE_DRIVE_HS, gate_hs_ctrl);
  DRV8353_WriteRegister(&drv_motor2, DRV8353_REG_GATE_DRIVE_LS, gate_ls_ctrl);
  DRV8353_WriteRegister(&drv_motor2, DRV8353_REG_OCP_CONTROL, ocp_ctrl);
  DRV8353_WriteRegister(&drv_motor2, DRV8353_REG_CSA_CONTROL, csa_ctrl);

  // M3
  DRV8353_WriteRegister(&drv_motor3, DRV8353_REG_DRIVER_CONTROL, driver_ctrl);
  DRV8353_WriteRegister(&drv_motor3, DRV8353_REG_GATE_DRIVE_HS, gate_hs_ctrl);
  DRV8353_WriteRegister(&drv_motor3, DRV8353_REG_GATE_DRIVE_LS, gate_ls_ctrl);
  DRV8353_WriteRegister(&drv_motor3, DRV8353_REG_OCP_CONTROL, ocp_ctrl);
  DRV8353_WriteRegister(&drv_motor3, DRV8353_REG_CSA_CONTROL, csa_ctrl);



  HAL_GPIO_WritePin(motor1_nBRAKE_GPIO_Port, motor1_nBRAKE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(motor2_nBRAKE_GPIO_Port, motor2_nBRAKE_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(motor3_nBRAKE_GPIO_Port, motor3_nBRAKE_Pin, GPIO_PIN_SET);
  HAL_Delay(100);

  // Calibration
  DRV8353_CalibrateCSAs(&drv_motor1);
  DRV8353_CalibrateCSAs(&drv_motor2);
  DRV8353_CalibrateCSAs(&drv_motor3);


  HAL_Delay(10);
  HAL_GPIO_WritePin(motor1.dir_port, motor1.dir_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(motor2.dir_port, motor2.dir_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(motor3.dir_port, motor3.dir_pin, GPIO_PIN_SET);
  HAL_Delay(10);

  /* USER CODE END 2 */


  MX_FREERTOS_Init();

  osKernelStart();



  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV2;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_TIM34;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim34ClockSelection = RCC_TIM34CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */


/* Function for microros, so it stops giving me compilation errors*/
int _gettimeofday(struct timeval *tv, void *tz) {
    (void)tz;
    uint32_t ms = HAL_GetTick();
    tv->tv_sec  = ms / 1000;
    tv->tv_usec = (ms % 1000) * 1000;
    return 0;
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
#ifdef USE_FULL_ASSERT
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
