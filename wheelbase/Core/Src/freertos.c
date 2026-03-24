/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#include "motor.h"
#include "drv8353.h"
#include "tim.h"
#include "spi.h"
#include "usbd_cdc_if.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* USER CODE BEGIN PD */
#define CONTROL_LOOP_PERIOD_MS  5
#define STATE_REPORT_PERIOD_MS  10

char debug_msg[256];
extern motor_st motor1, motor2, motor3;
extern DRV8353_Handle drv_motor1, drv_motor2, drv_motor3;
/* USER CODE END PD */

/* USER CODE BEGIN Variables */

volatile float motor_duty[3] = {0.0f, 0.0f, 0.0f};
volatile uint32_t cmd_last_tick = 0;
#define CMD_TIMEOUT_MS 500

// Encoder state
static uint16_t enc_raw[3]  = {0};
static float    enc_deg[3]  = {0};
static float    enc_vel[3]  = {0};
static uint16_t enc_prev[3] = {0};

// CDC receive buffer
#define CDC_RX_BUF_SIZE 128
static char cdc_rx_buf[CDC_RX_BUF_SIZE];
static volatile uint16_t cdc_rx_len = 0;
static volatile uint8_t  cdc_rx_ready = 0;

/* USER CODE END Variables */

osThreadId defaultTaskHandle;
osThreadId controlTaskHandle;

void StartDefaultTask(void const * argument);
void StartControlTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void);

static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer   = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize   = configMINIMAL_STACK_SIZE;
}


void CDC_UserRxCallback(uint8_t *buf, uint32_t len)
{
    if (len < CDC_RX_BUF_SIZE && !cdc_rx_ready) {
        memcpy(cdc_rx_buf, buf, len);
        cdc_rx_buf[len] = '\0';
        cdc_rx_len   = len;
        cdc_rx_ready = 1;
    }
}


static void Parse_CDC_Command(const char *str)
{
    if (strncmp(str, "CMD:", 4) != 0) return;

    float v1, v2, v3;
    if (sscanf(str + 4, "%f,%f,%f", &v1, &v2, &v3) == 3) {
        motor_duty[0] = v1;
        motor_duty[1] = v2;
        motor_duty[2] = v3;
        cmd_last_tick = HAL_GetTick();
    }
}


static void Motor_SetDuty(TIM_HandleTypeDef *htim, uint32_t channel,
                          GPIO_TypeDef *dir_port, uint16_t dir_pin,
                          GPIO_TypeDef *brake_port, uint16_t brake_pin,
                          float duty)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);

    if (fabsf(duty) < 0.01f) {
        HAL_GPIO_WritePin(brake_port, brake_pin, GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(htim, channel, 0);
        return;
    }

    HAL_GPIO_WritePin(brake_port, brake_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(dir_port, dir_pin,
        duty >= 0.0f ? GPIO_PIN_SET : GPIO_PIN_RESET);

    float clamped = fminf(fabsf(duty), 1.0f);
    __HAL_TIM_SET_COMPARE(htim, channel, (uint32_t)(clamped * arr));
}


uint16_t Read_SSI(GPIO_TypeDef *cs_port, uint16_t cs_pin)
{
    uint16_t raw = 0;

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    for (volatile int i = 0; i < 36; i++) { __NOP(); }

    for (int i = 16; i >= 0; i--) {
        HAL_GPIO_WritePin(SSI_CLK_GPIO_Port, SSI_CLK_Pin, GPIO_PIN_RESET);
        for (volatile int j = 0; j < 500; j++) { __NOP(); }
        HAL_GPIO_WritePin(SSI_CLK_GPIO_Port, SSI_CLK_Pin, GPIO_PIN_SET);
        for (volatile int j = 0; j < 500; j++) { __NOP(); }
        if (HAL_GPIO_ReadPin(SSI_DATA_GPIO_Port, SSI_DATA_Pin) == GPIO_PIN_SET)
            raw |= (1 << i);
    }

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    for (volatile int i = 0; i < 500; i++) { __NOP(); }

    raw >>= 1;
    return raw & 0x3FFF;
}


void MX_FREERTOS_Init(void)
{
    osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    osThreadDef(controlTask, StartControlTask, osPriorityAboveNormal, 0, 512);
    controlTaskHandle = osThreadCreate(osThread(controlTask), NULL);
}


void StartDefaultTask(void const *argument)
{
    (void)argument;
    osDelay(2000);

    char tx_buf[128];
    uint32_t last_report = 0;

    for (;;) {
        // Parse incoming command
        if (cdc_rx_ready) {
            Parse_CDC_Command(cdc_rx_buf);
            cdc_rx_ready = 0;
        }

        // Send state
        uint32_t now = HAL_GetTick();
        if (now - last_report >= STATE_REPORT_PERIOD_MS) {
            last_report = now;
            snprintf(tx_buf, sizeof(tx_buf),
                "STATE:%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                enc_deg[0], enc_deg[1], enc_deg[2],
                enc_vel[0], enc_vel[1], enc_vel[2]);
            CDC_Transmit_FS((uint8_t *)tx_buf, strlen(tx_buf));
        }

        osDelay(5);
    }
}


void StartControlTask(void const *argument)
{
    (void)argument;

    // Deselect all CS
    HAL_GPIO_WritePin(motor1_adc_GPIO_Port, motor1_adc_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor2_adc_GPIO_Port, motor2_adc_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor3_adc_GPIO_Port, motor3_adc_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor1_Cs_GPIO_Port,  motor1_Cs_Pin,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor2_Cs_GPIO_Port,  motor2_Cs_Pin,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor3_Cs_GPIO_Port,  motor3_Cs_Pin,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(cs_enc_1_GPIO_Port,   cs_enc_1_Pin,   GPIO_PIN_SET);
    HAL_GPIO_WritePin(cs_enc_2_GPIO_Port,   cs_enc_2_Pin,   GPIO_PIN_SET);
    HAL_GPIO_WritePin(cs_enc_3_GPIO_Port,   cs_enc_3_Pin,   GPIO_PIN_SET);

    // Start PWM
    __HAL_TIM_MOE_ENABLE(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

    const float dt          = CONTROL_LOOP_PERIOD_MS / 1000.0f;
    const float DEG_PER_TICK = 360.0f / 16384.0f;

    for (;;) {
        enc_raw[0] = Read_SSI(cs_enc_1_GPIO_Port, cs_enc_1_Pin);
        enc_raw[1] = Read_SSI(cs_enc_2_GPIO_Port, cs_enc_2_Pin);
        enc_raw[2] = Read_SSI(cs_enc_3_GPIO_Port, cs_enc_3_Pin);

        for (int i = 0; i < 3; i++) {
            enc_deg[i] = (float)enc_raw[i] * DEG_PER_TICK;

            int16_t delta = (int16_t)enc_raw[i] - (int16_t)enc_prev[i];
            if (delta >  8192) delta -= 16384;
            if (delta < -8192) delta += 16384;
            enc_vel[i]  = (float)delta * DEG_PER_TICK / dt;
            enc_prev[i] = enc_raw[i];
        }

        uint32_t now = HAL_GetTick();
        float d0 = (now - cmd_last_tick < CMD_TIMEOUT_MS) ? motor_duty[0] : 0.0f;
        float d1 = (now - cmd_last_tick < CMD_TIMEOUT_MS) ? motor_duty[1] : 0.0f;
        float d2 = (now - cmd_last_tick < CMD_TIMEOUT_MS) ? motor_duty[2] : 0.0f;

        Motor_SetDuty(&htim1, TIM_CHANNEL_1,
            motor1.dir_port, motor1.dir_pin,
            motor1_nBRAKE_GPIO_Port, motor1_nBRAKE_Pin, d0);
        Motor_SetDuty(&htim3, TIM_CHANNEL_1,
            motor2.dir_port, motor2.dir_pin,
            motor2_nBRAKE_GPIO_Port, motor2_nBRAKE_Pin, d1);
        Motor_SetDuty(&htim4, TIM_CHANNEL_1,
            motor3.dir_port, motor3.dir_pin,
            motor3_nBRAKE_GPIO_Port, motor3_nBRAKE_Pin, d2);

        osDelay(CONTROL_LOOP_PERIOD_MS);
    }
}
