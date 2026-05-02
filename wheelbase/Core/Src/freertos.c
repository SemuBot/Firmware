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
#define STATE_REPORT_PERIOD_MS  20
#define MAX_DUTY      0.40f
#define DUTY_DEADZONE 0.02f
#define NEG_PREKICK_DUTY      0.10f
#define NEG_PREKICK_MS        50

char debug_msg[256];
extern motor_st motor1, motor2, motor3;
extern DRV8353_Handle drv_motor1, drv_motor2, drv_motor3;
/* USER CODE END PD */

/* USER CODE BEGIN Variables */

volatile float motor_duty[3] = {0.0f, 0.0f, 0.0f};
volatile uint32_t cmd_last_tick = 0;
#define CMD_TIMEOUT_MS 500

#define COUNTS_PER_REV  4096.0f
#define TWO_PI          (2.0f * 3.14159265f)


// Encoder state
typedef struct {
    uint16_t prev_raw;
    int32_t  accum_counts;
    int32_t  accum_prev;
    float    velocity_rads;
} encoder_state_t;

encoder_state_t enc[3] = {0};

// CDC receive buffer
#define CDC_RX_BUF_SIZE 128

static char cdc_line_buf[CDC_RX_BUF_SIZE];
static volatile uint16_t cdc_line_len = 0;

static char cdc_cmd_buf[CDC_RX_BUF_SIZE];
static volatile uint8_t cdc_cmd_ready = 0;

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
    for (uint32_t i = 0; i < len; i++) {
        char c = (char)buf[i];

        if (c == '\r') {
            continue;
        }

        if (c == '\n') {
            cdc_line_buf[cdc_line_len] = '\0';

            if (!cdc_cmd_ready) {
                memcpy(cdc_cmd_buf, cdc_line_buf, cdc_line_len + 1);
                cdc_cmd_ready = 1;
            }

            cdc_line_len = 0;
        } else {
            if (cdc_line_len < CDC_RX_BUF_SIZE - 1) {
                cdc_line_buf[cdc_line_len++] = c;
            } else {
                cdc_line_len = 0;
            }
        }
    }
}


static void Parse_CDC_Command(const char *str)
{
    if (strncmp(str, "CMD:", 4) != 0) {
        return;
    }

    float v1 = 0.0f;
    float v2 = 0.0f;
    float v3 = 0.0f;

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

    if (fabsf(duty) < DUTY_DEADZONE) {
        __HAL_TIM_SET_COMPARE(htim, channel, 0);
        HAL_GPIO_WritePin(brake_port, brake_pin, GPIO_PIN_SET);
        return;
    }

    float clamped = fminf(fabsf(duty), MAX_DUTY);

    HAL_GPIO_WritePin(dir_port, dir_pin,
        duty >= 0.0f ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(brake_port, brake_pin, GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(htim, channel, (uint32_t)(clamped * arr));
}



uint16_t Read_SSI(GPIO_TypeDef *cs_port, uint16_t cs_pin,
                  GPIO_TypeDef *data_port, uint16_t data_pin)
{
    uint16_t raw = 0;

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    for (volatile int i = 0; i < 36; i++) { __NOP(); }

    for (int i = 15; i >= 0; i--) {
        HAL_GPIO_WritePin(SSI_CLK_GPIO_Port, SSI_CLK_Pin, GPIO_PIN_RESET);
        for (volatile int j = 0; j < 500; j++) { __NOP(); }

        HAL_GPIO_WritePin(SSI_CLK_GPIO_Port, SSI_CLK_Pin, GPIO_PIN_SET);
        for (volatile int j = 0; j < 500; j++) { __NOP(); }

        if (HAL_GPIO_ReadPin(data_port, data_pin) == GPIO_PIN_SET) {
            raw |= (1 << i);
        }
    }

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);
    for (volatile int i = 0; i < 500; i++) { __NOP(); }

    return (raw & 0x3FFF) >> 2;   // 12-bit position: 0..4095
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
    uint32_t last_fault_check = 0;
    for (;;) {
        // Parse incoming command
        uint32_t now = HAL_GetTick();

        if (cdc_cmd_ready) {
            Parse_CDC_Command(cdc_cmd_buf);
            cdc_cmd_ready = 0;
        }
        if (now - last_fault_check >= 500) {
            last_fault_check = now;

            uint16_t f1 = 0, f2 = 0, f3 = 0;
            uint16_t v1 = 0, v2 = 0, v3 = 0;

            DRV8353_ReadRegister(&drv_motor1, DRV8353_REG_FAULT_STATUS_1, &f1);
            DRV8353_ReadRegister(&drv_motor2, DRV8353_REG_FAULT_STATUS_1, &f2);
            DRV8353_ReadRegister(&drv_motor3, DRV8353_REG_FAULT_STATUS_1, &f3);
            DRV8353_ReadRegister(&drv_motor1, DRV8353_REG_VGS_STATUS_2, &v1);
            DRV8353_ReadRegister(&drv_motor2, DRV8353_REG_VGS_STATUS_2, &v2);
            DRV8353_ReadRegister(&drv_motor3, DRV8353_REG_VGS_STATUS_2, &v3);

            snprintf(tx_buf, sizeof(tx_buf),
                "FAULT:M1=%04X/%04X M2=%04X/%04X M3=%04X/%04X\n",
                f1, v1, f2, v2, f3, v3);
            CDC_Transmit_FS((uint8_t *)tx_buf, strlen(tx_buf));
        	uint16_t ctrl = 0;
        	DRV8353_ReadRegister(&drv_motor1, DRV8353_REG_DRIVER_CONTROL, &ctrl);
        	DRV8353_WriteRegister(&drv_motor1, DRV8353_REG_DRIVER_CONTROL, ctrl | DRV8353_CLR_FLT);

        	DRV8353_ReadRegister(&drv_motor2, DRV8353_REG_DRIVER_CONTROL, &ctrl);
        	DRV8353_WriteRegister(&drv_motor2, DRV8353_REG_DRIVER_CONTROL, ctrl | DRV8353_CLR_FLT);

        	DRV8353_ReadRegister(&drv_motor3, DRV8353_REG_DRIVER_CONTROL, &ctrl);
        	DRV8353_WriteRegister(&drv_motor3, DRV8353_REG_DRIVER_CONTROL, ctrl | DRV8353_CLR_FLT);

        }
        // Send state
        if (now - last_report >= STATE_REPORT_PERIOD_MS) {
            last_report = now;
            snprintf(tx_buf, sizeof(tx_buf),
                "STATE:%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
                (float)enc[0].accum_counts / COUNTS_PER_REV * TWO_PI,
                (float)enc[1].accum_counts / COUNTS_PER_REV * TWO_PI,
                (float)enc[2].accum_counts / COUNTS_PER_REV * TWO_PI,
                enc[0].velocity_rads,
                enc[1].velocity_rads,
                enc[2].velocity_rads);
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

    enc[0].prev_raw = Read_SSI(cs_enc_1_GPIO_Port, cs_enc_1_Pin, enc1_data_GPIO_Port, enc1_data_Pin);
    enc[1].prev_raw = Read_SSI(cs_enc_2_GPIO_Port, cs_enc_2_Pin, enc2_data_GPIO_Port, enc2_data_Pin);
    enc[2].prev_raw = Read_SSI(cs_enc_3_GPIO_Port, cs_enc_3_Pin, enc3_data_GPIO_Port, enc3_data_Pin);

    const float DT             = CONTROL_LOOP_PERIOD_MS / 1000.0f;

    for (;;) {
        uint16_t raw[3];
        raw[0] = Read_SSI(cs_enc_1_GPIO_Port, cs_enc_1_Pin, enc1_data_GPIO_Port, enc1_data_Pin);
        raw[1] = Read_SSI(cs_enc_2_GPIO_Port, cs_enc_2_Pin, enc2_data_GPIO_Port, enc2_data_Pin);
        raw[2] = Read_SSI(cs_enc_3_GPIO_Port, cs_enc_3_Pin, enc3_data_GPIO_Port, enc3_data_Pin);

        for (int i = 0; i < 3; i++) {
            int32_t delta = (int32_t)raw[i] - (int32_t)enc[i].prev_raw;

            if (delta > 2048) {
                delta -= 4096;
            } else if (delta < -2048) {
                delta += 4096;
            }

            if (delta > 1000 || delta < -1000) {
                delta = 0;
            }

            enc[i].accum_counts += delta;
            enc[i].prev_raw = raw[i];

            int32_t delta_for_vel = enc[i].accum_counts - enc[i].accum_prev;
            enc[i].accum_prev = enc[i].accum_counts;

            enc[i].velocity_rads =
                ((float)delta_for_vel / COUNTS_PER_REV) * TWO_PI / DT;
        }
        uint32_t now = HAL_GetTick();
        float d0 = (now - cmd_last_tick < CMD_TIMEOUT_MS) ? motor_duty[0] : 0.0f;
        float d1 = (now - cmd_last_tick < CMD_TIMEOUT_MS) ? motor_duty[1] : 0.0f;
        float d2 = (now - cmd_last_tick < CMD_TIMEOUT_MS) ? motor_duty[2] : 0.0f;
        float d[3] = {d0, d1, d2};
        static float prev_cmd[3] = {0.0f, 0.0f, 0.0f};
        static uint8_t neg_prekick_active[3] = {0, 0, 0};
        static uint32_t neg_prekick_until[3] = {0, 0, 0};

        for (int i = 0; i < 3; i++) {
            float commanded = d[i];

            int was_stopped = fabsf(prev_cmd[i]) < DUTY_DEADZONE;
            int wants_negative = commanded < -DUTY_DEADZONE;

            if (was_stopped && wants_negative && !neg_prekick_active[i]) {
                neg_prekick_active[i] = 1;
                neg_prekick_until[i] = now + NEG_PREKICK_MS;
            }

            if (neg_prekick_active[i]) {
                if (now < neg_prekick_until[i]) {
                    d[i] = NEG_PREKICK_DUTY;   // short positive kick
                } else {
                    neg_prekick_active[i] = 0;
                    d[i] = commanded;          // now apply real negative command
                }
            }

            prev_cmd[i] = commanded;
        }

        Motor_SetDuty(&htim1, TIM_CHANNEL_1,
            motor1.dir_port, motor1.dir_pin,
            motor1_nBRAKE_GPIO_Port, motor1_nBRAKE_Pin, d[0]);
        Motor_SetDuty(&htim3, TIM_CHANNEL_1,
            motor2.dir_port, motor2.dir_pin,
            motor2_nBRAKE_GPIO_Port, motor2_nBRAKE_Pin, d[1]);
        Motor_SetDuty(&htim4, TIM_CHANNEL_1,
            motor3.dir_port, motor3.dir_pin,
            motor3_nBRAKE_GPIO_Port, motor3_nBRAKE_Pin, d[2]);

        osDelay(CONTROL_LOOP_PERIOD_MS);
    }
}
