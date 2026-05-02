#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "motor.h"
#include "drv8353.h"
#include "tim.h"
#include "spi.h"
#include "usbd_cdc_if.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define CONTROL_LOOP_PERIOD_MS  5
#define STATE_REPORT_PERIOD_MS  100
#define CMD_TIMEOUT_MS          500

#define WHEEL_RADIUS    0.05f
#define ROBOT_RADIUS    0.15f

#define MOTOR1_ANGLE    4.1888f // 90 deg
#define MOTOR2_ANGLE    5.7596f // 240 deg
#define MOTOR3_ANGLE    1.5708f // 330 deg

#define COUNTS_PER_REV  4096.0f
#define TWO_PI          (2.0f * 3.14159265f)
#define DT              (CONTROL_LOOP_PERIOD_MS / 1000.0f)
#define MAX_DUTY              0.40f
#define DUTY_DEADZONE         0.01f
#define MIN_PWM               0.08f

#define NEG_PREKICK_DUTY      0.15f
#define NEG_PREKICK_MS        50
#define CDC_RX_BUF_SIZE 128

static char cdc_line_buf[CDC_RX_BUF_SIZE];
static volatile uint16_t cdc_line_len = 0;

static char cdc_cmd_buf[CDC_RX_BUF_SIZE];
static volatile uint8_t cdc_cmd_ready = 0;


extern motor_st motor1, motor2, motor3;
extern DRV8353_Handle drv_motor1, drv_motor2, drv_motor3;

volatile float cmd_vx    = 0.0f;
volatile float cmd_vy    = 0.0f;
volatile float cmd_omega = 0.0f;
volatile uint32_t cmd_last_tick = 0;

typedef struct {
    float kp, ki, kd;
    float integral;
    float prev_error;
    float output_min, output_max;
} PID_t;

static PID_t pid[3];

typedef struct {
    uint16_t prev_raw;
    int32_t  accum_counts;
    int32_t  accum_prev;
    float    velocity_rads;
} encoder_state_t;

encoder_state_t enc[3] = {0};

#define CDC_RX_BUF_SIZE 128
static char cdc_rx_buf[CDC_RX_BUF_SIZE];
static volatile uint16_t cdc_rx_len = 0;
static volatile uint8_t  cdc_rx_ready = 0;

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
    if (strncmp(str, "CMD:", 4) != 0) return;
    float vx, vy, omega;
    if (sscanf(str + 4, "%f,%f,%f", &vx, &vy, &omega) == 3) {
    	taskENTER_CRITICAL();
        cmd_vx        = vx;
        cmd_vy        = vy;
        cmd_omega     = omega;
        cmd_last_tick = HAL_GetTick();
        taskEXIT_CRITICAL();
    }
}

static void InvKinematics(float vx, float vy, float omega, float vel_out[3])
{
    float angles[3] = {
        MOTOR1_ANGLE,
        MOTOR2_ANGLE,
        MOTOR3_ANGLE
    };

    for (int i = 0; i < 3; i++) {
        vel_out[i] =
            (-sinf(angles[i]) * vx +
              cosf(angles[i]) * vy +
              omega * ROBOT_RADIUS) / WHEEL_RADIUS;
    }
}

static void PID_Init(PID_t *p, float kp, float ki, float kd,
                     float out_min, float out_max)
{
    p->kp         = kp;
    p->ki         = ki;
    p->kd         = kd;
    p->integral   = 0.0f;
    p->prev_error = 0.0f;
    p->output_min = out_min;
    p->output_max = out_max;
}

static float PID_Update(PID_t *p, float setpoint, float measured, float dt)
{
    float error = setpoint - measured;

    p->integral += error * dt;
    if (p->integral > p->output_max) p->integral = p->output_max;
    if (p->integral < p->output_min) p->integral = p->output_min;

    float derivative = (error - p->prev_error) / dt;
    p->prev_error = error;

    float output = p->kp * error + p->ki * p->integral + p->kd * derivative;
    if (output > p->output_max) output = p->output_max;
    if (output < p->output_min) output = p->output_min;

    return output;
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
    HAL_GPIO_WritePin(dir_port, dir_pin, duty >= 0.0f ? GPIO_PIN_SET : GPIO_PIN_RESET);
    float clamped = fminf(fabsf(duty), 0.4f);
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

    return (raw & 0x3FFF) >> 2;
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
        if (cdc_cmd_ready) {
            Parse_CDC_Command(cdc_cmd_buf);
            cdc_cmd_ready = 0;
        }

        uint32_t now = HAL_GetTick();
        if (now - last_report >= STATE_REPORT_PERIOD_MS) {
            last_report = now;
            // Position in radians, velocity in rad/s
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

        osDelay(10);
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

    PID_Init(&pid[0], 0.01f, 0.00f, 0.0f, -MAX_DUTY, MAX_DUTY);
    PID_Init(&pid[1], 0.01f, 0.00f, 0.0f, -MAX_DUTY, MAX_DUTY);
    PID_Init(&pid[2], 0.01f, 0.00f, 0.0f, -MAX_DUTY, MAX_DUTY);

    // Prime encoders to avoid first-tick velocity spike
    enc[0].prev_raw = Read_SSI(cs_enc_1_GPIO_Port, cs_enc_1_Pin, enc1_data_GPIO_Port, enc1_data_Pin);
    enc[1].prev_raw = Read_SSI(cs_enc_2_GPIO_Port, cs_enc_2_Pin, enc2_data_GPIO_Port, enc2_data_Pin);
    enc[2].prev_raw = Read_SSI(cs_enc_3_GPIO_Port, cs_enc_3_Pin, enc3_data_GPIO_Port, enc3_data_Pin);

    for (;;)
    {
        // --- Encoder reads ---
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

        float vx;
        float vy;
        float omega;

        taskENTER_CRITICAL();
        vx = cmd_vx;
        vy = cmd_vy;
        omega = cmd_omega;
        taskEXIT_CRITICAL();

        uint32_t now = HAL_GetTick();

        if (now - cmd_last_tick >= CMD_TIMEOUT_MS) {
            vx = 0.0f;
            vy = 0.0f;
            omega = 0.0f;
        }

        float duty[3] = {0.0f, 0.0f, 0.0f};


        // Forward / backward
        if (vx >= 0.0f) {
            duty[0] =  0.533f * vx;
            duty[1] =  0.533f * vx;
            duty[2] = -1.333f * vx;
        } else {
            duty[0] =  0.500f * vx;
            duty[1] =  0.500f * vx;
            duty[2] = -0.750f * vx;
        }

        // Left / right
        if (vy >= 0.0f) {
            duty[0] += -0.667f * vy;
            duty[1] +=  0.667f * vy;
            duty[2] +=  0.167f * vy;
        } else {
            duty[0] += -0.667f * vy;
            duty[1] +=  0.667f * vy;
            duty[2] +=  0.333f * vy;
        }

        // Rotation
        duty[0] += 0.50f * omega;
        duty[1] += 0.50f * omega;
        duty[2] += 0.50f * omega;

        // Clamp
        for (int i = 0; i < 3; i++) {
            if (duty[i] > MAX_DUTY) duty[i] = MAX_DUTY;
            if (duty[i] < -MAX_DUTY) duty[i] = -MAX_DUTY;
        }


        static float prev_cmd_duty[3] = {0.0f, 0.0f, 0.0f};
        static uint8_t neg_prekick_active[3] = {0, 0, 0};
        static uint32_t neg_prekick_until[3] = {0, 0, 0};

        uint32_t now2 = HAL_GetTick();

        for (int i = 0; i < 3; i++) {
            float commanded = duty[i];

            int was_stopped = fabsf(prev_cmd_duty[i]) < DUTY_DEADZONE;
            int wants_negative = commanded < -DUTY_DEADZONE;

            if (was_stopped && wants_negative && !neg_prekick_active[i]) {
                neg_prekick_active[i] = 1;
                neg_prekick_until[i] = now2 + NEG_PREKICK_MS;
            }

            if (neg_prekick_active[i]) {
                if (now2 < neg_prekick_until[i]) {
                    duty[i] = NEG_PREKICK_DUTY;
                } else {
                    neg_prekick_active[i] = 0;
                    duty[i] = commanded;
                }
            }

            if (fabsf(commanded) < DUTY_DEADZONE) {
                neg_prekick_active[i] = 0;
                neg_prekick_until[i] = 0;
            }

            prev_cmd_duty[i] = commanded;
        }

        // --- Motor drive ---
        Motor_SetDuty(&htim1, TIM_CHANNEL_1,
            motor1.dir_port, motor1.dir_pin,
            motor1_nBRAKE_GPIO_Port, motor1_nBRAKE_Pin, duty[0]);
        Motor_SetDuty(&htim3, TIM_CHANNEL_1,
            motor2.dir_port, motor2.dir_pin,
            motor2_nBRAKE_GPIO_Port, motor2_nBRAKE_Pin, duty[1]);
        Motor_SetDuty(&htim4, TIM_CHANNEL_1,
            motor3.dir_port, motor3.dir_pin,
            motor3_nBRAKE_GPIO_Port, motor3_nBRAKE_Pin, duty[2]);

        osDelay(CONTROL_LOOP_PERIOD_MS);
    }
}
