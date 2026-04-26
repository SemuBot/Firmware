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
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rmw_microros/rmw_microros.h>
#include "usb_cdc_transport.h"
#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include "diagnostic_msgs/msg/diagnostic_array.h"
#include "diagnostic_msgs/msg/diagnostic_status.h"
#include "diagnostic_msgs/msg/key_value.h"

#define CONTROL_LOOP_PERIOD_MS  5
#define JOINT_STATE_PERIOD_MS   10
#define DIAGNOSTICS_PERIOD_MS   100
#define CMD_TIMEOUT_MS          1000

#define WHEEL_RADIUS    0.05f
#define ROBOT_RADIUS    0.15f
#define MOTOR1_ANGLE    4.1888f  // 240 deg, rear-left
#define MOTOR2_ANGLE    5.7596f  // 330 deg, rear-right
#define MOTOR3_ANGLE    1.5708f  // 90 deg, front
#define COUNTS_PER_REV  4096.0f
#define TWO_PI          (2.0f * 3.14159265f)
#define DT              (CONTROL_LOOP_PERIOD_MS / 1000.0f)

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

// micro-ROS
rcl_node_t node;
rclc_support_t support;
rcl_subscription_t cmd_vel_sub;
rcl_publisher_t joint_state_pub;
rcl_publisher_t diagnostics_pub;
rclc_executor_t executor;
sensor_msgs__msg__JointState joint_state_msg;
geometry_msgs__msg__Twist cmd_vel_msg;
rcl_timer_t joint_state_timer;
rcl_timer_t diagnostics_timer;
diagnostic_msgs__msg__DiagnosticArray diagnostics_msg;
char g_debug_msg[128] = "init";

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

void uros_fini_all(void)
{
    rcl_ret_t ret;
    ret = rcl_subscription_fini(&cmd_vel_sub, &node);   (void)ret;
    ret = rcl_publisher_fini(&joint_state_pub, &node);  (void)ret;
    ret = rcl_publisher_fini(&diagnostics_pub, &node);  (void)ret;
    ret = rcl_timer_fini(&joint_state_timer);            (void)ret;
    ret = rcl_timer_fini(&diagnostics_timer);            (void)ret;
    ret = rclc_executor_fini(&executor);                 (void)ret;
    ret = rcl_node_fini(&node);                          (void)ret;
    rclc_support_fini(&support);
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

void cmd_vel_callback(const void *msgin)
{
    const geometry_msgs__msg__Twist *msg =
        (const geometry_msgs__msg__Twist *)msgin;
    cmd_vx        = (float)msg->linear.x;
    cmd_vy        = (float)msg->linear.y;
    cmd_omega     = (float)msg->angular.z;
    cmd_last_tick = HAL_GetTick();
}

void joint_state_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer == NULL) return;

    uint32_t tick = HAL_GetTick();
    joint_state_msg.header.stamp.sec     = tick / 1000;
    joint_state_msg.header.stamp.nanosec = (tick % 1000) * 1000000;

    const float COUNTS_TO_RAD = TWO_PI / COUNTS_PER_REV;

    joint_state_msg.position.data[0] = (double)enc[2].accum_counts * COUNTS_TO_RAD; // M3
    joint_state_msg.position.data[1] = (double)enc[0].accum_counts * COUNTS_TO_RAD; // M1
    joint_state_msg.position.data[2] = (double)enc[1].accum_counts * COUNTS_TO_RAD; // M2

    joint_state_msg.velocity.data[0] = (double)enc[2].velocity_rads;
    joint_state_msg.velocity.data[1] = (double)enc[0].velocity_rads;
    joint_state_msg.velocity.data[2] = (double)enc[1].velocity_rads;

    joint_state_msg.effort.data[0] = 0.0;
    joint_state_msg.effort.data[1] = 0.0;
    joint_state_msg.effort.data[2] = 0.0;

    rcl_ret_t ret = rcl_publish(&joint_state_pub, &joint_state_msg, NULL);
    (void)ret;
}

void diagnostics_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)timer;
    (void)last_call_time;

    uint32_t tick = HAL_GetTick();
    diagnostics_msg.header.stamp.sec     = tick / 1000;
    diagnostics_msg.header.stamp.nanosec = (tick % 1000) * 1000000;

    char uptime_buf[16];
    snprintf(uptime_buf, sizeof(uptime_buf), "%lu", tick);
    rosidl_runtime_c__String__assign(
        &diagnostics_msg.status.data[0].values.data[1].value, uptime_buf);
    rosidl_runtime_c__String__assign(
        &diagnostics_msg.status.data[0].message, g_debug_msg);

    rcl_ret_t ret = rcl_publish(&diagnostics_pub, &diagnostics_msg, NULL);
    (void)ret;
}

void MX_FREERTOS_Init(void)
{
    osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 4000);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
    osThreadDef(controlTask, StartControlTask, osPriorityAboveNormal, 0, 512);
    controlTaskHandle = osThreadCreate(osThread(controlTask), NULL);
}

void StartDefaultTask(void const *argument)
{
    (void)argument;
    osDelay(2000);

    rmw_uros_set_custom_transport(
        true, NULL,
        cubemx_transport_open,
        cubemx_transport_close,
        cubemx_transport_write,
        cubemx_transport_read);

    rcl_allocator_t allocator = rcl_get_default_allocator();

    for (;;)
    {
        while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
            HAL_GPIO_TogglePin(debug_GPIO_Port, debug_Pin);
            osDelay(500);
        }
        HAL_GPIO_WritePin(debug_GPIO_Port, debug_Pin, GPIO_PIN_SET);

        rclc_support_init(&support, 0, NULL, &allocator);
        rclc_node_init_default(&node, "semubot_onboard", "", &support);

        rclc_subscription_init_best_effort(
            &cmd_vel_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "/cmd_vel");

        sensor_msgs__msg__JointState__init(&joint_state_msg);
        rosidl_runtime_c__String__Sequence__init(&joint_state_msg.name, 3);
        rosidl_runtime_c__double__Sequence__init(&joint_state_msg.position, 3);
        rosidl_runtime_c__double__Sequence__init(&joint_state_msg.velocity, 3);
        rosidl_runtime_c__double__Sequence__init(&joint_state_msg.effort, 3);
        rosidl_runtime_c__String__assign(&joint_state_msg.name.data[0], "omni_ball_3_joint");
        rosidl_runtime_c__String__assign(&joint_state_msg.name.data[1], "omni_ball_1_joint");
        rosidl_runtime_c__String__assign(&joint_state_msg.name.data[2], "omni_ball_2_joint");

        rclc_publisher_init_default(&joint_state_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "/motor_states");

        diagnostic_msgs__msg__DiagnosticArray__init(&diagnostics_msg);
        diagnostic_msgs__msg__DiagnosticStatus__Sequence__init(&diagnostics_msg.status, 1);
        diagnostic_msgs__msg__DiagnosticStatus__init(&diagnostics_msg.status.data[0]);
        rosidl_runtime_c__String__assign(&diagnostics_msg.status.data[0].name, "semubot");
        rosidl_runtime_c__String__assign(&diagnostics_msg.status.data[0].message, "System OK");
        rosidl_runtime_c__String__assign(&diagnostics_msg.status.data[0].hardware_id, "wheelbase");
        diagnostic_msgs__msg__KeyValue__Sequence__init(&diagnostics_msg.status.data[0].values, 2);
        for (int i = 0; i < 2; i++)
            diagnostic_msgs__msg__KeyValue__init(&diagnostics_msg.status.data[0].values.data[i]);
        rosidl_runtime_c__String__assign(&diagnostics_msg.status.data[0].values.data[0].key, "motor_errors");
        rosidl_runtime_c__String__assign(&diagnostics_msg.status.data[0].values.data[0].value, "0x00");
        rosidl_runtime_c__String__assign(&diagnostics_msg.status.data[0].values.data[1].key, "uptime_ms");
        rosidl_runtime_c__String__assign(&diagnostics_msg.status.data[0].values.data[1].value, "0");

        rclc_publisher_init_default(&diagnostics_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticArray), "/diagnostics");

        rclc_timer_init_default2(&joint_state_timer, &support,
            RCL_MS_TO_NS(JOINT_STATE_PERIOD_MS), joint_state_timer_callback, true);
        rclc_timer_init_default2(&diagnostics_timer, &support,
            RCL_MS_TO_NS(DIAGNOSTICS_PERIOD_MS), diagnostics_timer_callback, true);

        executor = rclc_executor_get_zero_initialized_executor();
        rclc_executor_init(&executor, &support.context, 3, &allocator);
        rclc_executor_add_subscription(&executor, &cmd_vel_sub,
            &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);
        rclc_executor_add_timer(&executor, &joint_state_timer);
        rclc_executor_add_timer(&executor, &diagnostics_timer);

        uint32_t last_ping = HAL_GetTick();
        bool connected = true;

        while (connected) {
            if (HAL_GetTick() - last_ping > 2000) {
                last_ping = HAL_GetTick();
                if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK)
                    connected = false;
            }
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
            osDelay(10);
        }

        // Connection lost — stop motors
        Motor_SetDuty(&htim1, TIM_CHANNEL_1,
            motor1.dir_port, motor1.dir_pin,
            motor1_nBRAKE_GPIO_Port, motor1_nBRAKE_Pin, 0.0f);
        Motor_SetDuty(&htim3, TIM_CHANNEL_1,
            motor2.dir_port, motor2.dir_pin,
            motor2_nBRAKE_GPIO_Port, motor2_nBRAKE_Pin, 0.0f);
        Motor_SetDuty(&htim4, TIM_CHANNEL_1,
            motor3.dir_port, motor3.dir_pin,
            motor3_nBRAKE_GPIO_Port, motor3_nBRAKE_Pin, 0.0f);

        HAL_GPIO_WritePin(debug_GPIO_Port, debug_Pin, GPIO_PIN_RESET);
        uros_fini_all();
        osDelay(500);
    }
}

void StartControlTask(void const *argument)
{
    (void)argument;

    HAL_GPIO_WritePin(motor1_adc_GPIO_Port, motor1_adc_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor2_adc_GPIO_Port, motor2_adc_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor3_adc_GPIO_Port, motor3_adc_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor1_Cs_GPIO_Port,  motor1_Cs_Pin,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor2_Cs_GPIO_Port,  motor2_Cs_Pin,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(motor3_Cs_GPIO_Port,  motor3_Cs_Pin,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(cs_enc_1_GPIO_Port,   cs_enc_1_Pin,   GPIO_PIN_SET);
    HAL_GPIO_WritePin(cs_enc_2_GPIO_Port,   cs_enc_2_Pin,   GPIO_PIN_SET);
    HAL_GPIO_WritePin(cs_enc_3_GPIO_Port,   cs_enc_3_Pin,   GPIO_PIN_SET);

    __HAL_TIM_MOE_ENABLE(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

    PID_Init(&pid[0], 0.01f, 0.00f, 0.0f, -0.3f, 0.3f);
    PID_Init(&pid[1], 0.01f, 0.00f, 0.0f, -0.3f, 0.3f);
    PID_Init(&pid[2], 0.01f, 0.00f, 0.0f, -0.3f, 0.3f);

    // Prime encoders
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

        // --- Inverse kinematics ---
        float vel_target[3] = {0};
        uint32_t now = HAL_GetTick();
        if (now - cmd_last_tick < CMD_TIMEOUT_MS) {
            InvKinematics(cmd_vx, cmd_vy, cmd_omega, vel_target);
        } else {
            // Timeout — reset PID state
            for (int i = 0; i < 3; i++) {
                pid[i].integral   = 0.0f;
                pid[i].prev_error = 0.0f;
            }
        }

        // --- PID ---
        float duty[3];
        for (int i = 0; i < 3; i++) {
            if (fabsf(vel_target[i]) < 0.01f) {
                duty[i] = 0.0f;
                pid[i].integral = 0.0f;
                pid[i].prev_error = 0.0f;
            } else {
                float ff = vel_target[i] * 0.05f;
                float corr = PID_Update(&pid[i], vel_target[i], enc[i].velocity_rads, DT);

                duty[i] = ff + corr;

                if (duty[i] > 0.30f) duty[i] = 0.30f;
                if (duty[i] < -0.30f) duty[i] = -0.30f;

                const float MIN_PWM = 0.30f;
                if (fabsf(duty[i]) < MIN_PWM) {
                    duty[i] = copysignf(MIN_PWM, duty[i]);
                }
            }
        }
        static float prev_duty_cmd[3] = {0.0f, 0.0f, 0.0f};
        static uint32_t kick_until[3] = {0, 0, 0};

        for (int i = 0; i < 3; i++) {
            if (fabsf(prev_duty_cmd[i]) < 0.01f && fabsf(duty[i]) > 0.01f) {
                kick_until[i] = HAL_GetTick() + 120;
            }

            if (HAL_GetTick() < kick_until[i]) {
                duty[i] = copysignf(0.40f, duty[i]);
            }

            prev_duty_cmd[i] = duty[i];
        }
        snprintf(g_debug_msg, sizeof(g_debug_msg),
                 "T:%.2f %.2f %.2f V:%.2f %.2f %.2f",
                 vel_target[0], vel_target[1], vel_target[2],
                 enc[0].velocity_rads, enc[1].velocity_rads, enc[2].velocity_rads);

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
