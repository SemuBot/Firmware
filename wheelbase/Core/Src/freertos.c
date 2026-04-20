/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "drv8353.h"
#include "tim.h"
#include "spi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#include "sensor_msgs/msg/joint_state.h"
#include "diagnostic_msgs/msg/diagnostic_array.h"
#include "diagnostic_msgs/msg/diagnostic_status.h"
#include "diagnostic_msgs/msg/key_value.h"
#include "rosidl_runtime_c/string_functions.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <rmw_microros/rmw_microros.h>
#include "usb_cdc_transport.h"
#include "usbd_cdc_if.h"
#include <math.h>
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define CONTROL_LOOP_PERIOD_MS 1
#define JOINT_STATE_PERIOD_MS 10
#define DIAGNOSTICS_PERIOD_MS 100

typedef struct {
    uint16_t prev_raw;
    int32_t  accum_counts;
    int32_t  accum_prev;
    float    velocity_rads;
} encoder_state_t;

static encoder_state_t enc[3] = {0};
volatile float motor_current[3][3] = {0};  // [motor][phase]



char debug_msg[256];
extern motor_st motor1, motor2, motor3;
extern DRV8353_Handle drv_motor1, drv_motor2, drv_motor3;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void velocity_command_callback(const void * msgin);
void joint_state_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void diagnostics_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
volatile float target_velocities[3] = {0.0f, 0.0f, 0.0f};
#define CMD_TIMEOUT_MS 200

/* USER CODE END Variables */
rcl_node_t node;
rclc_support_t support;
rcl_publisher_t joint_state_pub;
rcl_publisher_t diagnostics_pub;
sensor_msgs__msg__JointState joint_state_msg;

rcl_subscription_t velocity_cmd_sub;
rclc_executor_t executor;
std_msgs__msg__Float32MultiArray velocity_cmd_msg;

// Timer handles
rcl_timer_t joint_state_timer;
rcl_timer_t diagnostics_timer;

// Diagnostic messages
diagnostic_msgs__msg__DiagnosticArray diagnostics_msg;
char g_debug_msg[128] = "init";
volatile uint32_t last_cmd_time = 0;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId controlTaskHandle;
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartControlTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  //osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 256);
 // defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 4000);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of controlTask */
  osThreadDef(controlTask, StartControlTask, osPriorityNormal, 0, 256);
  controlTaskHandle = osThreadCreate(osThread(controlTask), NULL);
  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

void uros_fini_all(void)
{
    rcl_ret_t ret; // Suppressing compiler warnings
    ret = rcl_subscription_fini(&velocity_cmd_sub, &node); (void)ret;
    ret = rcl_publisher_fini(&joint_state_pub, &node);     (void)ret;
    ret = rcl_publisher_fini(&diagnostics_pub, &node);     (void)ret;
    ret = rcl_timer_fini(&joint_state_timer);              (void)ret;
    ret = rcl_timer_fini(&diagnostics_timer);              (void)ret;
    ret = rclc_executor_fini(&executor);                   (void)ret;
    ret = rcl_node_fini(&node);                            (void)ret;
    rclc_support_fini(&support);
}


void StartDefaultTask(void const *argument)
{
    osDelay(2000);

    rmw_uros_set_custom_transport(
        true, NULL,
        cubemx_transport_open,
        cubemx_transport_close,
        cubemx_transport_write,
        cubemx_transport_read);

    for (;;) {

        // Wait for agent — slow blink
        while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
            HAL_GPIO_TogglePin(debug_GPIO_Port, debug_Pin);
            osDelay(500);
        }

        HAL_GPIO_WritePin(debug_GPIO_Port, debug_Pin, GPIO_PIN_SET);

        // Initialize micro-ROS
        rcl_allocator_t allocator = rcl_get_default_allocator();
        rclc_support_init(&support, 0, NULL, &allocator);
        rclc_node_init_default(&node, "motor_controller", "", &support);

        rclc_subscription_init_best_effort(
            &velocity_cmd_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            "/hardware_interface/velocity_cmd");

        sensor_msgs__msg__JointState__init(&joint_state_msg);
        rosidl_runtime_c__String__Sequence__init(&joint_state_msg.name, 3);
        rosidl_runtime_c__double__Sequence__init(&joint_state_msg.position, 3);
        rosidl_runtime_c__double__Sequence__init(&joint_state_msg.velocity, 3);
        rosidl_runtime_c__double__Sequence__init(&joint_state_msg.effort, 3);
        rosidl_runtime_c__String__assign(&joint_state_msg.name.data[0], "omni_ball_1_joint");
        rosidl_runtime_c__String__assign(&joint_state_msg.name.data[1], "omni_ball_2_joint");
        rosidl_runtime_c__String__assign(&joint_state_msg.name.data[2], "omni_ball_3_joint");

        diagnostic_msgs__msg__DiagnosticArray__init(&diagnostics_msg);
        diagnostic_msgs__msg__DiagnosticStatus__Sequence__init(&diagnostics_msg.status, 1);
        diagnostic_msgs__msg__DiagnosticStatus__init(&diagnostics_msg.status.data[0]);
        diagnostics_msg.status.data[0].level = 0;
        rosidl_runtime_c__String__assign(&diagnostics_msg.status.data[0].name, "__heartbeat__");
        rosidl_runtime_c__String__assign(&diagnostics_msg.status.data[0].message, "System OK");
        rosidl_runtime_c__String__assign(&diagnostics_msg.status.data[0].hardware_id, "wheelbase");
        diagnostic_msgs__msg__KeyValue__Sequence__init(&diagnostics_msg.status.data[0].values, 2);
        for (int i = 0; i < 2; i++)
            diagnostic_msgs__msg__KeyValue__init(&diagnostics_msg.status.data[0].values.data[i]);
        rosidl_runtime_c__String__assign(&diagnostics_msg.status.data[0].values.data[0].key, "motor_errors");
        rosidl_runtime_c__String__assign(&diagnostics_msg.status.data[0].values.data[0].value, "0x00");
        rosidl_runtime_c__String__assign(&diagnostics_msg.status.data[0].values.data[1].key, "uptime_ms");
        rosidl_runtime_c__String__assign(&diagnostics_msg.status.data[0].values.data[1].value, "0");

        rclc_publisher_init_default(&joint_state_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "motor_states");
        rclc_publisher_init_default(&diagnostics_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(diagnostic_msgs, msg, DiagnosticArray), "diagnostics");

        rclc_timer_init_default2(&joint_state_timer, &support,
            RCL_MS_TO_NS(JOINT_STATE_PERIOD_MS), joint_state_timer_callback, true);
        rclc_timer_init_default2(&diagnostics_timer, &support,
            RCL_MS_TO_NS(DIAGNOSTICS_PERIOD_MS), diagnostics_timer_callback, true);

        executor = rclc_executor_get_zero_initialized_executor();
        rclc_executor_init(&executor, &support.context, 3, &allocator);
        rclc_executor_add_timer(&executor, &joint_state_timer);
        rclc_executor_add_timer(&executor, &diagnostics_timer);
        rosidl_runtime_c__float__Sequence__init(&velocity_cmd_msg.data, 3);
        rclc_executor_add_subscription(&executor, &velocity_cmd_sub,
            &velocity_cmd_msg, &velocity_command_callback, ON_NEW_DATA);

        //ping agent every 2 seconds
        uint32_t last_ping = HAL_GetTick();
        bool connected = true;

        while (connected) {
            if (HAL_GetTick() - last_ping > 2000) {
                last_ping = HAL_GetTick();
                if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
                    connected = false;
                }
            }
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
            osDelay(10);
        }

        // Connection lost — stop motors and blink
        target_velocities[0] = 0.0f;
        target_velocities[1] = 0.0f;
        target_velocities[2] = 0.0f;
        HAL_GPIO_WritePin(debug_GPIO_Port, debug_Pin, GPIO_PIN_RESET);

        // Clean up
        uros_fini_all();

        osDelay(500);
    }
}



uint16_t Read_SSI(GPIO_TypeDef* cs_port, uint16_t cs_pin,
                  GPIO_TypeDef* data_port, uint16_t data_pin)
{
    uint16_t raw = 0;

    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    for (volatile int i = 0; i < 36; i++) { __NOP(); }

    for (int i = 16; i >= 0; i--) {
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

    raw >>= 1;
    return raw & 0x3FFF;
}


void StartControlTask(void const * argument)
{
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

    // Start PWM timers
    __HAL_TIM_MOE_ENABLE(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

    uint32_t arr_m1 = __HAL_TIM_GET_AUTORELOAD(&htim1);
    uint32_t arr_m2 = __HAL_TIM_GET_AUTORELOAD(&htim3);
    uint32_t arr_m3 = __HAL_TIM_GET_AUTORELOAD(&htim4);

    const float MAX_DUTY      = 0.30f;
    const float COUNTS_PER_REV = 16384.0f;
    const float DT            = 0.001f;
    const float TWO_PI        = 2.0f * (float)M_PI;

    // Prime encoders
    enc[0].prev_raw = Read_SSI(cs_enc_1_GPIO_Port, cs_enc_1_Pin, enc1_data_GPIO_Port, enc1_data_Pin);
    enc[1].prev_raw = Read_SSI(cs_enc_2_GPIO_Port, cs_enc_2_Pin, enc2_data_GPIO_Port, enc2_data_Pin);
    enc[2].prev_raw = Read_SSI(cs_enc_3_GPIO_Port, cs_enc_3_Pin, enc3_data_GPIO_Port, enc3_data_Pin);



    for (;;)
    {
        // --- Command timeout ---
        if (HAL_GetTick() - last_cmd_time > CMD_TIMEOUT_MS) {
            target_velocities[0] = 0.0f;
            target_velocities[1] = 0.0f;
            target_velocities[2] = 0.0f;
        }



        // --- Encoder reads ---
        uint16_t raw[3];
        raw[0] = Read_SSI(cs_enc_1_GPIO_Port, cs_enc_1_Pin, enc1_data_GPIO_Port, enc1_data_Pin);
        raw[1] = Read_SSI(cs_enc_2_GPIO_Port, cs_enc_2_Pin, enc2_data_GPIO_Port, enc2_data_Pin);
        raw[2] = Read_SSI(cs_enc_3_GPIO_Port, cs_enc_3_Pin, enc3_data_GPIO_Port, enc3_data_Pin);

        for (int i = 0; i < 3; i++)
        {
            int16_t delta = (int16_t)(raw[i] - enc[i].prev_raw);
            enc[i].accum_counts += delta;
            enc[i].prev_raw = raw[i];
            int32_t delta_for_vel = enc[i].accum_counts - enc[i].accum_prev;
            enc[i].accum_prev = enc[i].accum_counts;
            enc[i].velocity_rads = ((float)delta_for_vel / COUNTS_PER_REV) * TWO_PI / DT;
        }

        // --- Motor drive ---
        float mv1 = (float)target_velocities[0];
        float mv2 = (float)target_velocities[1];
        float mv3 = (float)target_velocities[2];

        HAL_GPIO_WritePin(motor1.dir_port, motor1.dir_pin,
            mv1 >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor1_nBRAKE_GPIO_Port, motor1_nBRAKE_Pin,
            fabsf(mv1) > 0.01f ? GPIO_PIN_SET : GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
            (uint32_t)(fminf(fabsf(mv1), MAX_DUTY) * arr_m1));

        HAL_GPIO_WritePin(motor2.dir_port, motor2.dir_pin,
            mv2 >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor2_nBRAKE_GPIO_Port, motor2_nBRAKE_Pin,
            fabsf(mv2) > 0.01f ? GPIO_PIN_SET : GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,
            (uint32_t)(fminf(fabsf(mv2), MAX_DUTY) * arr_m2));

        HAL_GPIO_WritePin(motor3.dir_port, motor3.dir_pin,
            mv3 >= 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(motor3_nBRAKE_GPIO_Port, motor3_nBRAKE_Pin,
            fabsf(mv3) > 0.01f ? GPIO_PIN_SET : GPIO_PIN_RESET);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,
            (uint32_t)(fminf(fabsf(mv3), MAX_DUTY) * arr_m3));

        osDelay(1);
    }
}



/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void velocity_command_callback(const void * msgin)
{
    last_cmd_time = HAL_GetTick();
    const std_msgs__msg__Float32MultiArray * msg =
        (const std_msgs__msg__Float32MultiArray *)msgin;

    if (msg->data.size >= 3) {
        target_velocities[0] = msg->data.data[0];
        target_velocities[1] = msg->data.data[1];
        target_velocities[2] = msg->data.data[2];
    }
}


void joint_state_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer != NULL) {
        uint32_t tick = HAL_GetTick();
        joint_state_msg.header.stamp.sec = tick / 1000;
        joint_state_msg.header.stamp.nanosec = (tick % 1000) * 1000000;

        const float COUNTS_TO_RAD = 2.0f * (float)M_PI / 16384.0f;
		joint_state_msg.position.data[0] = (float)enc[0].accum_counts * COUNTS_TO_RAD;
		joint_state_msg.position.data[1] = (float)enc[1].accum_counts * COUNTS_TO_RAD;
		joint_state_msg.position.data[2] = (float)enc[2].accum_counts * COUNTS_TO_RAD;

		// Velocity in rad/s
		joint_state_msg.velocity.data[0] = enc[0].velocity_rads;
		joint_state_msg.velocity.data[1] = enc[1].velocity_rads;
		joint_state_msg.velocity.data[2] = enc[2].velocity_rads;

		joint_state_msg.effort.data[0] = sqrtf(
		    (motor_current[0][0]*motor_current[0][0] +
		     motor_current[0][1]*motor_current[0][1] +
		     motor_current[0][2]*motor_current[0][2]) / 3.0f);

		joint_state_msg.effort.data[1] = sqrtf(
		    (motor_current[1][0]*motor_current[1][0] +
		     motor_current[1][1]*motor_current[1][1] +
		     motor_current[1][2]*motor_current[1][2]) / 3.0f);

		joint_state_msg.effort.data[2] = sqrtf(
		    (motor_current[2][0]*motor_current[2][0] +
		     motor_current[2][1]*motor_current[2][1] +
		     motor_current[2][2]*motor_current[2][2]) / 3.0f);

		rcl_ret_t ret = rcl_publish(&joint_state_pub, &joint_state_msg, NULL);
		(void)ret;
    }
}



void diagnostics_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void)timer;
    (void)last_call_time;

    uint32_t uptime_ms = HAL_GetTick();
    uint16_t fault1 = 0, fault2 = 0, fault3 = 0;
    DRV8353_ReadRegister(&drv_motor1, DRV8353_REG_FAULT_STATUS_1, &fault1);
    DRV8353_ReadRegister(&drv_motor2, DRV8353_REG_FAULT_STATUS_1, &fault2);
    DRV8353_ReadRegister(&drv_motor3, DRV8353_REG_FAULT_STATUS_1, &fault3);

    uint16_t motor_errors = fault1 | fault2 | fault3;
    diagnostics_msg.status.data[0].level = (motor_errors & DRV8353_FAULT) ? 2 : 0;
    // Stop motors if any fault detected
    if (motor_errors & DRV8353_FAULT) {
        target_velocities[0] = 0.0f;
        target_velocities[1] = 0.0f;
        target_velocities[2] = 0.0f;
    }

    char buffer[32];
    snprintf(buffer, sizeof(buffer), "0x%04X", motor_errors);
    rosidl_runtime_c__String__assign(&diagnostics_msg.status.data[0].values.data[0].value, buffer);

    snprintf(buffer, sizeof(buffer), "%lu", uptime_ms);
    rosidl_runtime_c__String__assign(&diagnostics_msg.status.data[0].values.data[1].value, buffer);


    rosidl_runtime_c__String__assign(&diagnostics_msg.status.data[0].message, g_debug_msg);


    uint32_t tick = HAL_GetTick();
    diagnostics_msg.header.stamp.sec = tick / 1000;
    diagnostics_msg.header.stamp.nanosec = (tick % 1000) * 1000000;

    rcl_ret_t ret = rcl_publish(&diagnostics_pub, &diagnostics_msg, NULL);
    (void)ret;
}

/* USER CODE END Application */

