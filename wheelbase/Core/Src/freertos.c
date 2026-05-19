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

/** Control loop period in milliseconds (200 Hz). */
#define CONTROL_LOOP_PERIOD_MS 5

/** JointState publish period in milliseconds (10 Hz). */
#define JOINT_STATE_PERIOD_MS 100

#define CMD_TIMEOUT_MS 		  1000

#define MAX_DUTY              0.50f
#define DUTY_DEADZONE         0.01f
#define NEG_PREKICK_DUTY      0.10f
#define NEG_PREKICK_MS        30

typedef struct {
    uint16_t prev_raw;
    int32_t  accum_counts;
    float    velocity_rads;
} encoder_state_t;

volatile float target_duty[3] = {0.0f, 0.0f, 0.0f};


static encoder_state_t enc[3] = {0};
volatile float debug_applied_duty[3] = {0.0f, 0.0f, 0.0f};
volatile float debug_target_duty[3] = {0.0f, 0.0f, 0.0f};
volatile uint32_t debug_ms_since_cmd = 0;
volatile uint8_t debug_timeout_active = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
void velocity_command_callback(const void * msgin);
void joint_state_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
//void diagnostics_timer_callback(rcl_timer_t * timer, int64_t last_call_time);
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
rcl_node_t node;
rclc_support_t support;
rcl_publisher_t joint_state_pub;
sensor_msgs__msg__JointState joint_state_msg;

rcl_subscription_t velocity_cmd_sub;
rclc_executor_t executor;
std_msgs__msg__Float32MultiArray velocity_cmd_msg;

// Timer handles
rcl_timer_t joint_state_timer;
//rcl_timer_t diagnostics_timer;

// Diagnostic messages
//diagnostic_msgs__msg__DiagnosticArray diagnostics_msg;
//char g_debug_msg[128] = "init";
volatile uint32_t last_cmd_time = 0;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId controlTaskHandle;
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */
extern motor_st motor1, motor2, motor3;
extern DRV8353_Handle drv_motor1, drv_motor2, drv_motor3;


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
    ret = rcl_timer_fini(&joint_state_timer);              (void)ret;
    ret = rclc_executor_fini(&executor);                   (void)ret;
    ret = rcl_node_fini(&node);                            (void)ret;
    rclc_support_fini(&support);
}


static void Motor_SetDuty(TIM_HandleTypeDef *htim, uint32_t channel,
                          GPIO_TypeDef *dir_port, uint16_t dir_pin,
                          GPIO_TypeDef *brake_port, uint16_t brake_pin,
                          float duty)
{
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(htim);

    if (fabsf(duty) < DUTY_DEADZONE) {
        __HAL_TIM_SET_COMPARE(htim, channel, 0);

        HAL_GPIO_WritePin(brake_port, brake_pin, GPIO_PIN_RESET);

        return;
    }

    float clamped = fminf(fabsf(duty), MAX_DUTY);

    HAL_GPIO_WritePin(dir_port, dir_pin,
        duty >= 0.0f ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_GPIO_WritePin(brake_port, brake_pin, GPIO_PIN_SET);

    __HAL_TIM_SET_COMPARE(htim, channel, (uint32_t)(clamped * arr));
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

        rclc_publisher_init_default(&joint_state_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "motor_states");
        rclc_timer_init_default2(&joint_state_timer, &support,
            RCL_MS_TO_NS(JOINT_STATE_PERIOD_MS), joint_state_timer_callback, true);
        executor = rclc_executor_get_zero_initialized_executor();
        rclc_executor_init(&executor, &support.context, 2, &allocator);

        rosidl_runtime_c__float__Sequence__init(&velocity_cmd_msg.data, 3);

        rclc_executor_add_subscription(&executor, &velocity_cmd_sub,
            &velocity_cmd_msg, &velocity_command_callback, ON_NEW_DATA);

        rclc_executor_add_timer(&executor, &joint_state_timer);

        //ping agent every 5 seconds
        last_cmd_time = HAL_GetTick();
        bool connected = true;

        while (connected) {
        	/*
            if (HAL_GetTick() - last_ping > 5000) {
                last_ping = HAL_GetTick();
                if (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
                    connected = false;
                }
            }
            */
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
            osDelay(1);
        }

        // Connection lost — stop motors and blink
        target_duty[0] = 0.0f;
        target_duty[1] = 0.0f;
        target_duty[2] = 0.0f;
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

    return (raw & 0x3FFF) >> 2 ;
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


    const float COUNTS_PER_REV = 4096.0f;
    const int ENCODER_CPR = 4096;
    const int ENCODER_HALF_CPR = 2048;
    const float TWO_PI_F = (2.0f * (float)M_PI);
    const float VELOCITY_FILTER_ALPHA = 0.2f;
    const float DT            = CONTROL_LOOP_PERIOD_MS / 1000.0f;

    // Prime encoders
    enc[0].prev_raw = Read_SSI(cs_enc_1_GPIO_Port, cs_enc_1_Pin, enc1_data_GPIO_Port, enc1_data_Pin);
    enc[1].prev_raw = Read_SSI(cs_enc_2_GPIO_Port, cs_enc_2_Pin, enc2_data_GPIO_Port, enc2_data_Pin);
    enc[2].prev_raw = Read_SSI(cs_enc_3_GPIO_Port, cs_enc_3_Pin, enc3_data_GPIO_Port, enc3_data_Pin);

    for (int i = 0; i < 3; i++) {
        enc[i].velocity_rads = 0.0f;
        enc[i].accum_counts = 0;
    }

    for (;;)
    {
    	static int startup_counter = 0;
    	startup_counter++;
        // --- Command timeout ---
    	debug_ms_since_cmd = HAL_GetTick() - last_cmd_time;
    	debug_timeout_active = debug_ms_since_cmd > CMD_TIMEOUT_MS;




        // --- Encoder reads ---
        uint16_t raw[3];
        raw[0] = Read_SSI(cs_enc_1_GPIO_Port, cs_enc_1_Pin, enc1_data_GPIO_Port, enc1_data_Pin);
        raw[1] = Read_SSI(cs_enc_2_GPIO_Port, cs_enc_2_Pin, enc2_data_GPIO_Port, enc2_data_Pin);
        raw[2] = Read_SSI(cs_enc_3_GPIO_Port, cs_enc_3_Pin, enc3_data_GPIO_Port, enc3_data_Pin);

        for (int i = 0; i < 3; i++)
        {
            int32_t delta = (int32_t)raw[i] - (int32_t)enc[i].prev_raw;

            // 12-bit wrap handling
            if (delta > ENCODER_HALF_CPR) {
                delta -= ENCODER_CPR;
            } else if (delta < -ENCODER_HALF_CPR) {
                delta += ENCODER_CPR;
            }
            // Discard large deltas, e.g. SPI glitches
            if (delta > 1000 || delta < -1000) {
                delta = 0;
            }
            if (delta >= -1 && delta <= 1) {
                delta = 0;
            }
            delta = -delta;


            enc[i].accum_counts += delta;
            enc[i].prev_raw = raw[i];

            float raw_vel = ((float)delta / COUNTS_PER_REV) * TWO_PI_F / DT;


            // Ignore first 100 cycles (~100ms)
            if (startup_counter < 100) {
                raw_vel = 0.0f;
            }
            if (fabsf(raw_vel) > 50.0f) {
                raw_vel = 0.0f;
            }
            // Low-pass filter velocity
            enc[i].velocity_rads =
                (1.0f - VELOCITY_FILTER_ALPHA) * enc[i].velocity_rads +
                VELOCITY_FILTER_ALPHA * raw_vel;
        }
        // --- Motor drive ---
        float mv1, mv2, mv3;

        taskENTER_CRITICAL();
        mv1 = target_duty[0];
        mv2 = target_duty[1];
        mv3 = target_duty[2];
        taskEXIT_CRITICAL();
        if (debug_timeout_active) {
            mv1 = 0.0f;
            mv2 = 0.0f;
            mv3 = 0.0f;
        }

        float mv[3] = {mv1, mv2, mv3};

        static float prev_cmd[3] = {0.0f, 0.0f, 0.0f};
        static uint8_t neg_prekick_active[3] = {0, 0, 0};
        static uint32_t neg_prekick_until[3] = {0, 0, 0};

        uint32_t now = HAL_GetTick();

        // When motor goes from rest to a negative duty cycle, it needs a litle kick. So a forward pulse is sent first and then negative
        for (int i = 0; i < 3; i++) {
            float commanded = mv[i];

            int was_stopped = fabsf(prev_cmd[i]) < DUTY_DEADZONE;
            int wants_negative = commanded < -DUTY_DEADZONE;

            if (was_stopped && wants_negative && !neg_prekick_active[i]) {
                neg_prekick_active[i] = 1;
                neg_prekick_until[i] = now + NEG_PREKICK_MS;
            }

            if (neg_prekick_active[i]) {
                if (now < neg_prekick_until[i]) {
                    mv[i] = NEG_PREKICK_DUTY;
                } else {
                    neg_prekick_active[i] = 0;
                    mv[i] = commanded;
                }
            }

            prev_cmd[i] = commanded;
        }

        mv1 = mv[0];
        mv2 = mv[1];
        mv3 = mv[2];
        debug_target_duty[0] = target_duty[0];
        debug_target_duty[1] = target_duty[1];
        debug_target_duty[2] = target_duty[2];

        debug_applied_duty[0] = mv1;
        debug_applied_duty[1] = mv2;
        debug_applied_duty[2] = mv3;

        Motor_SetDuty(&htim1, TIM_CHANNEL_1,
            motor1.dir_port, motor1.dir_pin,
            motor1_nBRAKE_GPIO_Port, motor1_nBRAKE_Pin,
            mv1);

        Motor_SetDuty(&htim3, TIM_CHANNEL_1,
            motor2.dir_port, motor2.dir_pin,
            motor2_nBRAKE_GPIO_Port, motor2_nBRAKE_Pin,
            mv2);

        Motor_SetDuty(&htim4, TIM_CHANNEL_1,
            motor3.dir_port, motor3.dir_pin,
            motor3_nBRAKE_GPIO_Port, motor3_nBRAKE_Pin,
            mv3);
        osDelay(CONTROL_LOOP_PERIOD_MS);
    }
}



/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void velocity_command_callback(const void * msgin)
{
    const std_msgs__msg__Float32MultiArray * msg =
        (const std_msgs__msg__Float32MultiArray *)msgin;

    if (msg->data.size >= 3) {
    	taskENTER_CRITICAL();
        target_duty[0] = msg->data.data[0];
        target_duty[1] = msg->data.data[1];
        target_duty[2] = msg->data.data[2];

        last_cmd_time=HAL_GetTick();

        taskEXIT_CRITICAL();
    }
}


void joint_state_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void)last_call_time;
    if (timer != NULL) {
    	uint16_t fault1 = 0;
    	uint16_t fault2 = 0;
    	uint16_t fault3 = 0;

    	DRV8353_ReadRegister(&drv_motor1, DRV8353_REG_FAULT_STATUS_1, &fault1);
    	DRV8353_ReadRegister(&drv_motor2, DRV8353_REG_FAULT_STATUS_1, &fault2);
    	DRV8353_ReadRegister(&drv_motor3, DRV8353_REG_FAULT_STATUS_1, &fault3);
        const float COUNTS_TO_RAD = 2.0f * (float)M_PI / 4096.0f;

        uint32_t tick = HAL_GetTick();
        joint_state_msg.header.stamp.sec = tick / 1000;
        joint_state_msg.header.stamp.nanosec = (tick % 1000) * 1000000;


        int32_t counts_now[3];
        float vel_now[3];

        taskENTER_CRITICAL();
        for (int i = 0; i < 3; i++) {
            counts_now[i] = enc[i].accum_counts;
            vel_now[i] = enc[i].velocity_rads;
        }
        taskEXIT_CRITICAL();


        for (int i = 0; i < 3; i++) {
            joint_state_msg.position.data[i] =
                (float)counts_now[i] * COUNTS_TO_RAD;

            if (fabsf(vel_now[i]) < 0.001f) {
                vel_now[i] = 0.0f;
            }


            joint_state_msg.velocity.data[i] = vel_now[i];
            joint_state_msg.effort.data[0] = (double)fault1;
            joint_state_msg.effort.data[1] = (double)fault2;
            joint_state_msg.effort.data[2] = (double)fault3;
        }

        rcl_ret_t ret = rcl_publish(&joint_state_pub, &joint_state_msg, NULL);
        (void)ret;
    }
}



/* USER CODE END Application */

