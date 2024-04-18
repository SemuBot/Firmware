#include "motors.h"
#include "tim.h"


motor_st motor1 = {.dir_pin = MOTOR1_DIR_Pin,
	    .pul_pin = MOTOR1_PUL_Pin,
	    .en_pin = MOTOR1_EN_Pin,
		.en_port = MOTOR1_EN_GPIO_Port,
		.dir_port = MOTOR1_DIR_GPIO_Port,
		.pul_port = MOTOR1_PUL_GPIO_Port,
		.timer = &htim1,
		.speed = 0,
		.cmd_steps = 0,
		.steps_left = 0
};

motor_st motor2 = {.dir_pin = MOTOR2_DIR_Pin,
	    .pul_pin = MOTOR2_PUL_Pin,
	    .en_pin = MOTOR2_EN_Pin,
		.en_port = MOTOR2_EN_GPIO_Port,
		.dir_port = MOTOR2_DIR_GPIO_Port,
		.pul_port = MOTOR2_PUL_GPIO_Port,
		.timer = &htim2,
		.speed = 0,
		.cmd_steps = 0,
		.steps_left = 0
};

motor_st motor3 = {.dir_pin = MOTOR3_DIR_Pin,
	    .pul_pin = MOTOR3_PUL_Pin,
	    .en_pin = MOTOR3_EN_Pin,
		.en_port = MOTOR3_EN_GPIO_Port,
		.dir_port = MOTOR3_DIR_GPIO_Port,
		.pul_port = MOTOR3_PUL_GPIO_Port,
		.timer = &htim3,
		.speed = 0,
		.cmd_steps = 0,
		.steps_left = 0
};

motor_st motor4 = {.dir_pin = MOTOR4_DIR_Pin,
	    .pul_pin = MOTOR4_PUL_Pin,
	    .en_pin = MOTOR4_EN_Pin,
		.en_port = MOTOR4_EN_GPIO_Port,
		.dir_port = MOTOR4_DIR_GPIO_Port,
		.pul_port = MOTOR4_PUL_GPIO_Port,
		.timer = &htim4,
		.speed = 0,
		.cmd_steps = 0,
		.steps_left = 0
};

motor_st motor5 = {.dir_pin = MOTOR5_DIR_Pin,
	    .pul_pin = MOTOR5_PUL_Pin,
	    .en_pin = MOTOR5_EN_Pin,
		.en_port = MOTOR5_EN_GPIO_Port,
		.dir_port = MOTOR5_DIR_GPIO_Port,
		.pul_port = MOTOR5_PUL_GPIO_Port,
		.timer = &htim8,
		.speed = 0,
		.cmd_steps = 0,
		.steps_left = 0
};




void static MOTOR_set_one(motor_st* motor, int8_t speed, uint8_t steps){
	motor->steps_left = steps*MOTOR_STEPS_PER_REVOLUTION+1;

	HAL_GPIO_WritePin(motor->dir_port, motor->dir_pin, speed > 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
	speed = speed > 0 ? speed : -speed;
	motor->timer->Instance->PSC = 0x8000 >> speed; // Mingi LUT
	for (uint8_t i = 0; i < 254; i++){
		asm("nop");
	}
	if (motor->steps_left){
		HAL_GPIO_WritePin(motor->en_port, motor->en_pin, GPIO_PIN_SET);
		HAL_TIM_PWM_Start_IT(motor->timer, TIM_CHANNEL_1);
	}
}

void static MOTOR_update(motor_st* motor){
	if (motor->pulse_finished){
		if (motor->steps_left){
			motor->steps_left--;
			motor->timer->Instance->CCR1 = MOTOR_CCR1_START_VALUE;
		} else{
			motor->timer->Instance->CCR1 = 0;
			HAL_TIM_PWM_Stop_IT(motor->timer, TIM_CHANNEL_1);
		}
		motor->pulse_finished = false;
	}
}

void MOTOR_main(){
	MOTOR_update(&motor1);
	MOTOR_update(&motor2);
	MOTOR_update(&motor3);
	MOTOR_update(&motor4);
	MOTOR_update(&motor5);
}

void MOTOR_init(){
	HAL_TIM_PWM_Start_IT(motor1.timer, TIM_CHANNEL_1);
}

void MOTOR_set_all(usart_buffer_st* buf){
	MOTOR_set_one(&motor1, buf->m1_speed, buf->m1_steps);
	MOTOR_set_one(&motor2, buf->m2_speed, buf->m2_steps);
	MOTOR_set_one(&motor3, buf->m3_speed, buf->m3_steps);
	MOTOR_set_one(&motor4, buf->m4_speed, buf->m4_steps);
	MOTOR_set_one(&motor5, buf->m5_speed, buf->m5_steps);
}

void MOTOR_interrupt(TIM_HandleTypeDef* htim){
	if (htim == motor1.timer){
		motor1.pulse_finished = true;
	}
	if (htim == motor2.timer){
		motor2.pulse_finished = true;
	}
	if (htim == motor3.timer){
		motor3.pulse_finished = true;
	}
	if (htim == motor4.timer){
		motor4.pulse_finished = true;
	}
	if (htim == motor5.timer){
		motor5.pulse_finished = true;
	}

}

/*
void MOTOR_test(){
	MOTOR_set_one(&motor1, 10, 1);
}
*/
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef* htim){
	MOTOR_interrupt(htim);
}



/*
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
 motor5.is_high = false;*/
