#include "motors.h"

void moveMotor(struct Motors* motor){

	if (motor->STEPS > 0){
		if(!motor->moving){
			HAL_GPIO_WritePin(motor->EN_PORT, motor->EN_PIN, GPIO_PIN_RESET); // Set EN high to enable the driver
			motor->moving = true;
		}
	  //HAL_GPIO_WritePin(GPIOC, motor->dirPin, GPIO_PIN_RESET); // Set DIR high for one direction
		/*
        if(motor->STEPS > 0){
            HAL_GPIO_WritePin(motor->DIR_PORT, motor->DIR_PIN, GPIO_PIN_SET); // Set DIR high for one direction
        } else {
            HAL_GPIO_WritePin(motor->DIR_PORT, motor->DIR_PIN, GPIO_PIN_RESET); // Set DIR low for another direction
        }
*/
	  motor->TIMER-> CCR1 = clamp(motor->SPEED,1,200);

	  // Wait for the specified duration
	  HAL_Delay(MOVE_DURATION);

	  // Stop the motor
	  motor->TIMER-> CCR1 = 0;

	  // Wait for a moment
	  HAL_Delay(MOVE_DURATION);


      motor->STEPS -= (motor->STEPS > 0) ? 1 : -1;

      if(motor->STEPS == 0){
          HAL_GPIO_WritePin(motor->EN_PORT, motor->EN_PIN, GPIO_PIN_SET);
      }
	}
}

// Function to clamp values for the duty cycle/ speed
int clamp(int value, int min, int max) {
    if (value < min) {
        return min;
    } else if (value > max) {
        return max;
    } else {
        return value;
    }
}
