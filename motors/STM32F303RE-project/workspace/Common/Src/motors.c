#include "motors.h"

void moveMotor(struct Motors* motor){

	if (motor->STEPS > 0){
		if(!motor->moving){

			HAL_GPIO_WritePin(motor->EN_PORT, motor->EN_PIN, GPIO_PIN_SET); // Set EN high to enable the driver
			motor->moving = true;
			HAL_Delay(1);
		}
	  motor->TIMER-> CCR1 = clamp(motor->SPEED,1,200);

	  // Wait for the specified duration
	  HAL_Delay(8000);

	  // Stop the motor
	  motor->TIMER-> CCR1 = 0;

	  // Wait for a moment
	  //HAL_Delay(MOVE_DURATION);


      //motor->STEPS -= (motor->STEPS > 0) ? 1 : -1;

	  motor->STEPS = 0;

      /*if(motor->STEPS == 0){
          HAL_GPIO_WritePin(motor->EN_PORT, motor->EN_PIN, GPIO_PIN_RESET);
          motor->moving = false;
      }*/
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




/*


void moveMotor(struct Motors* motor){

	if (motor->STEPS > 0){
		if(!motor->moving){

			HAL_GPIO_WritePin(motor->EN_PORT, motor->EN_PIN, GPIO_PIN_SET); // Set EN high to enable the driver
			motor->moving = true;
			HAL_Delay(1);
		}
	  motor->TIMER-> CCR1 = clamp(motor->SPEED,1,200);

	  // Wait for the specified duration
	  HAL_Delay(5000);

	  // Stop the motor
	  motor->TIMER-> CCR1 = 0;

	  // Wait for a moment
	  //HAL_Delay(MOVE_DURATION);


      //motor->STEPS -= (motor->STEPS > 0) ? 1 : -1;

	  motor->STEPS = 0;

      if(motor->STEPS == 0){
          HAL_GPIO_WritePin(motor->EN_PORT, motor->EN_PIN, GPIO_PIN_RESET);
          motor->moving = false;
      }
	}
}


 * */
