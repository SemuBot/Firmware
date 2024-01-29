#include "StepperMotors.h"
#include "TMC26XStepper.h"
#include "encoders.h"

TMC26XStepper tmc26XStepper = TMC26XStepper(STEPS_PER_ROTATION, CS, DIR, STEP_PIN, CURRENT);

uint16_t encoderValue = 0;


void setSpeed(int speed){
    tmc26XStepper.SPI_setSpeed(speed);
}



void motorSetup(){
  tmc26XStepper.setSpreadCycleChopper(2, 24, 8, 6, 0);
  tmc26XStepper.setRandomOffTime(0);
  tmc26XStepper.SPI_setCoilCurrent(200);
  tmc26XStepper.setMicrosteps(50);
  tmc26XStepper.setStallGuardThreshold(4, 0);
  Serial.println("Motor Config finished, starting");
  initialPosition = tmc26XStepper.getMotorPosition();
}

void down(int steps, int speedToSet) {
    for (int i = 0; i < steps; i++){
    encoderValue = getEncoderPosition();
      if (encoderValue < UPPER_LIMIT && encoderValue > LOWER_LIMIT){
        tmc26XStepper.SPI_setSpeed(speedToSet);
        tmc26XStepper.SPI_step(-steps); 
        tmc26XStepper.spi_start();
        delay(DELAY_VALUE);
      } else {
        Serial.println("Maximum");

      }
    }
}

void up(int steps, int speedToSet) {
    for (int i = 0; i < steps; i++){
    encoderValue = getEncoderPosition();
      Serial.println(encoderValue);
      if (encoderValue < UPPER_LIMIT && encoderValue > LOWER_LIMIT){
        tmc26XStepper.SPI_setSpeed(speedToSet);
        tmc26XStepper.SPI_step(steps); 
        tmc26XStepper.spi_start();
        delay(DELAY_VALUE);
      } else{
        Serial.println("Maximum");
      }
    }
}