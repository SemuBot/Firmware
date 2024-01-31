#include "StepperMotors.h"
#include "TMC26XStepper.h"
#include "encoders.h"
#include <SPI.h>

TMC26XStepper tmc26XStepper = TMC26XStepper(STEPS_PER_ROTATION, CS, DIR, STEP_PIN, CURRENT);

uint16_t encoderValue = 0;


void setSpeed(int speed){
    tmc26XStepper.SPI_setSpeed(speed);
}

int linear_map(int x, int in_min, int in_max, int out_min, int out_max) {
    return static_cast<int>((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

void motorSetup(){
  tmc26XStepper.setSpreadCycleChopper(2, 24, 8, 6, 0);
  tmc26XStepper.setRandomOffTime(0);
  tmc26XStepper.SPI_setCoilCurrent(100);
  tmc26XStepper.setMicrosteps(128);
  tmc26XStepper.setStallGuardThreshold(4, 0);
  Serial.println("Motor Config finished, starting");
}

void down(int steps, int speedToSet, int encoderValue2) {
    //int test = linear_map(encoderValue,0,100,0,4096);
    //if (encoderValue2 < UPPER_LIMIT){
      tmc26XStepper.SPI_setSpeed(speedToSet);
      tmc26XStepper.SPI_step(-steps); 
      tmc26XStepper.spi_start();
    //}
}

void up(int steps, int speedToSet, int encoderValue2) {
    //if (encoderValue2 < UPPER_LIMIT){
      tmc26XStepper.SPI_setSpeed(speedToSet);
      tmc26XStepper.SPI_step(steps); 
      tmc26XStepper.spi_start();
    //}

}

void stop(){
  tmc26XStepper.stop();
}

