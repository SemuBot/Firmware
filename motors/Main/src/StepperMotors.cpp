#include "StepperMotors.h"
#include "TMC26XStepper.h"
#include "encoders.h"
#include <SPI.h>

TMC26XStepper tmc26XStepper = TMC26XStepper(STEPS_PER_ROTATION, CS, DIR, STEP_PIN, CURRENT);

uint16_t encoderValue = 0;

void motorSetup(){
  tmc26XStepper.setSpreadCycleChopper(2, 24, 8, 6, 0);
  tmc26XStepper.setRandomOffTime(0);
  tmc26XStepper.SPI_setCoilCurrent(100);
  tmc26XStepper.setMicrosteps(128);
  tmc26XStepper.setStallGuardThreshold(4, 0);
  Serial.println("Motor Config finished, starting");
}


void setSpeed(int speed){
    tmc26XStepper.SPI_setSpeed(speed);
}

char isMotorMoving(){
  return tmc26XStepper.isMoving();
}

void down(int steps, int speedToSet) {
      tmc26XStepper.SPI_setSpeed(speedToSet);
      tmc26XStepper.SPI_step(-steps); 
      tmc26XStepper.spi_start();
      
}

void down2(int steps, int speedToSet){
    tmc26XStepper.setSpeed(speedToSet);
    tmc26XStepper.step(10*speedToSet);
    tmc26XStepper.move();
    Serial.println("Moving down.");
}
void up(int steps, int speedToSet) {
      tmc26XStepper.SPI_setSpeed(speedToSet);
      tmc26XStepper.SPI_step(steps); 
      tmc26XStepper.spi_start();
}

void stop(){
  tmc26XStepper.stop();
}

