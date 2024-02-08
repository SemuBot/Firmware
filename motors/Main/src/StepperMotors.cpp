#include "StepperMotors.h"
#include "TMC26XStepper.h"
#include "encoders.h"
#include <SPI.h>

TMC26XStepper tmc26XStepper = TMC26XStepper(STEPS_PER_ROTATION,CS,DIR,STEP_PIN,CURRENT);

uint16_t encoderValue = 0;

void motorSetup(){
  tmc26XStepper.setSpreadCycleChopper(2, 24, 8, 6, 0);
  tmc26XStepper.setRandomOffTime(0);
  //tmc26XStepper.SPI_setCoilCurrent(100);
  tmc26XStepper.setMicrosteps(128);
  tmc26XStepper.setStallGuardThreshold(4, 0);
  Serial.println("Config finished, starting");
}

void test(){
  tmc26XStepper.getCurrentStallGuardReading();
  tmc26XStepper.move();
  Serial.println("T");
}

char isMotorMoving(){
  return tmc26XStepper.isMoving();
}


void down(int steps, int speedToSet) {
  if (!tmc26XStepper.isMoving()) {
    tmc26XStepper.setSpeed(speedToSet*10);
    // Moving the motor for a certain number of steps based on the speed
    Serial.print("Going ");
    Serial.print(-steps);
    Serial.println(" steps");
    tmc26XStepper.step(-steps);
  }
}

void up(int steps, int speedToSet) {
  if (!tmc26XStepper.isMoving()) {
    tmc26XStepper.setSpeed(speedToSet*10);
    // Moving the motor for a certain number of steps based on the speed
    Serial.print("Going ");
    Serial.print(steps);
    Serial.println(" steps");
    tmc26XStepper.step(steps);
  }
}

void stop(){
  tmc26XStepper.stop();
}

