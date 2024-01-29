#include <Arduino.h>
#include <SPI.h>

#include "encoders.h"
#include "TMC26XStepper.h"

int initialPosition = 0;
int CS = 2;
int dir = 6;
int stepPin = 7;
int stepsPerRotation = 200;
int current = 2000;
int delayValue = 500;
int speed = 20;

TMC26XStepper tmc26XStepper = TMC26XStepper(stepsPerRotation, CS, dir, stepPin, current);

int upperLimit = 1000; // Set upper limit for encoder position
int lowerLimit = 0;    // Set lower limit for encoder position

void setup() {
  encoderSetup();
  tmc26XStepper.setSpreadCycleChopper(2, 24, 8, 6, 0);
  tmc26XStepper.setRandomOffTime(0);
  tmc26XStepper.SPI_setCoilCurrent(200);
  tmc26XStepper.setMicrosteps(50);
  tmc26XStepper.setStallGuardThreshold(4, 0);
  Serial.println("Config finished, starting");
  initialPosition = tmc26XStepper.getMotorPosition();
  Serial.println(initialPosition);
  Serial.begin(baudRate);
}

void down(int steps) {
    for (int i = 0; i < steps; i++){
      int encoderValue = getEncoderPosition();
      if (encoderValue < upperLimit && encoderValue > lowerLimit){
        tmc26XStepper.SPI_setSpeed(speed);
        tmc26XStepper.SPI_step(-steps); 
        tmc26XStepper.spi_start();
        delay(delayValue);
      } else {
        Serial.println("Maximum");

      }
    }
}

void up(int steps, int speedToSet) {
    for (int i = 0; i < steps; i++){
      int encoderValue = getEncoderPosition();
      Serial.println(encoderValue);
      if (encoderValue < upperLimit && encoderValue > lowerLimit){
        tmc26XStepper.SPI_setSpeed(speedToSet);
        tmc26XStepper.SPI_step(steps); 
        tmc26XStepper.spi_start();
        delay(delayValue);
      } else{
        Serial.println("Maximum");
      }
    }
}

void loop() {
  while(1){
    encoderLoop();
    // Check for incoming serial data
    if (Serial.available() > 0) {
      String command = Serial.readStringUntil('\n');
      // Process the received command
      if (command.startsWith("up")) {
        // Extract the steps and speed numbers from the command
        int stepsToMove = command.substring(3, command.lastIndexOf('_')).toInt();
        int speedToSet = command.substring(command.lastIndexOf('_') + 1).toInt();

        // Set the motor speed
        tmc26XStepper.SPI_setSpeed(speedToSet);

        // Move the motor up
        up(stepsToMove,speedToSet);
      } else if (command.startsWith("down")) {
        // Extract the steps number from the command
        int stepsToMove = command.substring(5).toInt();
        // Move the motor down
        down(stepsToMove);
      }
    }
  }
}


