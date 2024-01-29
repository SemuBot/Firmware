#include <Arduino.h>
#include <SPI.h>

#include "encoders.h"
#include "StepperMotors.h"


int initialPosition = 0;


void setup() {
  encoderSetup();
  motorSetup();
  Serial.println(initialPosition);
  Serial.begin(baudRate);
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

        // Move the motor up
        up(stepsToMove,speedToSet);
      } else if (command.startsWith("down")) {
        // Extract the steps number from the command
        int stepsToMove = command.substring(5).toInt();
        int speedToSet = command.substring(command.lastIndexOf('_') + 1).toInt();
        // Move the motor down
        down(stepsToMove,speedToSet);
      }
    }
  }
}


