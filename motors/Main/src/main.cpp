#include <Arduino.h>
#include <SPI.h>
#include "encoders.h"
#include "StepperMotors.h"


void setup() {
  Serial.begin(baudRate);
  encoderSetup();
  motorSetup();
  Serial.println("Config finished, starting");

}





void loop() {
  encoderLoop();
  Serial.println(getEncoderPosition());
  // Check for incoming serial data
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    // Process the received command
    if (command.startsWith("up")) {
      // Extract the steps and speed numbers from the command
      int stepsToMove = command.substring(3, command.lastIndexOf('_')).toInt();
      int speedToSet = command.substring(command.lastIndexOf('_') + 1).toInt();

      // Move the motor up
      Serial.print("Moving up");
      up(stepsToMove,speedToSet);
    } else if (command.startsWith("down")) {
      // Extract the steps number from the command
      int stepsToMove = command.substring(5).toInt();
      int speedToSet = command.substring(command.lastIndexOf('_') + 1).toInt();
      // Move the motor down
      Serial.println("Moving down");
      down(stepsToMove,speedToSet);
    }
  }
}





