#include <Arduino.h>
#include <SPI.h>
#include "encoders.h"
#include "StepperMotors.h"

void encoder();

void setup() {
  Serial.begin(baudRate);
  encoderSetup();
  motorSetup();
  Serial.println("Encoder setup Config finished, starting");

}


void loop() {
  encoderUpdate();
  up(300,50,10);
  delay(1000);
  down(300,50,10);
  encoderValue = getEncoderPosition();
  Serial.print("Value: ");
  Serial.println(encoderValue);

  // Check for incoming serial data
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    // Process the received command
    if (command.startsWith("up")) {
      // Extract the steps and speed numbers from the command
      int stepsToMove = command.substring(3, command.lastIndexOf('_')).toInt();
      int speedToSet = command.substring(command.lastIndexOf('_') + 1).toInt();
      up(stepsToMove,speedToSet,encoderValue);
    } else if (command.startsWith("down")) {
      // Extract the steps number from the command
      int stepsToMove = command.substring(5).toInt();
      int speedToSet = command.substring(command.lastIndexOf('_') + 1).toInt();
      // Move the motor down
      down(stepsToMove,speedToSet,encoderValue);
    }
  }
}





