#include <Arduino.h>
#include <SPI.h>
#include "encoders.h"
#include "StepperMotors.h"

void setup() {
  Serial.begin(baudRate);
  encoderSetup();
  motorSetup();
  Serial.println("Encoder setup Config finished, starting");

}


void loop() {
  encoderUpdate();
  if ((encoderPosition - 100) > ENCODER_LIMIT){
    if (isMotorMoving()){
      stop();
    }
  }
  up(300,50);
  delay(1000);
  down(300,50);
  //encoderValue = getEncoderPosition();
  //Serial.print("Value: ");
  //Serial.println(encoderValue);

  // Check for incoming serial data
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    // Process the received command
    if (command.startsWith("up")) {
      // Extract the steps and speed numbers from the command
      int stepsToMove = command.substring(3, command.lastIndexOf('_')).toInt();
      int speedToSet = command.substring(command.lastIndexOf('_') + 1).toInt();
      if (encoderPosition > ENCODER_LIMIT) {
        up(stepsToMove,speedToSet);
      }
    } else if (command.startsWith("down")) {
      // Extract the steps number from the command
      int stepsToMove = command.substring(5).toInt();
      int speedToSet = command.substring(command.lastIndexOf('_') + 1).toInt();
      // Move the motor down
      if (encoderPosition > ENCODER_LIMIT) {
        down(stepsToMove,speedToSet);
      }
    }
  }
}





