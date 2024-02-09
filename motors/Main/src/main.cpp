#include <Arduino.h>
#include <SPI.h>
#include "encoders.h"
#include "StepperMotors.h"
#include "variables.h"

Motor motor1 = {MOTOR1_DIR, MOTOR1_STEP_PIN, MOTOR1_EN_PIN, 100, 0, false};

void setup() {
  Serial.begin(baudRate);
  initMotor(motor1);
  encoderSetup();
}


void loop() {
  //encoderUpdate();
  moveMotor(motor1);

 if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command.startsWith("up")) {
      int stepsToMove = command.substring(3).toInt();
      Serial.println(stepsToMove);
      //int speedToSet = command.substring(command.lastIndexOf('_') + 1).toInt();
      set_Steps(motor1,stepsToMove);
      startMotor(motor1);
    } else if (command.startsWith("down")) {

      int stepsToMove = command.substring(5).toInt();
      //int speedToSet = command.substring(command.lastIndexOf('_') + 1).toInt();
      // Move the motor down
      set_Steps(motor1,-stepsToMove);
      startMotor(motor1);
    } else if (command.startsWith("stop")) {
      Serial.println("STOPPING");
      stopMotor(motor1);
    }
  }
}





