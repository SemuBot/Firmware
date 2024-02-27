#include <Arduino.h>
#include <SPI.h>
#include "encoders.h"
#include "motors.h"
#include "variables.h"


Motor elbowMotor = {"elbow",MOTOR1_DIR, MOTOR1_STEP_PIN, MOTOR1_EN_PIN, 100, 0, false};
Motor shoulderMotor = {"shoulder",MOTOR2_DIR, MOTOR2_STEP_PIN, MOTOR2_EN_PIN, 100, 0, false};
Encoder encoder1 = {ENCODER_CS_1, 0, elbowMotor};
Encoder encoder2 = {ENCODER_CS_2, 0, shoulderMotor};
bool debug = true;
int previousEncoder;

Encoder encoderList[NUM_ENCODERS];
Motor motorList[NUM_MOTORS];


void setup() {
  Serial.begin(baudRate);
  encoderList[0] = encoder1;
  encoderList[1] = encoder2;
  motorList[0] = elbowMotor;
  motorList[1] = shoulderMotor;
  setupMotors(motorList);
  setupEncoders(encoderList);
  encoderUpdate(encoderList);
  Serial.println("Setup finished!"), Serial.println("Starting");

}
void encoderCheck(Encoder* encoders){
    for (int i = 0; i < NUM_ENCODERS; i++){
      if ((encoders[i].data < ENCODER_LIMIT_LOWER && encoders[i].data > 800) || (encoders[i].data > 700 && encoders[i].data < 900)){
        if (encoders[i].data < ENCODER_LIMIT_LOWER && encoders[i].data > 800){
          if (encoders[i].motor.steps > 0){
            stopMotor(encoders[i].motor);
          }
        } else if (encoders[i].data > 700 && encoders[i].data < 900){
          if (encoders[i].motor.steps < 0){
            stopMotor(encoders[i].motor);
          }
        }
      } 
  }
}

Motor get_motor_name(const String &motorName) {
  if (motorName == "elbow") {
    return elbowMotor;
  } else if (motorName == "shoulder") {
    return shoulderMotor;
  }
}

void loop() {
  encoderUpdate(encoderList);
  if (!debug){
      encoderCheck(encoderList);
      Serial.println("Mitte siin");
  } else{
    getEncoderPosition(encoderList);
  }
  
  moveMotor(elbowMotor);
  moveMotor(shoulderMotor);
 if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    
    if (command.startsWith("motor")) {
      String motorName = command.substring(command.indexOf('_') + 1, command.indexOf('_', command.indexOf('_') + 1)); //command.substring(3, command.indexOf('_')); // Extract motor name
      String action = command.substring(command.indexOf('_', command.indexOf('_') + 1) + 1, command.indexOf('_', command.indexOf('_', command.indexOf('_') + 1) + 1));
      int stepsToMove = command.substring(command.lastIndexOf('_') + 1).toInt(); // Extract steps

      Serial.println("Motor: " + motorName + ", Action: " + action + ", Steps: " + String(stepsToMove));
      if (action == "up") {
        if (motorName == "elbow"){
          set_Steps(elbowMotor,stepsToMove);
          startMotor(elbowMotor);
        } else{
          set_Steps(shoulderMotor,stepsToMove);
          startMotor(shoulderMotor);
        }
      } else if (action == "down") {
        if (motorName == "shoulder") {
          set_Steps(shoulderMotor, -stepsToMove);
          startMotor(shoulderMotor);
        } else {
          set_Steps(elbowMotor, -stepsToMove);
          startMotor(elbowMotor);
        }

      } else if (action == "stop") {
        Serial.println("STOPPING Motors");
        stopMotor(elbowMotor);
        stopMotor(shoulderMotor);
      }
    }
  }
}




