#include <Arduino.h>
#include <SPI.h>
#include "encoders.h"
#include "motors.h"
#include "variables.h"


Motor elbowMotor = {MOTOR1_DIR, MOTOR1_STEP_PIN, MOTOR1_EN_PIN, 100, 0, false};
Motor shoulderMotor = {MOTOR2_DIR, MOTOR2_STEP_PIN, MOTOR2_EN_PIN, 100, 0, false};
Motor motor3 = {MOTOR3_DIR, MOTOR3_STEP_PIN, MOTOR3_EN_PIN, 100, 0, false};
Motor motor4 = {MOTOR4_DIR, MOTOR4_STEP_PIN, MOTOR4_EN_PIN, 100, 0, false};
Motor motor5 = {MOTOR5_DIR, MOTOR5_STEP_PIN, MOTOR5_EN_PIN, 100, 0, false};

Encoder encoder1 = {ENCODER_CS_1, 0, elbowMotor};
Encoder encoder2 = {ENCODER_CS_2, 0, shoulderMotor};
Encoder encoder3 = {ENCODER_CS_3,  0, motor3};
Encoder encoder4 = {ENCODER_CS_4, 0, motor4};
Encoder encoder5 = {ENCODER_CS_5, 0, motor5};

bool debug = true;

Encoder encoderList[NUM_ENCODERS];
Motor motorList[NUM_MOTORS];


void setup() {
  Serial.begin(baudRate);
  encoderList[0] = encoder1;
  encoderList[1] = encoder2;
  encoderList[2] = encoder3;
  encoderList[3] = encoder4;
  encoderList[4] = encoder5;
  motorList[0] = elbowMotor;
  motorList[1] = shoulderMotor;
  motorList[2] = motor3;
  motorList[3] = motor4;
  motorList[4] = motor5;
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

Motor* getMotorByName(const String& motorName) {
  if (motorName == "elbow") {
    return &elbowMotor;
  } else if (motorName == "shoulder") {
    return &shoulderMotor;
  } else if (motorName == "motor3") {
    return &motor3;
  } else if (motorName == "motor4") {
    return &motor4;
  } else if (motorName == "motor5") {
    return &motor5;
  } else {
    Serial.println(motorName);
    Serial.println("Invalid motor name");
    return nullptr;
  }
}
void loop() {
  encoderUpdate(encoderList);
  if (!debug) {
    encoderCheck(encoderList);
  }

  moveMotor(elbowMotor);
  moveMotor(shoulderMotor);
  moveMotor(motor3);  
  moveMotor(motor4);  
  moveMotor(motor5);  

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');

    if (command.startsWith("motor")) {
      String motorName = command.substring(command.indexOf('_') + 1, command.indexOf('_', command.indexOf('_') + 1));
      String action = command.substring(command.indexOf('_', command.indexOf('_') + 1) + 1, command.indexOf('_', command.indexOf('_', command.indexOf('_') + 1) + 1));
      int stepsToMove = command.substring(command.lastIndexOf('_') + 1).toInt();

      Serial.println("Motor: " + motorName + ", Action: " + action + ", Steps: " + String(stepsToMove));
      
      if (action == "CCW") {
        Motor* motor = getMotorByName(motorName);
        if (motor != nullptr) {
          Serial.println("Moving");
          set_Steps(*motor, stepsToMove);
          startMotor(*motor);
        }
      } else if (action == "CW") {
        Motor* motor = getMotorByName(motorName);
        if (motor != nullptr) {
          set_Steps(*motor, -stepsToMove);
          startMotor(*motor);
        }
      } else if (action == "stop") {
        Serial.println("STOPPING Motors");
        stopMotor(elbowMotor);
        stopMotor(shoulderMotor);
        stopMotor(motor3);
        stopMotor(motor4);
        stopMotor(motor5);
      }
    } else if (command.startsWith("all")){
      set_Steps(elbowMotor,5000);
      set_Steps(shoulderMotor,5000);
      set_Steps(motor3,5000);
      set_Steps(motor4,5000);
      set_Steps(motor5,5000);
      startMotor(elbowMotor);
      startMotor(shoulderMotor);
      startMotor(motor3);
      startMotor(motor4);
      startMotor(motor5);
    }
  }
}

