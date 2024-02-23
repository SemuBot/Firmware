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

int test = 0;
Encoder encoderList[NUM_ENCODERS];

void setup() {
  Serial.begin(baudRate);
  encoderList[0] = encoder1;
  encoderList[1] = encoder2;
  initMotor(elbowMotor);
  initMotor(shoulderMotor);
  setupEncoders(encoderList);
  encoderUpdate(encoderList);
  /*
  set_Steps(motor1,200);

  previousEncoder = encoderPosition;
  while(encoderPosition < ENCODER_LIMIT){
    startMotor(motor1);
    Serial.println("HERE");
    while(true){
        encoderUpdate();
        if (test == 0){
          previousEncoder = encoderPosition;
          test++;
        }
        moveMotor(motor1);
        
        Serial.print("Encoderposition:");
        Serial.println(encoderPosition);
        Serial.print("Previousencoder: ");
        Serial.println(previousEncoder);

        if (encoderPosition < previousEncoder && test <50 && motor1.steps == 0){
          set_Steps(motor1,-400);
          Serial.println("Forward");
        } else if (encoderPosition > previousEncoder && test <50 && motor1.steps == 0) {
          set_Steps(motor1,400);
          Serial.println("Back");
        }
      if (encoderPosition > ENCODER_LIMIT) break;
    }
  }
  stopMotor(motor1);
  */
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
  //Serial.println(motor1.steps);
  //Serial.print("Encoder:");
  //Serial.println(encoderPosition);
  getEncoderPosition(encoderList);
  
  if (!debug){
      encoderCheck(encoderList);
  }
  
  moveMotor(elbowMotor);
  moveMotor(shoulderMotor);


 if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    
    if (command.startsWith("motor")) {
      String motorName = command.substring(5, command.indexOf('_')); // Extract motor name
      String action = command.substring(command.indexOf('_') + 1, command.indexOf('_', command.indexOf('_') + 1)); // Extract action (up/down)
      int stepsToMove = command.substring(command.lastIndexOf('_') + 1).toInt(); // Extract steps

      Serial.println("Motor: " + motorName + ", Action: " + action + ", Steps: " + String(stepsToMove));

      
      Motor motor = get_motor_name(motorName);

      if (motor.name != "") { 
        if (motor.name == "elbow") {
          if (action == "up") {
            set_Steps(motor, stepsToMove);
            startMotor(motor);
          } else if (action == "down") {
            set_Steps(motor, -stepsToMove);
            startMotor(motor);
          } else if (action == "stop") {
            Serial.println("STOPPING Motor");
            stopMotor(motor);
          }
        }
      }
    }
  }
}




