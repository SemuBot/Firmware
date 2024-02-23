#include <Arduino.h>
#include <SPI.h>
#include "encoders.h"
#include "motors.h"
#include "variables.h"


Motor motor1 = {MOTOR1_DIR, MOTOR1_STEP_PIN, MOTOR1_EN_PIN, 100, 0, false};
Motor motor2 = {MOTOR2_DIR, MOTOR2_STEP_PIN, MOTOR2_EN_PIN, 100, 0, false};
Encoder encoder1 = {ENCODER_CS_1, 0, 0};
Encoder encoder2 = {ENCODER_CS_2, 0, 0};
bool debug = true;
int previousEncoder;

int test = 0;
Encoder encoderList[NUM_ENCODERS];

void setup() {
  Serial.begin(baudRate);
  encoderList[0] = encoder1;
  encoderList[1] = encoder2;
  initMotor(motor1);
  initMotor(motor2);
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


void loop() {
  encoderUpdate(encoderList);
  //Serial.println(motor1.steps);
  //Serial.print("Encoder:");
  //Serial.println(encoderPosition);
  getEncoderPosition(encoderList);
  /*
  if (!debug){
    if ((encoderPosition < ENCODER_LIMIT_LOWER && encoderPosition > 800) || (encoderPosition > 700 && encoderPosition < 900)){
      if (encoderPosition < ENCODER_LIMIT_LOWER && encoderPosition > 800){
        if (motor1.steps > 0){
          stopMotor(motor1);
        }
      } else if (encoderPosition > 700 && encoderPosition < 900){
        if (motor1.steps < 0){
          stopMotor(motor1);
        }
      }

      //stopMotor(motor1);
      //set_Steps(motor1,-200);
    } 
  }
  */
  moveMotor(motor1);
  moveMotor(motor2);


 if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command.startsWith("up")) {
      int stepsToMove = command.substring(3).toInt();
      Serial.println(stepsToMove);
      //int speedToSet = command.substring(command.lastIndexOf('_') + 1).toInt();
      set_Steps(motor1,stepsToMove);
      set_Steps(motor2,stepsToMove);
      startMotor(motor1);
      startMotor(motor2);
    } else if (command.startsWith("down")) {

      int stepsToMove = command.substring(5).toInt();
      //int speedToSet = command.substring(command.lastIndexOf('_') + 1).toInt();
      // Move the motor down
      set_Steps(motor1,-stepsToMove);
      set_Steps(motor2,-stepsToMove);
      startMotor(motor1);
      startMotor(motor2);
    } else if (command.startsWith("stop")) {
      Serial.println("STOPPING");
      stopMotor(motor1);
      stopMotor(motor2);
    }
  }
}





