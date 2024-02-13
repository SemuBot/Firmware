#include <Arduino.h>
#include <SPI.h>
#include "encoders.h"
#include "StepperMotors.h"
#include "variables.h"


Motor motor1 = {MOTOR1_DIR, MOTOR1_STEP_PIN, MOTOR1_EN_PIN, 10, 0, false};
bool debug = false;
int previousEncoder;

int test = 0;


void setup() {
  Serial.begin(baudRate);
  initMotor(motor1);
  encoderSetup();
  encoderUpdate();
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
  encoderUpdate();
  //Serial.println(motor1.steps);
  //Serial.print("Encoder:");
  //Serial.println(encoderPosition);
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





