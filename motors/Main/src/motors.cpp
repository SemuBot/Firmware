#include "motors.h"
#include "encoders.h"


void initMotor(const Motor &motor){
  pinMode(motor.dirPin, OUTPUT);
  pinMode(motor.stepPin, OUTPUT);
}



void moveMotor(Motor &motor){
  if (motor.moving && motor.steps != 0){
    digitalWrite(motor.dirPin, (motor.steps > 0) ? HIGH : LOW);
    digitalWrite(motor.enPin, HIGH);
    digitalWrite(motor.stepPin, HIGH);
    delayMicroseconds(motor.speed);
    digitalWrite(motor.stepPin, LOW);
    delayMicroseconds(motor.speed);
    digitalWrite(motor.enPin, LOW);
    if (motor.steps > 0) {
      motor.steps--;
    } else {
      motor.steps++;
    }
  }else {
    set_Steps(motor, 0);
  }
}

void set_Steps(Motor &motor, int steps){
  motor.steps = steps;
}

void startMotor(Motor &motor){
  motor.moving = true;
}
void stopMotor(Motor &motor) {
  motor.moving = false;
 // digitalWrite(motor.enPin, LOW);
  set_Steps(motor,0);
}