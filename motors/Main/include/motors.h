#include <Arduino.h>
#include "variables.h"

#ifndef MOTORS_H
#define MOTORS_H

void setupMotors(Motor* motors);
void moveMotor(Motor &motor);
void set_Steps(Motor &motor, int steps);
void stopMotor(Motor &motor);
void startMotor(Motor &motor);


#endif // MOTORS_H