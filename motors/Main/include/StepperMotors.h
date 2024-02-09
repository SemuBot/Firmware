#include <Arduino.h>
#include "variables.h"

#ifndef STEPPERMOTORS_H
#define STEPPERMOTORS_H

void initMotor(const Motor &motor);
void moveMotor(Motor &motor);
void set_Steps(Motor &motor, int steps);
void stopMotor(Motor &motor);
void startMotor(Motor &motor);


#endif // STEPPERMOTORS_H