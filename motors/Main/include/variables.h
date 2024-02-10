#ifndef VARIABLES_H
#define VARIABLES_H


/* Stepper driver pins*/
// Motor 1
#define MOTOR1_DIR 11
#define MOTOR1_STEP_PIN 9
#define MOTOR1_EN_PIN 13

/* Encoder pins */
#define ENCODER_CS          2 // Brown
#define ENCODER_SCK         3 // Orange CLOCK
#define ENCODER_SDO         6 // green DATA
                        // Blue is 5V
                        // Red is GND

struct Motor {
  int dirPin;
  int stepPin;
  int enPin;
  int speed;
  int steps;
  bool moving;
};

#endif 