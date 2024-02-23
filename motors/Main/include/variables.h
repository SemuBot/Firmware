#ifndef VARIABLES_H
#define VARIABLES_H


/* Stepper driver pins*/
// Motor 1

#define MOTOR1_STEP_PIN 11
#define MOTOR1_DIR 12
#define MOTOR1_EN_PIN 13

// Motor 2

#define MOTOR2_DIR 757
#define MOTOR2_STEP_PIN 6
#define MOTOR2_EN_PIN 77

// Encoders

#define ENCODER_CS_1 4 // Brown colour wire
#define ENCODER_CS_2 7


/* Encoder pins */
#define ENCODER_SCK         2 // Orange CLOCK
#define ENCODER_SDO         3 // green DATA
                        // Blue is 5V
                        // Red is GND

const int NUM_ENCODERS = 2;

struct Motor {
  String name;
  int dirPin;
  int stepPin;
  int enPin;
  int speed;
  int steps;
  bool moving;
};

struct Encoder {
  int cs_pin;
  uint16_t data;
  Motor motor;
};

#endif 