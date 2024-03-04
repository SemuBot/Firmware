#ifndef VARIABLES_H
#define VARIABLES_H


/* Stepper driver pins*/
// Motor 1

#define MOTOR1_STEP_PIN 2
#define MOTOR1_DIR 3
#define MOTOR1_EN_PIN 4

// Motor 2

#define MOTOR2_DIR 5
#define MOTOR2_STEP_PIN 6
#define MOTOR2_EN_PIN 7


// Motor 3

#define MOTOR3_DIR 8
#define MOTOR3_STEP_PIN 9
#define MOTOR3_EN_PIN 10


// Motor 4

#define MOTOR4_DIR 11
#define MOTOR4_STEP_PIN 12
#define MOTOR4_EN_PIN 13


// Motor 5

#define MOTOR5_DIR 22
#define MOTOR5_STEP_PIN 24
#define MOTOR5_EN_PIN 26



// Encoder CS pins, brown wires

#define ENCODER_CS_1 28
#define ENCODER_CS_2 30
#define ENCODER_CS_3 32
#define ENCODER_CS_4 34
#define ENCODER_CS_5 36


/* Encoder pins */
#define ENCODER_SCK 38 // Orange CLOCK
#define ENCODER_SDO 40 // green DATA
                        // Blue is 5V
                        // Red is GND

const int NUM_ENCODERS = 5;
const int NUM_MOTORS = 5;
struct Motor {
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