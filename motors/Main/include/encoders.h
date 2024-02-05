#ifndef ENCODERS_H
#define ENCODERS_H
#include <Arduino.h>


#define NOP __asm__ __volatile__ ("nop\n\t")

/* Define special ascii characters */
#define carriageReturn  0x0D
#define newLine         0x0A
#define tab             0x09

#define OFF             0
#define ON              1

/* pins */
#define SSI_CS          5 // Brown
#define SSI_SCK         3 // Orange
#define SSI_SDO         2 // green
                        // Blue is 5V
                        // Red is GND
#define res12           12 //resolution is 12 bits
#define baudRate        115200

#define ENCODER_LIMIT 1900 //The middle position, when arm is horisontally to the ground, value is 4000. ~90degrees up and down equals to 1900.

extern uint16_t encoderPosition; // holder for encoder position

void encoderSetup();
void encoderUpdate();
uint16_t getPositionSSI(uint8_t resolution);
uint16_t getPositionSSI_efficient(uint8_t resolution);
uint16_t getEncoderPosition();

#endif // ENCODERS_H