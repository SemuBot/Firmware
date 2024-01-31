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
#define SSI_CS          1 // Brown
#define SSI_SCK         2 // Orange
#define SSI_SDO         3 // green
                        // Blue is 5V
                        // Red is GND
#define res12           12 //resolution is 12 bits
#define baudRate        115200

void encoderSetup();
void encoderLoop();
uint16_t getPositionSSI(uint8_t resolution);
uint16_t getPositionSSI_efficient(uint8_t resolution);
uint16_t getEncoderPosition();
#endif // ENCODERS_H