#include <SPI.h>
#include <TMC26XStepper.h>

int speed = 200;
int start = 0;
int initialPosition = 0;
int CS = 2;
int dir = 6;
int stepPin = 7;
int stepsPerRotation = 200;
int current = 800;
TMC26XStepper tmc26XStepper = TMC26XStepper(stepsPerRotation, CS, dir, stepPin, current);

void setup() {
    Serial.begin(9600);

    // Set up the stepper driver
    tmc26XStepper.setSpreadCycleChopper(2, 24, 8, 6, 0);
    tmc26XStepper.setRandomOffTime(0);
    tmc26XStepper.SPI_setCoilCurrent(100);
    tmc26XStepper.setMicrosteps(128);
    tmc26XStepper.setStallGuardThreshold(4, 0);
    Serial.println("Config finished, starting");

    initialPosition = tmc26XStepper.getMotorPosition();
}

void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');

        if (input == "start") {
            int stepsBack = initialPosition - start;
            tmc26XStepper.SPI_setSpeed(speed);
            tmc26XStepper.SPI_step(-stepsBack);
            tmc26XStepper.spi_start();
            start = initialPosition;

            delay(500);

            Serial.println("Stepping back to start position.");
        } else if (input.toInt() != 0) {
            int steps = input.toInt();
            tmc26XStepper.SPI_setSpeed(speed);
            tmc26XStepper.SPI_step(steps);
            tmc26XStepper.spi_start();
            start += steps;

            delay(500);

            Serial.print("Stepping: ");
            Serial.println(steps);
        } else {
            Serial.println("Invalid input. Please enter a valid integer or 'start'.");
        }
    }
}
