#include <SPI.h>
#include <TMC26XStepper.h>
int cs1 = 35
int cs2 = 37
// Stepper motor 1 configuration
TMC26XStepper stepper1 = TMC26XStepper(200, cs1, 4, 5, 800);

// Stepper motor 2 configuration
TMC26XStepper stepper2 = TMC26XStepper(200, cs2, 7, 8, 800); // Adjust pin numbers as needed

void setup() {
    Serial.begin(9600);
    Serial.println("==============================");
    Serial.println("TMC26X Stepper Driver Demo App");
    Serial.println("==============================");

    // Stepper 1 configuration
    Serial.println("Configuring stepper driver 1");
    stepper1.setSpreadCycleChopper(2, 24, 8, 6, 0);
    stepper1.setRandomOffTime(0);
    stepper1.SPI_setCoilCurrent(100);
    stepper1.setMicrosteps(128);
    stepper1.setStallGuardThreshold(4, 0);

    // Stepper 2 configuration
    Serial.println("Configuring stepper driver 2");
    stepper2.setSpreadCycleChopper(2, 24, 8, 6, 0);
    stepper2.setRandomOffTime(0);
    stepper2.SPI_setCoilCurrent(100);
    stepper2.setMicrosteps(128);
    stepper2.setStallGuardThreshold(4, 0);

    Serial.println("Stepper 1 started");
    stepper1.SPI_setSpeed(80);
    stepper1.SPI_step(-200);
    stepper1.spi_start();
    delay(2000);

    Serial.println("Stepper 2 started");
    stepper2.SPI_setSpeed(80);
    stepper2.SPI_step(-200);
    stepper2.spi_start();
    delay(2000);

}

void loop() {
    Serial.println("Stepper 1 started");
    stepper1.SPI_setSpeed(80);
    stepper1.SPI_step(-200);
    stepper1.spi_start();
    delay(2000);

    Serial.println("Stepper 1 started");
    stepper1.SPI_setSpeed(80);
    stepper1.SPI_step(200);
    stepper1.spi_start();
    delay(2000);


    Serial.println("Stepper 2 started");
    stepper2.SPI_setSpeed(80);
    stepper2.SPI_step(-200);
    stepper2.spi_start();
    delay(2000);

    Serial.println("Stepper 2 started");
    stepper2.SPI_setSpeed(80);
    stepper2.SPI_step(200);
    stepper2.spi_start();
    delay(2000);

}
