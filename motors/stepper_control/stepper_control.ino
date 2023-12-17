#include <SPI.h>
#include <TMC26XStepper.h> //NB Install the library first

int speed = 20;
int start = 0;
int initialPosition = 0;

int CS = 2;
int dir = 6;
int stepPin = 7;
int stepsPerRotation = 200;
int current = 2000;
int delayValue = 500;



TMC26XStepper tmc26XStepper = TMC26XStepper(stepsPerRotation, CS, dir, stepPin, current);

void setup() {
    Serial.begin(9600);

    // Set up the stepper driver
    tmc26XStepper.setSpreadCycleChopper(2, 24, 8, 6, 0);
    tmc26XStepper.setRandomOffTime(0);
    tmc26XStepper.SPI_setCoilCurrent(200);
    tmc26XStepper.setMicrosteps(50);
    tmc26XStepper.setStallGuardThreshold(4, 0);
    Serial.println("Config finished, starting");

    initialPosition = tmc26XStepper.getMotorPosition();
    Serial.println(initialPosition);
    updown();
}

void down() {
    int steps = 200; 
    tmc26XStepper.SPI_setSpeed(speed);
    tmc26XStepper.SPI_step(-500); 
    tmc26XStepper.spi_start();
    start -= steps; 
    delay(delayValue);
    Serial.println("Moving down.");
}

void up() {
    int steps = 200;
    tmc26XStepper.SPI_setSpeed(speed);
    tmc26XStepper.SPI_step(500);
    tmc26XStepper.spi_start();
    start += steps;
    delay(delayValue);
    Serial.println("Moving up.");
}
void startPosition(){
      int stepsBack = initialPosition - start;
     Serial.print("Step back: ");
     Serial.println(stepsBack);
    tmc26XStepper.SPI_setSpeed(speed);
    tmc26XStepper.SPI_step(stepsBack);
    tmc26XStepper.spi_start();
    start = initialPosition;

}


void updown(){
  up();
  down();
  down();
  up();
}

void loop() {
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
      
        if (input == "start") {
            startPosition();
        } else if (input == "up") {
            up();
        } else if (input == "down") {
            down();
        } else if (input == "updown"){
          updown();
    }else if (input.toInt() != 0) {
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
