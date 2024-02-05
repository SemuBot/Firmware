#include <SPI.h>
#include <TMC26XStepper.h> //NB Install the library first

int speed = 20;
int start = 0;
int initialPosition = 0;

int CS = 6;
int dir = 4;
int stepPin = 5;
int stepsPerRotation = 200;
int current = 800;
int delayValue = 500;

int speed =  0;
int maxSpeed = 1000;

TMC26XStepper tmc26XStepper = TMC26XStepper(stepsPerRotation,CS,dir,stepPin,current);


void setup() {
  Serial.begin(115200);
    // Set up the stepper driver
  tmc26XStepper.setSpreadCycleChopper(2, 24, 8, 6, 0);
  tmc26XStepper.setRandomOffTime(0);
  tmc26XStepper.SPI_setCoilCurrent(100);
  tmc26XStepper.setMicrosteps(128);
  tmc26XStepper.setStallGuardThreshold(4, 0);
  Serial.println("Config finished, starting");

}
void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    // Process the received command
    if (command.startsWith("up")) {
      // Extract the steps and speed numbers from the command
      int stepsToMove = command.substring(3, command.lastIndexOf('_')).toInt();
      int speedToSet = command.substring(command.lastIndexOf('_') + 1).toInt();

      // Move the motor up
      Serial.print("Moving up");
      up(stepsToMove,speedToSet);
    } else if (command.startsWith("down")) {
      // Extract the steps number from the command
      int stepsToMove = command.substring(5).toInt();
      int speedToSet = command.substring(command.lastIndexOf('_') + 1).toInt();
      // Move the motor down
      Serial.println("Moving down");
      down(stepsToMove,speedToSet);
    }
  }
}

void down(int steps, int speedToSet) {
  
    tmc26XStepper.setSpeed(speedToSet);
    tmc26XStepper.step(10*speedToSet);
    tmc26XStepper.move();
    Serial.println("Moving down.");
}

void up(int steps, int speedToSet) {
    tmc26XStepper.SPI_setSpeed(speedToSet);
    tmc26XStepper.SPI_step(steps); 
    tmc26XStepper.spi_start();
}