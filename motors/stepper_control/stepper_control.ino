#include <SPI.h>
#include <TMC26XStepper.h> //NB Install the library first

int start = 0;
int initialPosition = 0;

int CS = 6;
int dir = 7;
int stepPin = 8;
int stepsPerRotation = 800;
int current = 800;
int delayValue = 500;
int curr_step;
int speed =  1000;
int speedDirection = 100;
int maxSpeed = 1000;

TMC26XStepper tmc26XStepper = TMC26XStepper(stepsPerRotation,CS,dir,stepPin,current);


void setup() {

  Serial.begin(115200);
    // Set up the stepper driver
  tmc26XStepper.setSpreadCycleChopper(2, 24, 8, 6, 0);
  tmc26XStepper.setRandomOffTime(0);
  tmc26XStepper.SPI_setCoilCurrent(50);
  //tmc26XStepper.setMicrosteps(248);
  tmc26XStepper.setStallGuardThreshold(4, 0);
  Serial.println("Config finished, starting");
}
// down(50,3);
// up(50,3);
void loop() {
  up(50,50);
  while (tmc26XStepper.isMoving()){
    tmc26XStepper.getCurrentStallGuardReading();
    tmc26XStepper.move();
  }
  Serial.println("Moving");
  delay(2000);
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
      while (tmc26XStepper.isMoving()){
        tmc26XStepper.getCurrentStallGuardReading();
        tmc26XStepper.move();
      }
    } else if (command.startsWith("down")) {
      // Extract the steps number from the command
      int stepsToMove = command.substring(5).toInt();
      int speedToSet = command.substring(command.lastIndexOf('_') + 1).toInt();
      // Move the motor down
      Serial.println("Moving down");
      down(stepsToMove,speedToSet);
      while (tmc26XStepper.isMoving()){
        tmc26XStepper.getCurrentStallGuardReading();
        tmc26XStepper.move();
      }
    }
  }
}



void down(int steps, int speedToSet) {
  if (!tmc26XStepper.isMoving()) {
    tmc26XStepper.setSpeed(speedToSet*10);
    // Moving the motor for a certain number of steps based on the speed
    Serial.print("Going ");
    Serial.print(10 * speed);
    Serial.println(" steps");
    tmc26XStepper.step(-1000 * steps);
  }
}
void up(int steps, int speedToSet) {
  if (!tmc26XStepper.isMoving()) {
    tmc26XStepper.setSpeed(speedToSet*10);
    // Moving the motor for a certain number of steps based on the speed
    Serial.print("Going ");
    Serial.print(10 * speed);
    Serial.println(" steps");
    tmc26XStepper.step(100);
  }
}

void test()