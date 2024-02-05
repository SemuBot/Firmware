#include <SPI.h>
#include <TMC26XStepper.h> //NB Install the library first

int start = 0;
int initialPosition = 0;

int CS = 6;
int dir = 4;
int stepPin = 7;
int stepsPerRotation = 400;
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
  tmc26XStepper.SPI_setCoilCurrent(100);
  tmc26XStepper.setMicrosteps(128);
  tmc26XStepper.setStallGuardThreshold(4, 0);
  Serial.println("Config finished, starting");

}
/*
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
*/

void loop() {
  down(50,550);
  //int test = tmc26XStepper.getCurrentStallGuardReading();
  //up(50,50);
  Serial.print("Steps left: ");
  Serial.println(tmc26XStepper.getStepsLeft() % 100);
  while (tmc26XStepper.getStepsLeft() % 100 == 0) {
    //Serial.println("Stall Guard: ");
    int test = tmc26XStepper.getCurrentStallGuardReading();
    //Serial.println(tmc26XStepper.getCurrentStallGuardReading());
  }
  //tmc26XStepper.move();
  //delay(2000);
}
void down(int steps, int speedToSet) {
  if (!tmc26XStepper.isMoving()) {
    //speed += speedDirection;

    if (speed > maxSpeed) {
      speed = maxSpeed;
    }

    // Setting the speed
    Serial.print("Setting speed to ");
    Serial.println(speed);

    tmc26XStepper.setSpeed(speed);

    // Moving the motor for a certain number of steps based on the speed
    Serial.print("Going ");
    Serial.print(10 * speed);
    Serial.println(" steps");
    
    tmc26XStepper.step(10 * speed);
    tmc26XStepper.move();

  } else {
    // Output the status every 100 steps
  
    if (tmc26XStepper.getStepsLeft() % 100 == 0) {
      Serial.print("Stall Guard: ");
      Serial.println(tmc26XStepper.getCurrentStallGuardReading());
    }
  }

  tmc26XStepper.move();
}

void up(int steps, int speedToSet) {
  if (!tmc26XStepper.isMoving()) {
    //speed += speedDirection;

    if (speed > maxSpeed) {
      speed = maxSpeed;
    }

    // Setting the speed
    Serial.print("Setting speed to ");
    Serial.println(speed);

    tmc26XStepper.setSpeed(speed);

    // Moving the motor for a certain number of steps based on the speed
    Serial.print("Going ");
    Serial.print(10 * speed);
    Serial.println(" steps");
    
    tmc26XStepper.step(-10 * speed);
    tmc26XStepper.move();

  } else {
    // Output the status every 100 steps
    if (tmc26XStepper.getStepsLeft() % 100 == 0) {
      Serial.print("Stall Guard: ");
      Serial.println(tmc26XStepper.getCurrentStallGuardReading());
    }
  }

  tmc26XStepper.move();
}