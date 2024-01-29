# Stepper Motor Control System

This PlatformIO project provides a simple system for controlling a stepper motor using an Arduino and a DualSense controller.

## Table of Contents

- [Requirements](#requirements)
- [Setup](#setup)
  - [Arduino Setup](#arduino-setup)
  - [Python Setup](#python-setup)
- [Usage](#usage)

## Requirements

- Arduino board (e.g., Arduino Uno)
- DualSense controller
- Stepper motor
- Python 3.x installed on your computer
- [pydualsense](https://flok.github.io/pydualsense/) library installed for Python (if you have the DualSense controller)

## Setup

### Arduino Setup

1. Connect the stepper motor driver to the Arduino board.
2. Upload the provided Arduino code (`Main.cpp`) to your Arduino board, using PlatformIO. Guide how to get it on VSCode: [link](https://docs.platformio.org/en/stable/integration/ide/vscode.html).
3. Connecting the absolute encoder to the Arduino.

### Python Setup

1. Install the required Python libraries:
   ```bash
   pip install pydualsense
   ```
    2. Launch the controller.py, if you want to control the motors with DualSense controller.


## Usage

1. When you have uploaded the code to your Arduino Uno board, connected the Stepper Motor Driver to the Arduino, and also done the wiring for absolute encoder.
2. Check out what COM port your arduino is detected in your computer and change it in controller.py.
3. Run the controller.py. If you have DualSense controller, you can control the motor with L2 and R2. If you don, you have to send data in serial monitor, with format: 
   ```bash
   up_stepstomove_speed
   ```

