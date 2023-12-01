#!/bin/bash

sudo apt update
sudo apt install python3-pip -y

sudo pip3 install rpi_ws281x adafruit-circuitpython-neopixel -y #Installind necessary stuff for controlling the LED strip
sudo python3 -m pip install --force-reinstall adafruit-blinka -y

pip install Flask -y
