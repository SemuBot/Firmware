#!/bin/bash

sudo apt update
sudo apt install python3-pip -y


cd
mkdir environments && cd environments && mkdir LedStrip 
python3 -m venv LedStrip

#source  LedStrip/bin/activate # Have to do it manually
#pip install Flask
#pip3 install adafruit-circuitpython-neopixel

cd ..

#python3 ~/Electronics/Raspberry/codes app.py