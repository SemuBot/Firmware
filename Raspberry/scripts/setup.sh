#!/bin/bash

sudo apt update
sudo apt install python3-pip -y


cd ..
mkdir environments && mkdir LedStrip && cd LedStrip
python3 -m venv LedStrip

source  LedStrip/bin/activate
pip install Flask
pip3 install adafruit-circuitpython-neonpixel
