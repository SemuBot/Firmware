#!/bin/bash

sudo apt update
sudo apt install python3-pip -y

pip install Adafruit-Blinka
sudo pip3 install rpi_ws281x adafruit-circuitpython-neopixel -y #Installing necessary stuff for controlling the LED strip
sudo python3 -m pip install --force-reinstall adafruit-blinka -y

cd ..
mkdir flask
cd flask
python3 -m venv .venv

. venv/bin/activate
pip install Flask
cd ..

sudo apt install python3.11-venv
python -m venv env --system-site-packages
source env/bin/activate

cd ~
pip3 install --upgrade adafruit-python-shell
wget https://raw.githubusercontent.com/adafruit/Raspberry-Pi-Installer-Scripts/master/raspi-blinka.py
sudo -E env PATH=$PATH python3 raspi-blinka.py



pip install Flask
