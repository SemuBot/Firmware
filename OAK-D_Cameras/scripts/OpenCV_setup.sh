#!/bin/bash

sudo apt install git -y
sudo wget -qO- https://docs.luxonis.com/install_depthai.sh | bash

cd depthai
sudo apt-get update
sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt-get install python3.9 -y
python3 depthai_demo.py
