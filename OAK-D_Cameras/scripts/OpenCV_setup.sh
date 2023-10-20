#!/bin/bash

sudo apt install git -y
git clone https://github.com/luxonis/depthai.git "SemuBot-Electronics/OAK-D-Cameras"

cd
cd SemuBot-Electronics/OAK-D-Cameras/depthai
sudo apt-get update -y
sudo yes "" | sudo add-apt-repository ppa:deadsnakes/ppa
sudo apt -y install python3.10
sudo apt -y install python3.10-venv
python3 depthai_demo.py
