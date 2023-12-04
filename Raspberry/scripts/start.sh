#!/bin/bash

cd /home/semubot/
. environments/LedStrip/bin/activate
cd Electronics/Raspberry/codes
screen -D -m -L bash -cc "python3 app.py"