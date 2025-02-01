#!/bin/bash
controller_session="controller"
controller_script="/home/semubot/Software/Motion_Controller/motion_controller_w_matrix.py"
if screen -list | grep -q "$controller_session"; then
    echo "Screen session '$controller_session' is running. Doing nothing."
else
    echo "Screen session '$controller_session' is not running. Starting controller script...."
	/usr/bin/screen -dmS "$controller_session" /usr/bin/python3 "$controller_script"
	echo "Started "$controller_script" in a screen session named $controller_session."
fi

