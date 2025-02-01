#!/bin/bash

script_to_run="/home/semubot/Software/start_button/check_and_run.sh" 

i=0

while [ $i -lt 12 ]; do
	bash "$script_to_run" &
	sleep 5
	i=$(( i + 1))
done
