#!/bin/bash

crontab -l > mycron_backup

sed -i '24 s/^/#/' mycron_backup

crontab mycron_backup
echo "Auto start has been disabled."

