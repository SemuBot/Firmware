#!/bin/bash

sudo apt update
sudo apt upgrade

# Function to check if a command is available
check_command() {
    if command -v "$1" &>/dev/null; then
        echo "$1 is installed."
    else
        echo "$1 is not installed."
        return 1
    fi
}

# Check if Python is installed
check_command "python3" || {
    # Install Python
    echo "Installing Python..."
    sudo apt-get update
    sudo apt-get install -y python3
}

# Check if screen is installed
check_command "screen" || {
    # Install screen
    echo "Installing screen..."
    sudo apt-get install -y screen
}

# Check if pip is installed
check_command "pip3" || {
    # Install pip
    echo "Installing pip..."
    sudo apt-get install -y python3-pip
}

# Check RPi.GPIO installation
check_command "python3" && check_command "pip3" && {
    if python3 -c "import RPi.GPIO" &>/dev/null; then
        echo "RPi.GPIO is installed."
    else
        echo "Installing RPi.GPIO..."
        sudo pip3 install RPi.GPIO
    fi
}

cd
mkdir environments && cd environments && mkdir LedStrip 
python3 -m venv LedStrip
. environments/LedStrip/bin/activate
pip install flask && pip install Adafruit-Blinka && cd

#Add the start script to begin on reboot
reboot_command="@reboot sh /home/semubot/Electronics/Raspberry/scripts/start.sh" 

(crontab -l ; echo "$reboot_command") | crontab -


config_file="$HOME/.config/wayfire.ini"

# Config, what to add to the wayfire file. This is needed for chromium kiosk mode
wayfire_config="[autostart]
panel = wfrespawn wf-panel-pi
background = wfrespawn pcmanfm --desktop --profile LXDE-pi
chromium = chromium-browser \"127.0.0.1:5000/refresh_webpage\" --kiosk --noerrdialogs --disable-infobars --no-first-run --ozone-platform=wayland --enab>
xdg-autostart = lxsession-xdg-autostart
screensaver = false
dpms = false"

# Check if the config file already exists
if [ -f "$config_file" ]; then
    # Append the configuration to the file
    echo -e "$wayfire_config" >> "$config_file"
    echo "Configuration added to $config_file"
else
    # Create the file and add the configuration
    mkdir -p "$(dirname "$config_file")"
    echo -e "$wayfire_config" > "$config_file"
    echo "Configuration file created at $config_file"
fi