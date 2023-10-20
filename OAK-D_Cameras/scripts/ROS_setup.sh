#!/bin/bash

# Enable the Ubuntu Universe repository
sudo apt update
sudo apt install software-properties-common -y
echo "" | sudo add-apt-repository universe
sudo apt install git -y


# Add ROS2 GPG key and repository to the sources list
sudo apt update
sudo apt install curl -y
echo "" | sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update repository caches and upgrade the system
sudo apt update
sudo apt upgrade -y

# Install ROS packages
sudo apt install ros-humble-desktop -y
sudo apt install ros-humble-ros-base -y
sudo apt install ros-dev-tools -y

# Install udev rules and depthai-ros package
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo apt install ros-humble-depthai-ros -y

# Install Python dependencies
sudo apt install python3.10 -y
sudo apt install libopencv-dev -y
sudo apt install python3-rosdep -y
sudo rosdep init
rosdep update

# Clone example code repository and build
cd ..
mkdir -p camera/src
cd camera/src
git clone --branch humble https://github.com/luxonis/depthai-ros.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
source /opt/ros/humble/setup.bash
MAKEFLAGS="-j1 -l1" colcon build
source install/setup.bash

# Launch the example code
source install/setup.bash
ros2 launch depthai_examples stereo_inertial_node.launch.py