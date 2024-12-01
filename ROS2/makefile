ROS2_WS = ./ros2_ws
ROS2_DISTRO = jazzy
ROS2_INSTALL = /opt/ros/$(ROS2_DISTRO)
BUILD_DIR = $(ROS2_WS)/build
INSTALL_DIR = $(ROS2_WS)/install
LOG_DIR = $(ROS2_WS)/log
BUILD_CMD = colcon build
SOURCE_CMD = source $(ROS2_WS)/install/setup.bash

PUBLISHER_CMD = ros2 run semubot cmd_vel_publisher
SERIAL_CMD = ros2 run semubot cmd_vel_serial

.PHONY: all publisher serial clean

all: publisher serial

publisher:
	@echo "Building ROS 2 workspace..."
	@bash -c "source /opt/ros/$(ROS2_DISTRO)/setup.bash && $(BUILD_CMD)"
	@echo "Sourcing ROS 2 environment..."
	@bash -c "$(SOURCE_CMD)"
	@echo "Running publisher..."
	@bash -c "$(SOURCE_CMD)" && $(PUBLISHER_CMD)

serial:
	@echo "Building ROS 2 workspace..."
	@bash -c "source /opt/ros/$(ROS2_DISTRO)/setup.bash && $(BUILD_CMD)"
	@echo "Sourcing ROS 2 environment..."
	@bash -c "$(SOURCE_CMD)"
	@echo "Running serial..."
	@bash -c "$(SOURCE_CMD)" && $(SERIAL_CMD)

clean:
	@echo "Cleaning ROS 2 workspace..."
	@rm -rf $(BUILD_DIR) $(INSTALL_DIR) $(LOG_DIR)

