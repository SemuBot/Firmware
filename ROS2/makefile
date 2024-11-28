ROS2_WS = ./ros_ws
ROS2_DISTRO = jazzy
ROS2_INSTALL = /opt/ros/$(ROS2_DISTRO)
BUILD_DIR = $(ROS2_WS)/build
INSTALL_DIR = $(ROS2_WS)/install
LOG_DIR = $(ROS2_WS)/log
BUILD_CMD = colcon build
SOURCE_CMD = source $(ROS2_WS)/install/setup.bash


PUBLISHER_CMD = ros2 run semubot cmd_vel_publisher
SERIAL_CMD = ros2 run semubot_cmd_vel_serial

.PHONY: all publisher serial clean

all: publisher serial

publisher:
	@echo "Building ROS 2 workspace..."
	@source /opt/ros/$(ROS2_DISTRO)/setup.bash && $(BUILD_CMD)
	@echo "Sourcing ROS 2 environment..."
	@source /opt/ros/$(ROS2_DISTRO)/setup.bash && $(SOURCE_CMD)
	@echo "Running publisher..."
	@source $(ROS2_WS)/install/setup.bash && $(PUBLISHER_CMD)

serial:
	@echo "Building ROS 2 workspace..."
	@source /opt/ros/$(ROS2_DISTRO)/setup.bash && $(BUILD_CMD)
	@echo "Sourcing ROS 2 environment..."
	@source /opt/ros/$(ROS2_DISTRO)/setup.bash && $(SOURCE_CMD)
	@echo "Running serial..."
	@source $(ROS2_WS)/install/setup.bash && $(SERIAL_CMD)

clean:
	@echo "Cleaning ROS 2 workspace..."
	@rm -rf $(BUILD_DIR) $(INSTALL_DIR) $(LOG_DIR)

