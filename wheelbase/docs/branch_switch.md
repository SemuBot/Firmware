# Switching Branches with micro-ROS Submodule

## When switching to a micro-ROS branch (fw/ros2ctrl-microros, fw/onboard-pid-microros)


* This project needs a custom colcon.meta, so it needs to be copied every time when switching branches.

```bash
# 1. Switch branch
git checkout fw/ros2ctrl-microros

# 2. Populate submodule
git submodule update --init --recursive

# 3. Copy colcon.meta
cp wheelbase/colcon.meta wheelbase/micro_ros_stm32cubemx_utils/microros_static_library_ide/library_generation/colcon.meta

```
