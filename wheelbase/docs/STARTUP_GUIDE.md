## Variant 2 — ros2_control + micro-ROS

**Firmware branch:** `fw/ros2ctrl-microros`

**Description:** ros2_control runs on the host computer. The STM32 runs a micro-ROS node that subscribes to per-motor velocity commands and publishes joint states directly into the ROS 2 graph via the micro-ROS agent.

### Flash firmware

```bash
cd ~/repos/SemuBot-Firmware
git checkout fw/ros2ctrl-microros
git submodule update --init --recursive
cp wheelbase/colcon.meta wheelbase/micro_ros_stm32cubemx_utils/microros_static_library_ide/library_generation/colcon.meta
```

Flash `wheelbase/Debug/wheelbase_main.elf` using STM32CubeIDE.

### Launch

```bash
# Terminal 1 - micro-ROS agent
ros2 launch semubot_bringup agent.launch.py

# Terminal 2 - ros2_control stack (wait for agent to connect first)
ros2 launch semubot_ros_control semubot_control.launch.py
```

