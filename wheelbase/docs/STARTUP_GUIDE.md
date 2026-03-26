## Variant 4 — onboard PID + micro-ROS

**Firmware branch:** `fw/onboard-pid-microros`

### Flash firmware

```bash
cd ~/repos/SemuBot-Firmware
git checkout fw/onboard-pid-microros
git submodule update --init --recursive
cp wheelbase/colcon.meta wheelbase/micro_ros_stm32cubemx_utils/microros_static_library_ide/library_generation/colcon.meta
```

Flash `wheelbase/Debug/wheelbase_main.elf` using STM32CubeIDE.

### Launch

```bash
# Terminal 1 - micro-ROS agent
ros2 launch semubot_bringup agent.launch.py

# Terminal 2 - joystick
ros2 run joy joy_node

# Terminal 3 - teleop
ros2 run teleop_twist_joy teleop_node \
  --ros-args \
  --params-file ~/semubot_driver/config/joy_config.yaml
```


