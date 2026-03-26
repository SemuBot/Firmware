## Variant 1 — ros2_control + serial

**Firmware branch:** `fw/ros2ctrl-serial`

**Description:** ros2_control runs on the host computer and handles PID and inverse kinematics. Commands are sent to the STM32 over a USB CDC serial protocol.

### Flash firmware

```bash
cd ~/repos/SemuBot-Firmware
git checkout fw/ros2ctrl-serial
```

Flash `wheelbase/Debug/wheelbase_main.elf` using STM32CubeIDE 

### Launch

```bash
# Terminal 1 - ros2_control stack
ros2 launch semubot_ros_control semubot_control.launch.py

# Terminal 2 - serial driver in ros2_control mode
ros2 run semubot_driver driver_node --ros-args -p mode:=ros2_control
```

