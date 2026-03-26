## Variant 3 — onboard PID + serial

**Firmware branch:** `fw/onboard-pid-serial`

**Description:** PID and inverse kinematics run entirely on the STM32. The host sends `cmd_vel` body-frame velocity commands over plain USB CDC serial.

### Flash firmware

```bash
cd ~/repos/SemuBot-Firmware
git checkout fw/onboard-pid-serial
```

Flash `wheelbase/Debug/wheelbase_main.elf` using STM32CubeIDE.

### Launch

```bash
# Terminal 1 - serial driver + joystick teleop
ros2 launch semubot_driver semubot_driver.launch.py
```

