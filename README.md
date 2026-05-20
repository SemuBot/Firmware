# onboard-pid - micro-ROS

# Building and Flashing the SemuBot Wheelbase Firmware

This guide explains how to compile and upload the existing SemuBot wheelbase firmware that uses micro-ROS.

The firmware project is located in:

```text
SemuBot-Firmware/wheelbase
````

Example repository layout:

```text
SemuBot-Firmware/
├── README.md
└── wheelbase/
    ├── Core/
    ├── Drivers/
    ├── Middlewares/
    ├── USB_DEVICE/
    ├── Debug/
    ├── docs/
    ├── colcon.meta
    ├── micro_ros_stm32cubemx_utils/
    ├── STM32F303RETX_FLASH.ld
    ├── wheelbase_main.ioc
    └── wheelbase_main Debug.launch
```

The micro-ROS setup in this project is based on the official `micro_ros_stm32cubemx_utils` STM32CubeIDE workflow for ROS 2 Jazzy. The upstream guide documents the same Docker-based static library build step, include path, linker settings, required extra source files, and transport configuration. ([GitHub][1])

## Requirements

Install the following before building:

* STM32CubeIDE
* Docker
* Git
* An ST-LINK-compatible programmer/debugger
* A ROS 2 Jazzy environment on the host computer, if you want to run the micro-ROS Agent

Check that Docker is running:

```bash
docker --version
docker ps
```

If `docker ps` fails, start Docker before opening or building the STM32 project.

## 1. Clone the firmware repository

Clone the firmware repository and enter the project folder:

```bash
git clone <REPOSITORY_URL>
cd SemuBot-Firmware
```

Switch to the firmware branch that contains the micro-ROS wheelbase project:

```bash
git checkout fw/ros2ctrl-microros
```

Then enter the STM32CubeIDE project directory:

```bash
cd wheelbase
```

You should see the existing STM32 project files:

```bash
ls
```

Expected important files and folders include:

```text
Core
Drivers
Middlewares
USB_DEVICE
micro_ros_stm32cubemx_utils
colcon.meta
wheelbase_main.ioc
STM32F303RETX_FLASH.ld
```

## 2. Open the project in STM32CubeIDE

Open STM32CubeIDE.

Then import the existing project:

```text
File -> Import -> General -> Existing Projects into Workspace
```

Select the repository folder:

```text
SemuBot-Firmware/wheelbase
```

Make sure the `wheelbase` project is detected, then click **Finish**.

## 3. Confirm the micro-ROS pre-build step

The project should already contain the micro-ROS utility folder:

```text
wheelbase/micro_ros_stm32cubemx_utils
```

STM32CubeIDE must run the micro-ROS static library builder before compiling the firmware.

Open:

```text
Project -> Properties -> C/C++ Build -> Settings -> Build Steps
```

In **Pre-build steps**, the command should be:

```bash
docker pull microros/micro_ros_static_library_builder:jazzy && docker run --rm -v ${workspace_loc:/${ProjName}}:/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library_ide microros/micro_ros_static_library_builder:jazzy
```

This command builds the micro-ROS static library inside the project using Docker. The official micro-ROS STM32CubeIDE guide uses the same `microros/micro_ros_static_library_builder:jazzy` image and generates the library under `micro_ros_stm32cubemx_utils/microros_static_library_ide`. ([GitHub][1])

After the first successful build, the generated library should appear under:

```text
wheelbase/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros
```

## 4. Confirm the include path

Open:

```text
Project -> Properties -> C/C++ Build -> Settings -> Tool Settings
```

Then go to:

```text
MCU GCC Compiler -> Include paths
```

The project should include:

```text
micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros/include
```

This is required so the firmware can include micro-ROS headers. The same include path is listed in the official STM32CubeIDE integration instructions. ([GitHub][1])

## 5. Confirm the linker settings

Open:

```text
Project -> Properties -> C/C++ Build -> Settings -> Tool Settings
```

Then go to:

```text
MCU GCC Linker -> Libraries
```

Under **Library search path (-L)**, make sure this path is configured:

```text
<ABSOLUTE_PATH_TO_REPO>/SemuBot-Firmware/wheelbase/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros
```

For example:

```text
/home/medved/repos/SemuBot-Firmware/wheelbase/micro_ros_stm32cubemx_utils/microros_static_library_ide/libmicroros
```

Under **Libraries (-l)**, make sure this is added:

```text
microros
```

Do not write `libmicroros.a` in the `Libraries (-l)` field. The `-lmicroros` linker option automatically resolves to `libmicroros.a`.

## 6. Build the firmware

In STM32CubeIDE, select the `wheelbase` project and build it:

```text
Project -> Build Project
```

Alternatively, use the build button in the toolbar.

During the build, STM32CubeIDE will:

1. Run the Docker pre-build command.
2. Build or update the micro-ROS static library.
3. Compile the STM32 firmware.
4. Link the firmware with `libmicroros.a`.

A successful build should generate firmware output inside the build configuration folder, for example:

```text
wheelbase/Debug/
```

Typical output files include:

```text
wheelbase_main.elf
wheelbase_main.bin
wheelbase_main.hex
```

The exact generated files depend on the STM32CubeIDE build configuration.

## 7. Connect the board

Connect the SemuBot wheelbase STM32 board to the computer.

Make sure:

* The board is powered.
* The ST-LINK programmer/debugger is connected.
* STM32CubeIDE can detect the target.
* The correct debug configuration is selected.

This project already contains a launch configuration:

```text
wheelbase_main Debug.launch
```

## 8. Flash the firmware

In STM32CubeIDE, flash the firmware using:

```text
Run -> Run
```

After flashing, reset the board if the firmware does not start automatically.


## Reference

This project’s micro-ROS integration follows the official micro-ROS STM32CubeMX/IDE utilities guide for ROS 2 Jazzy:

[https://github.com/micro-ROS/micro_ros_stm32cubemx_utils/tree/jazzy](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils/tree/jazzy)

[1]: https://github.com/micro-ROS/micro_ros_stm32cubemx_utils/tree/jazzy "GitHub - micro-ROS/micro_ros_stm32cubemx_utils at jazzy · GitHub"
