# ESP Micro Drone Code

This folder contains all the software of the micro-drone. It is split into two parts:

| Folder | What it is |
|--------|------------|
| [`esp_code/`](https://github.com/ruibartsegura/ESP-Micro_drone/tree/main/code/esp_code) | Firmware for the ESP32-S2 (ESP-IDF + micro-ROS), its host unit tests and some analysis tools. |
| [`ros_ws/`](https://github.com/ruibartsegura/ESP-Micro_drone/tree/main/code/ros_ws) | ROS 2 workspace with the custom messages (`my_msgs`) and the Gazebo simulation. |

## Index

- [How it works](#how-it-works)
  - [State machine](#state-machine) · [Flight control](#flight-control) · [ROS interface](#ros-interface) · [Simulation mode](#simulation-mode)
- [Requirements](#requirements)
- [Setup](#setup)
- [Build and flash the firmware](#build-and-flash-the-firmware)
- [Run](#run)
  - [Fly](#3-fly) · [Run in simulation](#run-in-simulation) · [Tune the controller](#tune-the-controller)
- [Tests](#tests)
- [Analysis tools](#analysis-tools)
- [How to add or change things](#how-to-add-or-change-things)
  - [Project structure](#project-structure) · [New driver or module](#add-a-new-driver-or-module) · [Configuration option](#add-a-configuration-option) · [Publisher, subscriber or service](#add-a-new-publisher-subscriber-or-service) · [Custom message](#add-a-custom-message-or-service) · [Controller](#change-the-controller)

## How it works

The firmware runs on an ESP32-S2 using FreeRTOS. Each part of the system is a separate task:

| Task | Job | Period | Priority |
|------|-----|--------|----------|
| `system_task` | Runs the state machine of the drone | 20 ms | 5 |
| `attitude_task` | Cascade PID controller, height control and motor mixing | 15 ms | 5 |
| `odom_task` | Odometry: orientation (Madgwick filter), velocity and position | 10 ms | 3 |
| `imu_task` | Reads the MPU-6050 (accelerometer + gyroscope) | 10 ms | 6 |
| `bmp180_task` | Reads the BMP180 barometer and computes the altitude | 10 ms | 5 |
| `uros_task` | micro-ROS executor: topics and services | event driven | 4 |
| `tuning_task` | Changes the controller gains from the serial console (optional) | 20 ms | 1 |

The priorities and stack sizes can be changed in `menuconfig`. In simulation mode `imu_task` and `bmp180_task` are not started: the sensor data comes from Gazebo.

All the tasks share the data (sensors, odometry and current state) through `global_data/state`, which protects every access with a critical section.

### State machine

```
INIT → CHECKING → ARMING → TAKING_OFF → HOVERING ↔ EXTERNAL_CONTROL → LANDING → DISARMING → ARMING
```

| State | What happens | LEDs |
|-------|--------------|------|
| `INIT` | Starts every module | |
| `CHECKING` | Self test of every module. The flight parameters can still be changed | green on if it passes |
| `ARMING` | Waits for the take-off service | solid green |
| `TAKING_OFF` | Goes up to the requested altitude | solid green + blue blinking |
| `HOVERING` | Holds the altitude without moving | solid green +  solid blue |
| `EXTERNAL_CONTROL` | Follows `drone/cmd_vel`. Goes back to `HOVERING` if no command arrives for 500 ms | green blinking + solid blue|
| `LANDING` | Goes down to the ground (after the land service) | solid green + red blinking |
| `DISARMING` | Stops the motors and goes back to `ARMING`, ready for a new flight |  |
| `ERROR` | Error state | solid red |

The ESP32 built-in LED turns on when the drone connects to the micro-ROS agent.

### Flight control

- **Outer loop:** the velocity command (`cmd_vel`) is converted into target roll and pitch angles (limited to 20°).
- **Inner loop:** the angle error gives a target angular rate, and the rate error gives the power of each axis. The power goes to the motors with an X-frame mix.
- **Height:** PD controller over a base throttle. The height is estimated with a complementary filter of the accelerometer and the barometer.
- The drone has reached a target height when it stays within 5 cm, with a vertical velocity under 0.05 m/s, for 500 ms.

### ROS interface

The drone talks to a ROS 2 computer over **WiFi (UDP)**. The ESP32 runs a single micro-ROS node called `rui_drone`, and a **micro-ROS agent** on the computer connects it to the rest of the ROS 2 network.

| Direction | Name | Type |
|-----------|------|------|
| Publish | `drone/state` | `my_msgs/msg/State` |
| Subscribe | `drone/cmd_vel` | `geometry_msgs/TwistStamped` |
| Subscribe | `drone/params` | `my_msgs/msg/Params` |
| Service | `drone/takeoff_srv` | `my_msgs/srv/Takeoff` |
| Service | `drone/land_srv` | `std_srvs/srv/Trigger` |

`drone/state` is published every second. It contains the position, orientation, linear and angular velocity, acceleration and the current state of the state machine, with SI units (m, m/s, rad/s, m/s²). `sm_state` is a number. Its values are the constants of `State.msg` (`State.HOVERING`, ...), which are the same as the `sm_states_t` enum of the firmware. The firmware checks at build time that both match.

**Time:** the stamps of the messages use the ROS time. On the real drone the clock is synchronized with the agent when it connects and again every 60 s. In simulation the time of Gazebo is used.

### Simulation mode

The firmware can also fly a simulated drone in Gazebo. In this mode the real sensors and motors are not used:

- IMU and barometer data come from ROS topics (`imu/data`, `barometer/data`) published by Gazebo.
- Motor speeds are published to `/drone/command/motor_speed` (`actuator_msgs/Actuators`) instead of driving the PWM pins.

The control code is exactly the same, so you can test the controller on the computer before flying the real drone.

## Requirements

- [ESP-IDF v5.2](https://github.com/espressif/esp-idf)
- ROS 2 Jazzy (with `colcon`)
- Docker (to run the micro-ROS agent)
- Gazebo Harmonic with `ros_gz` (only for simulation)

## Setup

### 1. Install ESP-IDF

```bash
git clone -b v5.2 https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh esp32s2   # Linux/macOS. For Windows, check the ESP-IDF docs.
```

You must activate ESP-IDF in every new terminal before using `idf.py`:

```bash
. esp-idf/export.sh
```

### 2. Download the micro-ROS component

The micro-ROS component is not included in this repository. Clone it inside the `components` folder, and the `actuator_msgs` package inside its `extra_packages` folder:

```bash
cd ESP-Micro_drone/code/esp_code/components
git clone https://github.com/ruibartsegura/micro_ros_espidf_component

cd micro_ros_espidf_component
git clone https://github.com/rudislabs/actuator_msgs.git extra_packages/actuator_msgs
```

The custom messages (`my_msgs`) are already in `extra_packages/my_msgs` of that fork.

Then install the Python packages that micro-ROS needs (only once):

```bash
pip3 install catkin_pkg colcon-common-extensions lark
```

### 3. Download the simulation (optional)

The Gazebo simulation is the [`sim_drone`](https://github.com/docencia-juanscelyg/sim_drone) package. Clone it inside the ROS workspace:

```bash
cd ESP-Micro_drone/code/ros_ws/src
git clone https://github.com/docencia-juanscelyg/sim_drone.git
```

## Build and flash the firmware

```bash
cd ESP-Micro_drone/code/esp_code
idf.py set-target esp32s2
idf.py menuconfig
idf.py build
idf.py flash monitor
```

In `menuconfig` you should check at least these options:

- **micro-ROS Settings** → WiFi SSID and password, agent IP (your computer's IP) and agent port (`8888`).
- **X-WING Config** → GPIO pins, default flight parameters (hovering height, max velocity, land on site), task priorities and stack sizes.
> With simulation, it is recommended to give the micro-ROS task the highest priority, because it acts as a sensor. With the real drone it is recommended to place it between the system and the odometry tasks.
- **X-WING Config → Simulation → Simulation on** → enable it to work with Gazebo instead of the real hardware.
- **X-WING Config → Tunning → Gains tune on** → starts the tuning task (see [Tune the controller](#tune-the-controller)).

The first build takes a long time because the micro-ROS library is compiled. If you change something inside the micro-ROS component (for example `colcon.meta` or the custom messages), rebuild it with:

```bash
idf.py clean-microros
idf.py build
```

## Run

### 1. Start the micro-ROS agent

On the computer, in the same network as the drone:

```bash
docker run -it --rm --net=host microros/micro-ros-agent:jazzy udp4 --port 8888 -v6
```

When the drone connects, you will see its topics with `ros2 topic list`.

### 2. Build the ROS workspace

The workspace contains the custom messages (`my_msgs`). You need to source it to read `drone/state` and call the drone services from the computer.

```bash
source /opt/ros/jazzy/setup.bash
cd ESP-Micro_drone/code/ros_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

Remember to run `source install/setup.bash` in every new terminal.

### 3. Fly

Watch the state of the drone:

```bash
ros2 topic echo /drone/state
```

Ask the drone to take off (it must be in `ARMING`):

```bash
ros2 service call /drone/takeoff_srv my_msgs/srv/Takeoff "{altitude: 0.5}"
```

The altitude is in metres and must be between 0.5 and 3.0. If the request is rejected, the `reason` field of the response explains why.

Send velocity commands while it is in the air:

```bash
ros2 topic pub /drone/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.2}}}"
```

Land (it must be in `HOVERING` or `EXTERNAL_CONTROL`):

```bash
ros2 service call /drone/land_srv std_srvs/srv/Trigger
```

Change the flight parameters (they are applied in the `CHECKING` state):

```bash
ros2 topic pub --once /drone/params my_msgs/msg/Params "{h_max: 0.5, v_max: 2.0, land_on_site: true}"
```

### Run in simulation

1. Enable simulation mode in `menuconfig`, then build and flash the ESP32.
2. Start the micro-ROS agent.
3. Launch Gazebo:

   ```bash
   ros2 launch sim_drone sim_gazebo.launch.py
   ```

The ESP32 still runs the control loop, but it reads the simulated sensors and moves the simulated motors.

### Tune the controller

With **Gains tune on** enabled in `menuconfig`, the gains of the controller can be changed while the drone (or the simulation) is running. Type the commands in `idf.py monitor` and press Enter:

| Command | What it does |
|---------|--------------|
| `help` | List of commands |
| `get` | Print the current gains |
| `kp_rate 8` | Change one gain (`kp_rate=8` also works) |
| `reset` | Go back to the default gains |

Gains: `kp_h`, `kd_h`, `kp_angle`, `kp_rate`, `max_rate`, `max_pow`, `base`. The changes are lost when the ESP32 restarts: copy the good values to the defines of `attitude_controller.c`.

## Tests

You can find the instructions in [`Tests`](https://github.com/ruibartsegura/ESP-Micro_drone/tree/main/code/esp_code/host_test/README.md)

## How to add or change things

### Project structure

```
esp_code/
├── main/                 # app_main() and the project Kconfig (menuconfig options)
├── components/
│   ├── system/           # state machine and start-up of all tasks
│   ├── flight_control/   # attitude controller, odometry, Madgwick filter, tuning task
│   ├── ros_coordinator/  # micro-ROS node, topics, services and ROS time
│   ├── global_data/
│   │   ├── state/        # shared sensor data, odometry and current state (thread safe)
│   │   └── parameters/   # flight parameters that can change at runtime
│   ├── drivers/          # imu (MPU-6050), height (BMP180), motors, leds
│   └── micro_ros_espidf_component/   # micro-ROS library (cloned, see Setup)
├── host_test/            # host unit tests and style check
└── tools/                # log and rosbag analysis
```

### Add a new driver or module

1. Create a new folder in `components/` (or in `components/drivers/`) with its own `CMakeLists.txt`, the `.c` files and an `include/` folder for the headers.
2. Register it with `idf_component_register(...)` and list the components it uses in `REQUIRES`.
3. Add your component to the `REQUIRES` list of every component that uses it.
4. If it needs its own task, create it with `xTaskCreate()` and add its priority and stack size to `main/Kconfig.projbuild` (menu "tasks settings"), like the other tasks.
5. If other tasks need its data, share it through `global_data/state` instead of global variables, so the access is thread safe.
6. Add its host tests in `host_test/`.

### Add a configuration option

Add a new `config` entry to `main/Kconfig.projbuild`. It will appear in `menuconfig` and you can use it in the code as `CONFIG_<NAME>`. Copy the new value to `host_test/mocks/sdkconfig.h` too.

### Add a new publisher, subscriber or service

1. Create it in `components/ros_coordinator/ros_coordinator.c`, next to the existing ones.
2. Every subscriber, service and timer needs a slot in the executor, so increase `N_HANDLERS` in the same file.
3. micro-ROS uses static memory, so the maximum number of entities is fixed at build time. Increase the limits in `components/micro_ros_espidf_component/colcon.meta`, in both `rmw_microxrcedds` and `embeddedrtps`:

   ```json
   "rmw_microxrcedds": {
       "cmake-args": [
           "-DRMW_UXRCE_MAX_NODES=1",
           "-DRMW_UXRCE_MAX_PUBLISHERS=4",
           "-DRMW_UXRCE_MAX_SUBSCRIPTIONS=6",
           "-DRMW_UXRCE_MAX_SERVICES=2",
           "-DRMW_UXRCE_MAX_CLIENTS=1",
           "-DRMW_UXRCE_MAX_HISTORY=4"
       ]
   },
   ```

4. Rebuild micro-ROS with `idf.py clean-microros` and then `idf.py build`.
5. Use `stamp_now()` to fill the `header.stamp` of the new messages, so they have the ROS time.

If you forget steps 2 or 3, the node will fail when it starts (usually with an `RCCHECK` error in the monitor).

> **QoS:** the drone uses *best effort* for the high-rate topics. A best effort subscriber can receive data from a reliable publisher, but a reliable subscriber **cannot** receive data from a best effort publisher. Keep this in mind when you connect new nodes.

### Add a custom message or service

1. Add the `.msg` or `.srv` file to `components/micro_ros_espidf_component/extra_packages/my_msgs/` and list it in its `CMakeLists.txt` (and its dependencies in `DEPENDENCIES` and `package.xml`).
2. Add the **same** files to `ros_ws/src/my_msgs/`, so the computer also knows the new type.
3. Rebuild both sides: `idf.py clean-microros && idf.py build` on the firmware, and `colcon build` in `ros_ws`.
4. Add a mock of the new type in `host_test/mocks/` so the host tests still compile.

If you add or change a state of `sm_states_t`, change the constants of `State.msg` too: the build fails if they do not match.

### Change the controller

The controller is in `components/flight_control/attitude_controller.c`. After changing it, run the host tests (`make test`), tune it with the tuning task and try it in simulation before flying the real drone.
