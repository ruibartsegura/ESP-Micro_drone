# ESP Micro Drone Code

This folder contains all the software of the micro-drone. It is split into two parts:

| Folder | What it is |
|--------|------------|
| [`esp_code/`](https://github.com/ruibartsegura/ESP-Micro_drone/tree/main/code/esp_code) | Firmware for the ESP32-S2 (ESP-IDF + micro-ROS), its host unit tests and some analysis tools. |
| [`ros_ws/`](https://github.com/ruibartsegura/ESP-Micro_drone/tree/main/code/ros_ws) | ROS 2 workspace with the custom messages (`my_msgs`) and the Gazebo simulation. |

## Index

- [How it works](#how-it-works)
  - [State machine](#state-machine) · [Failsafe](#failsafe) · [Flight control](#flight-control) · [ROS interface](#ros-interface) · [Simulation mode](#simulation-mode)
- [Requirements](#requirements)
- [Setup](#setup)
- [Build and flash the firmware](#build-and-flash-the-firmware)
- [Run](#run)
  - [Fly](#3-fly) · [Change the parameters](#change-the-parameters) · [Run in simulation](#run-in-simulation) · [Tune the controller](#tune-the-controller)
- [Tests](#tests)
- [Analysis tools](#analysis-tools)
- [How to add or change things](#how-to-add-or-change-things)
  - [Project structure](#project-structure) · [New driver or module](#add-a-new-driver-or-module) · [Configuration option](#add-a-configuration-option) · [Runtime parameter](#add-a-runtime-parameter) · [Publisher, subscriber or service](#add-a-new-publisher-subscriber-or-service) · [Custom message](#add-a-custom-message-or-service) · [Controller](#change-the-controller)

## Requirements

- [ESP-IDF v5.2](https://github.com/espressif/esp-idf)
- ROS 2 Jazzy (with `colcon`)
- Docker (to run the micro-ROS agent)
- Gazebo Harmonic with `ros_gz` (only for simulation)

## How it works

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
│   │   └── parameters/   # flight, controller and odometry parameters that can change at runtime
│   ├── drivers/          # imu (MPU-6050), height (BMP180), motors, leds
│   └── micro_ros_espidf_component/   # micro-ROS library (cloned, see Setup)
├── host_test/            # host unit tests and style check
└── tools/                # log and rosbag analysis
```

### Task information

The firmware runs on an ESP32-S2 using FreeRTOS. Each part of the system is a separate task:

| Task | Job | Period | Priority |
|------|-----|--------|----------|
| `imu_task` | Reads the MPU-6050 (accelerometer + gyroscope) | 5 ms | 7 |
| `attitude_task` | Cascade PID controller, height control and motor mixing | 5 ms (15 ms in simulation) | 6 |
| `odom_task` | Odometry: orientation (Madgwick filter), velocity and position | 10 ms | 5 |
| `bmp180_task` | Reads the BMP180 barometer and computes the altitude | 10 ms | 5 |
| `system_task` | Runs the state machine and watches the sensors | 20 ms | 4 |
| `uros_task` | micro-ROS executor: topics and services | event driven | 3 |
| `tuning_task` | Changes the runtime parameters (gains, flight, odometry) from the serial console (optional) | 20 ms | 1 |

The priorities and stack sizes can be changed in `menuconfig`. In simulation mode `imu_task` and `bmp180_task` are not started: the sensor data comes from Gazebo.

All the tasks share the data (sensors, odometry and current state) through `global_data/state`, which protects every access with a critical section. The parameters that can change at runtime (gains of the controller, flight and odometry parameters) are in `global_data/parameters`, with the same kind of protection.

### State machine

```
INIT → CHECKING → ARMING → TAKING_OFF → HOVERING ↔ EXTERNAL_CONTROL → LANDING → DISARMING → ARMING

TAKING_OFF, HOVERING, EXTERNAL_CONTROL, LANDING → EMERGENCY_LANDING → ERROR   (a sensor stops sending data)
CHECKING → ERROR                                                              (the self test fails)
```

| State | What happens | LEDs |
|-------|--------------|------|
| `INIT` | Starts every module | |
| `CHECKING` | Self test of every module, which spins each motor. The flight parameters can still be changed | green on if it passes |
| `ARMING` | Waits for the take-off service (in flight the same service changes the altitude) | solid green |
| `TAKING_OFF` | Goes up to the requested altitude | solid green + blue blinking |
| `HOVERING` | Holds the altitude without moving | solid green +  solid blue |
| `EXTERNAL_CONTROL` | Follows `drone/cmd_vel`. Goes back to `HOVERING` if no command arrives for 500 ms | green blinking + solid blue|
| `LANDING` | Goes down to the ground (after the land service) | solid green + red blinking |
| `DISARMING` | Stops the motors and goes back to `ARMING`, ready for a new flight |  |
| `EMERGENCY_LANDING` | A sensor has stopped sending data, or the link with the agent is lost (see [Failsafe](#failsafe)). The drone ignores `drone/cmd_vel` and rejects the take-off and land services | red blinking |
| `ERROR` | Final state with the motors stopped. Restart the drone to leave it | solid red |

The ESP32 built-in LED turns on when the drone connects to the micro-ROS agent.

> **Warning:** the self test of `CHECKING` spins the motors on every start-up. Keep clear of the propellers.

### Failsafe

While flying, `system_task` checks how long it has been since the last sample of each sensor and since the last answer of the micro-ROS agent. It uses the local time of arrival, so it works the same with the real sensors and in simulation. If one of them exceeds its limit, the drone goes to `EMERGENCY_LANDING`:

| Failure | Limit | Limit in simulation | Reaction |
|---------|-------|---------------------|----------|
| IMU | 100 ms | 300 ms | Power ramp to 0 |
| Barometer | 500 ms | 500 ms | Stabilized descent |
| Link with the agent | 1500 ms | not checked | Landing with the sensors |

- **Barometer failure:** the drone keeps stabilizing itself with the IMU. The power goes down to a bit less than the hover power and stays there. The hover power is the base throttle plus the mean height correction measured in stable flight. The motors stop after `sqrt(2 * h / 0.5) + 1` seconds, where `h` is the estimated height in metres when the failure starts.
- **IMU failure:** without the IMU the drone cannot stabilize itself. The controller stops writing to the motors, and `system_task` takes the 4 motors from their mean power down to 0 in 500 ms. This limits the fall, but **it is not a safe landing**.
- **Link lost (real drone):** `uros_task` pings the agent every 500 ms (100 ms timeout, 1 attempt), and `system_task` only compares the time of the last answer, so it never waits. After `LINK_TIMEOUT` (1500 ms in `menuconfig`, 3 pings) the sensors are still ok, so the drone lands like in `LANDING` (target height 0, no velocity), but without ROS: it ignores `cmd_vel` and the services. When it reaches the ground the motors stop. If the ground is not detected after `h / 0.15 + 3` seconds, they stop anyway.
- **Link lost in simulation:** the IMU and the barometer come from Gazebo over ROS, so a lost link is detected as an IMU failure (it is the shortest timeout). The drone does the same power ramp: the motor speeds can not arrive to Gazebo, but it ends in `ERROR` after 500 ms anyway. If only the IMU topic stops, the link is ok and the ramp does arrive. The ping is not used in simulation: it would stop the executor, and the motors timer runs in that executor.
- **A worse failure replaces a milder one:** IMU > barometer > link. For example, a barometer failure while landing without link starts the descent without barometer; an IMU failure changes to the IMU reaction.

Every case ends in `ERROR`, even if the sensor or the link comes back: nobody can command the drone, and it must be checked before it flies again. The drone does not detect the ground without barometer: that descent ends only by time.

The descent values are starting points that must be tuned in simulation: `EMRGY_POW_DOWN` and `EMRGY_POW_RATE` in `attitude_controller.c`, and `EMRGY_ACC` in `system.c`.

### Flight control

Three-stage cascade: velocity → angle → angular rate.

- **Velocity (P):** the error between the velocity command (`cmd_vel`) and the odometry velocity is converted into target roll and pitch angles (limited to 20°).
- **Angle (P):** the angle error gives a target angular rate (limited to `max_rate`).
- **Angular rate (PI in roll / pitch, P in yaw):** the rate error gives the power of each axis. The power goes to the motors with an X-frame mix. Yaw only has this stage.
- **Height:** PD controller over a base throttle. The height is estimated with a complementary filter of the accelerometer and the barometer.
- The drone has reached a target height when it stays within 5 cm, with a vertical velocity under 0.05 m/s, for 500 ms.

### ROS interface

The drone talks to a ROS 2 computer over **WiFi (UDP)**. The ESP32 runs a single micro-ROS node called `rui_drone`, and a **micro-ROS agent** on the computer connects it to the rest of the ROS 2 network.

| Direction | Name | Type |
|-----------|------|------|
| Publish | `drone/state` | `my_msgs/msg/State` |
| Subscribe | `drone/cmd_vel` | `geometry_msgs/TwistStamped` |
| Subscribe | `drone/params` | `my_msgs/msg/Params` |
| Service | `drone/set_altitude` | `my_msgs/srv/SetAltitude` |
| Service | `drone/land_srv` | `std_srvs/srv/Trigger` |

`drone/state` is published every 300 ms. It contains the position, orientation, linear and angular velocity, acceleration and the current state of the state machine, with SI units (m, m/s, rad/s, m/s²). `sm_state` is a number. Its values are the constants of `State.msg` (`State.HOVERING`, ...), which are the same as the `sm_states_t` enum of the firmware. The firmware checks at build time that both match.

**Time:** the stamps of the messages use the ROS time. On the real drone the clock is synchronized with the agent when it connects and again every 60 s. In simulation the time of Gazebo is used.

### Simulation mode

The firmware can also fly a simulated drone in Gazebo. In this mode the real sensors and motors are not used:

- IMU and barometer data come from ROS topics (`imu/data`, `barometer/data`) published by Gazebo.
- Motor speeds are published to `/drone/command/motor_speed` (`actuator_msgs/Actuators`) instead of driving the PWM pins.

The control code is exactly the same, so you can test the controller on the computer before flying the real drone.

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
- **X-WING Config** → GPIO pins, default flight parameters (min / max altitude, max horizontal velocity in cm/s), default gains of the controller, rotor drag of the odometry, task priorities and stack sizes. These are only the start values: the flight, controller and odometry parameters can be changed later by ROS or the serial console (see [Change the parameters](#change-the-parameters)).
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
ros2 service call /drone/set_altitude my_msgs/srv/SetAltitude "{altitude: 0.5}"
```

The altitude is in metres and must be between 0.5 and 3.0. If the request is rejected, the `reason` field of the response explains why.

The same service changes the altitude in flight (`TAKING_OFF`, `HOVERING` or `EXTERNAL_CONTROL`). The drone goes to the new altitude and keeps its state; the response says `Altitude changed`:

```bash
ros2 service call /drone/set_altitude my_msgs/srv/SetAltitude "{altitude: 1.2}"
```

In the other states (`LANDING`, `DISARMING`, `CHECKING`...) it is rejected with `Not armed`.

Send velocity commands while it is in the air:

```bash
ros2 topic pub /drone/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.2}}}"
```

The horizontal velocity is limited to `v_max` (0.5 m/s by default, `VEL_MAX` in `menuconfig` in cm/s). The limit is on the module of `linear.x` and `linear.y`: a faster command is scaled down and keeps its direction.

Land (it must be in `HOVERING` or `EXTERNAL_CONTROL`):

```bash
ros2 service call /drone/land_srv std_srvs/srv/Trigger
```

### Change the parameters

The flight parameters, the gains of the controller and the rotor drag of the odometry can be changed with the `drone/params` topic. All of them go in one message (`my_msgs/msg/Params`), so the drone only needs one subscriber.

Only the fields that you fill change: the rest keep their default value `-1`, and the drone does not touch them (any negative value means "do not change").

```bash
# Only the max velocity
ros2 topic pub --once /drone/params my_msgs/msg/Params "{v_max: 0.3}"

# One gain and the rotor drag
ros2 topic pub --once /drone/params my_msgs/msg/Params "{kp_rate: 9.0, k_drag: 0.13}"
```

From Python it works the same way, because `Params()` starts with every field at `-1`:

```python
msg = Params()
msg.kp_vel = 0.06
pub.publish(msg)
```

| Group | Fields |
|-------|--------|
| Flight | `v_max` (max horizontal velocity of `cmd_vel`, m/s) |
| Controller | `kp_h`, `kd_h`, `kp_vel`, `kp_angle`, `kp_rate`, `ki_rate`, `kp_yaw_rate`, `max_rate`, `max_pow_rp`, `base`, `alpha`, `max_angle` (rad), `max_pow_yaw`, `pitch_deviation`, `roll_deviation` |
| Odometry | `k_drag` (1/s) |

- The parameters are applied only in the `CHECKING` state. A message that arrives in another state (for example in flight) waits until the next `CHECKING`, after landing. If several messages arrive before that, they are merged: each filled field keeps its last value, so no change is lost. To change gains in flight use the serial console ([Tune the controller](#tune-the-controller)).
- Every value is checked against the range of `param_info[]` in `components/global_data/parameters/parameters.c`. If one value is out of range, NaN or inf, **the whole message is rejected** and the monitor shows `drone/params rejected: <name> out of range`.
- The changes are lost when the ESP32 restarts. To keep a value, change its default in `menuconfig`.
- The subscriber is *reliable* (and transient local), so a message lost over WiFi is sent again. The publisher must be reliable too: `ros2 topic pub` and the default `rclpy` / `rclcpp` publishers are. A best effort publisher never reaches the drone.

### Run in simulation

1. Enable simulation mode in `menuconfig`, then build and flash the ESP32.
2. Start the micro-ROS agent.
3. Launch Gazebo:

   ```bash
   ros2 launch sim_drone sim_gazebo.launch.py
   ```

The ESP32 still runs the control loop, but it reads the simulated sensors and moves the simulated motors.

### Tune the controller

With **Gains tune on** enabled in `menuconfig`, the parameters can be changed while the drone (or the simulation) is running, also in flight. Type the commands in `idf.py monitor` and press Enter:

| Command | What it does |
|---------|--------------|
| `help` | List of parameters with their allowed range |
| `get` | Print the current value of every parameter |
| `kp_rate 8` | Change one parameter (`kp_rate=8` also works) |
| `reset` | Go back to the `menuconfig` values |

The names and the ranges are the same as the fields of `drone/params` (see [Change the parameters](#change-the-parameters)). The changes are lost when the ESP32 restarts: copy the good values to `menuconfig` (**X-WING Config → controller config**; the gains are multiplied by 1000 there).

## Tests

You can find the instructions in [`Tests`](https://github.com/ruibartsegura/ESP-Micro_drone/tree/main/code/esp_code/host_test/README.md)

## How to add or change things

### Add a new driver or module

1. Create a new folder in `components/` (or in `components/drivers/`) with its own `CMakeLists.txt`, the `.c` files and an `include/` folder for the headers.
2. Register it with `idf_component_register(...)` and list the components it uses in `REQUIRES`.
3. Add your component to the `REQUIRES` list of every component that uses it.
4. If it needs its own task, create it with `xTaskCreate()` and add its priority and stack size to `main/Kconfig.projbuild` (menu "tasks settings"), like the other tasks.
5. If other tasks need its data, share it through `global_data/state` instead of global variables, so the access is thread safe.
6. Add its host tests in `host_test/`.

### Add a configuration option

Add a new `config` entry to `main/Kconfig.projbuild`. It will appear in `menuconfig` and you can use it in the code as `CONFIG_<NAME>`. Copy the new value to `host_test/mocks/sdkconfig.h` too.

### Add a runtime parameter

1. Add the field (a `float`) to the right struct of `components/global_data/parameters/include/parameters.h` (`flight_params_t`, `GAINS` or `odom_params_t`).
2. In `parameters.c`, add its default value (usually from a new `menuconfig` option) and a row in `param_info[]` with its name and range. The build fails if a field of `params_t` has no row. The minimum must be `>= 0`: a negative value means "do not change".
3. Add the same name with default value `-1.0` to `Params.msg`, in both copies (`components/micro_ros_espidf_component/extra_packages/my_msgs/msg/` and `ros_ws/src/my_msgs/msg/`), and copy it from the message to `params_t` in `apply_pending_params()` (`ros_coordinator.c`).
4. Add the field to `my_msgs__msg__Params` in `host_test/mocks/mock_ros_msgs.h`.
5. Rebuild both sides (`idf.py clean-microros && idf.py build`, and `colcon build` in `ros_ws`).

The serial console finds the new parameter by itself, because it uses `param_info[]`.

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

> **QoS:** the drone uses *best effort* for the high-rate topics (`drone/params` and the services are reliable). A best effort subscriber can receive data from a reliable publisher, but a reliable subscriber **cannot** receive data from a best effort publisher. Keep this in mind when you connect new nodes.

### Add a custom message or service

1. Add the `.msg` or `.srv` file to `components/micro_ros_espidf_component/extra_packages/my_msgs/` and list it in its `CMakeLists.txt` (and its dependencies in `DEPENDENCIES` and `package.xml`).
2. Add the **same** files to `ros_ws/src/my_msgs/`, so the computer also knows the new type.
3. Rebuild both sides: `idf.py clean-microros && idf.py build` on the firmware, and `colcon build` in `ros_ws`.
4. Add a mock of the new type in `host_test/mocks/` so the host tests still compile.

If you add or change a state of `sm_states_t`, change the constants of `State.msg` too: the build fails if they do not match.

### Change the controller

The controller is in `components/flight_control/attitude_controller.c`, and its gains and limits in `components/global_data/parameters/` (read them with `get_gains()`, once per cycle). After changing it, run the host tests (`make test`), tune it with the tuning task and try it in simulation before flying the real drone.
