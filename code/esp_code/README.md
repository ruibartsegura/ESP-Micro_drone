# esp_code

Firmware of the micro-drone for the ESP32-S2 (ESP-IDF v5.2 + micro-ROS).

The complete guide (how it works, setup, build, flight, simulation and how to add things) is in [`../README.md`](../README.md).

## Quick start

```bash
. $IDF_PATH/export.sh
idf.py set-target esp32s2
idf.py menuconfig          # WiFi, agent IP/port, pins, simulation, tuning
idf.py build flash monitor
idf.py clean-microros      # after changing colcon.meta or the custom messages
```

## Folders

| Folder | What it is |
|--------|------------|
| `main/` | `app_main()` and `Kconfig.projbuild` (the "X-WING Config" menu of `menuconfig`). |
| `components/` | Every module of the firmware: system, flight control, ROS coordinator, global data and drivers. |
| `components/micro_ros_espidf_component/` | micro-ROS library (fork, cloned apart). The custom messages are in its `extra_packages/my_msgs/`. |
| `host_test/` | Host unit tests and style check. See [`host_test/README.md`](host_test/README.md). |
| `tools/` | Scripts to analyze the monitor log and the rosbags of the simulation. |
