# x_wing

## Futures Configuration Manual

### New ROS publisher subcriber or services
To add new publisher subcriber or services to the program would be necesary to modify the next:

In components/micro_ros_espidf_component/colcon.meta, the max number of the thing we want to change.
```c
"rmw_microxrcedds": {
            "cmake-args": [
                "-DRMW_UXRCE_XML_BUFFER_LENGTH=400",
                "-DRMW_UXRCE_TRANSPORT=udp",
                "-DRMW_UXRCE_MAX_NODES=1",
                "-DRMW_UXRCE_MAX_PUBLISHERS=2",
                "-DRMW_UXRCE_MAX_SUBSCRIPTIONS=4",
                "-DRMW_UXRCE_MAX_SERVICES=2",
                "-DRMW_UXRCE_MAX_CLIENTS=1",
                "-DRMW_UXRCE_MAX_HISTORY=1"
            ]
        },
        "embeddedrtps": {
            "cmake-args": [
                "-DERTPS_MAX_PUBLISHERS=2",
                "-DERTPS_MAX_SUBSCRIPTIONS=4",
                "-DERTPS_MAX_SERVICES=2",
                "-DERTPS_MAX_CLIENTS=1",
                "-DERTPS_MAX_HISTORY=10"
            ]
        },
```
