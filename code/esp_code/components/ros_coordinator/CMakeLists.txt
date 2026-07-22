idf_component_register(SRCS "ros_coordinator.c"
                       INCLUDE_DIRS "." "include"
                       REQUIRES micro_ros_espidf_component freertos leds imu system parameters
                       )
