idf_component_register(
    SRCS "attitude_controller.c"
    INCLUDE_DIRS "." "include"
    REQUIRES micro_ros_espidf_component freertos imu motors system parameters ros_coordinator
)
