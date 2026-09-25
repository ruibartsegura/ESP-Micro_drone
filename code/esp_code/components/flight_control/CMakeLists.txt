idf_component_register(
    SRCS "attitude_controller.c" "odometry_estimator.c"
    INCLUDE_DIRS "." "include"
    REQUIRES micro_ros_espidf_component freertos imu height motors system parameters ros_coordinator
)
