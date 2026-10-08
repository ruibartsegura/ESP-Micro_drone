idf_component_register(
    SRCS "attitude_controller.c" "odometry_estimator.c" "tuning.c"
    INCLUDE_DIRS "." "include"
    REQUIRES micro_ros_espidf_component freertos pthread imu height motors system parameters ros_coordinator
)
