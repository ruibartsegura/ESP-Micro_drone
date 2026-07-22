idf_component_register(
    SRCS "imu.c" "mpu6050.c" "quaternions.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer
)