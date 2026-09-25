idf_component_register(
    SRCS "height.c" "bmp180.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_timer ros_coordinator
)