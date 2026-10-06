idf_component_register(SRCS "ros_coordinator.c"
                       INCLUDE_DIRS "." "include"
                       REQUIRES micro_ros_espidf_component freertos leds imu system parameters state
                       )

target_include_directories(${COMPONENT_LIB} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../micro_ros_espidf_component/micro_ros_src/install/include/actuator_msgs
)