idf_component_register(SRCS "system.c"
                INCLUDE_DIRS "." "include"
                REQUIRES leds ros_coordinator)

idf_component_get_property( FREERTOS_ORIG_INCLUDE_PATH freertos ORIG_INCLUDE_PATH)
target_include_directories(${COMPONENT_TARGET} PUBLIC
  "${FREERTOS_ORIG_INCLUDE_PATH}"
)
target_compile_options(${COMPONENT_LIB} PRIVATE "-fno-strict-aliasing"
                                                "-Wno-error=stringop-truncation"
                                                "-Wno-absolute-value"
                                                "-Wno-address-of-packed-member"
                                                "-Wno-stringop-truncation"
                                                "-Wno-format")