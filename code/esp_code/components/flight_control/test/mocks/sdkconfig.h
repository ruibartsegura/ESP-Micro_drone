#pragma once
/*
 * Mock de sdkconfig.h para compilar los tests EN HOST (gcc), sin ESP-IDF.
 * El valor real de cada CONFIG_* lo decide tu Kconfig.projbuild (los hay en
 * main/ y en algunos components (cada uno con su Kconfig.projbuild); aquí solo hace falta
 * que el símbolo EXISTA para que las cabeceras compilen. Si tu proyecto
 * define alguno más (o lo renombra), añádelo/ajústalo aquí.
 */

/* ---- Pines GPIO ---- */
#define CONFIG_MOTOR01_PIN     1
#define CONFIG_MOTOR02_PIN     2
#define CONFIG_MOTOR03_PIN     3
#define CONFIG_MOTOR04_PIN     4
#define CONFIG_I2C0_PIN_SDA    8
#define CONFIG_I2C0_PIN_SCL    9
#define CONFIG_LED_PIN_ESP     10
#define CONFIG_LED_PIN_RED     11
#define CONFIG_LED_PIN_GREEN   12
#define CONFIG_LED_PIN_BLUE    13

/* ---- Tareas FreeRTOS (main/Kconfig.projbuild) ---- */
#define CONFIG_SYSTEM_TASK_STACK   4096
#define CONFIG_SYSTEM_TASK_PRIO    5
#define CONFIG_IMU_TASK_STACK      4096
#define CONFIG_IMU_TASK_PRIO       5
#define CONFIG_ATTITUDE_TASK_STACK 4096
#define CONFIG_ATTITUDE_TASK_PRIO  5
#define CONFIG_MICRO_ROS_STACK     8192
#define CONFIG_MICRO_ROS_TASK_PRIO 5

/* ---- micro-ROS / red ---- */
#define CONFIG_MICRO_ROS_AGENT_IP   "192.168.1.1"
#define CONFIG_MICRO_ROS_AGENT_PORT "8888"
/* Dejar SIN definir CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE,
 * CONFIG_MICRO_ROS_ESP_NETIF_WLAN/ENET: son #ifdef opcionales y
 * ros_coordinator.c no entra en la compilación de los tests. */

/* ---- Parámetros de vuelo (configurations/parameters) ---- */
#define CONFIG_HOVERING_H    100   /* cm */
#define CONFIG_VEL_MAX       100   /* cm/s */
#define CONFIG_LAND_ON_SITE  1

/* CONFIG_SIMULATION_ON: NO definido a propósito.
 * Así imu.h/height.h exponen las macros de pines I2C (rama hardware real),
 * que es la que ejercitan estos tests. */
