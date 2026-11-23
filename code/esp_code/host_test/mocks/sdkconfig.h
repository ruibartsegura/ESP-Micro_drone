/**
 * Made by Rui B.S.
 * Date: 08/10/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Mock of sdkconfig.h for the host tests. The values are a copy of the
 *   real sdkconfig of the project (keep them in sync if you change
 *   menuconfig). CONFIG_SIMULATION_ON is NOT defined here: the Makefile
 *   adds -DCONFIG_SIMULATION_ON=1 for the simulation builds, so every
 *   module can be tested in both modes.
 */
#pragma once

/* ---- Communication pins ---- */
#define CONFIG_I2C0_PIN_SDA 11
#define CONFIG_I2C0_PIN_SCL 10
#define CONFIG_I2C1_PIN_SDA 40
#define CONFIG_I2C1_PIN_SCL 41

/* ---- LEDs ---- */
#define CONFIG_LED_PIN_ESP   15
#define CONFIG_LED_PIN_BLUE  7
#define CONFIG_LED_PIN_GREEN 9
#define CONFIG_LED_PIN_RED   8

/* ---- Motors ---- */
#define CONFIG_MOTOR01_PIN 5
#define CONFIG_MOTOR02_PIN 6
#define CONFIG_MOTOR03_PIN 3
#define CONFIG_MOTOR04_PIN 4

/* ---- Flight parameters ---- */
#define CONFIG_VEL_MAX      50   /* cm/s */
#define CONFIG_LAND_ON_SITE 1
#define CONFIG_FLIGHT_H_MIN 50   /* cm */
#define CONFIG_FLIGHT_H_MAX 300  /* cm */

#define CONFIG_IMU_TIMEOUT  300
#define CONFIG_BARO_TIMEOUT 500
#define CONFIG_LINK_TIMEOUT 1500

/* ---- Controller (decimal values x1000) ---- */
#define CONFIG_THROTTLE_BASE_SIM  2378
#define CONFIG_THROTTLE_BASE_REAL 50
#define CONFIG_THROTTLE_BASE      2378
#define CONFIG_ALPHA              98
#define CONFIG_MAX_TILT_ANGLE     35
#define CONFIG_MAX_POWER_RATE     200
#define CONFIG_MAX_POWER_RP       400
#define CONFIG_MAX_POWER_YAW      150
#define CONFIG_PITCH_DEVIATION    105
#define CONFIG_ROLL_DEVIATION     0
#define CONFIG_KP_H        100000
#define CONFIG_KD_H        110000
#define CONFIG_KP_VEL      50
#define CONFIG_KP_ANGLE    1500
#define CONFIG_KP_RATE     300
#define CONFIG_KI_RATE     600
#define CONFIG_KP_YAW_RATE 3000

/* ---- Odometry (x1000) ---- */
#define CONFIG_K_DRAG 124

/* ---- Tasks (stack / priority) ---- */
#define CONFIG_SYSTEM_TASK_STACK   4000
#define CONFIG_SYSTEM_TASK_PRIO    4
#define CONFIG_IMU_TASK_STACK      3200
#define CONFIG_IMU_TASK_PRIO       7
#define CONFIG_HEIGHT_TASK_STACK   3200
#define CONFIG_HEIGHT_TASK_PRIO    5
#define CONFIG_ODOM_TASK_STACK     3200
#define CONFIG_ODOM_TASK_PRIO      5
#define CONFIG_ATTITUDE_TASK_STACK 4900
#define CONFIG_ATTITUDE_TASK_PRIO  6
#define CONFIG_TUNING_TASK_STACK   3072
#define CONFIG_TUNING_TASK_PRIO    1
#define CONFIG_MICRO_ROS_STACK     8000
#define CONFIG_MICRO_ROS_TASK_PRIO 3

/* ---- Tuning (gains from the serial console) ---- */
#define CONFIG_GAINS_TUNE_ON 1
/* CONFIG_CPU_STATS_ON is on in the real sdkconfig, but not here: the other
 * modules get the empty task_period_tick() of cpu_stats.h. The cpu_stats
 * suite turns it on with -D (see the Makefile). */

/* ---- micro-ROS ---- */
#define CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE 1
#define CONFIG_MICRO_ROS_ESP_NETIF_WLAN          1
#define CONFIG_MICRO_ROS_AGENT_IP   "10.167.216.60"
#define CONFIG_MICRO_ROS_AGENT_PORT "8888"
