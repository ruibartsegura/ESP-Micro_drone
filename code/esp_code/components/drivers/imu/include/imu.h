/**
 * Made by Rui B.S.
 * Date: 23/05/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Public interface and data types of the IMU module, and the limits used
 *   to decide if the drone is stable enough to arm.
 *
 * Functions:
 *   - imu_init(): starts the I2C bus, the sensor, the calibration and the task.
 *   - imu_test(): checks that the module started correctly.
 *   - imu_check_stable_for_arming(): checks if the drone is still and level, so it is safe to arm.
 */

#ifndef IMU_H
#define IMU_H

#include <stdbool.h>
#include <stdint.h>
#include <sensor_msgs/msg/imu.h>


#include "esp_err.h"
#include "driver/i2c.h"

#include "sdkconfig.h"
#include "mpu6050.h"
#include "ros_coordinator.h"

// With simulation skip this params
#ifndef CONFIG_SIMULATION_ON
    // ---- I2C bus configuration ----
    #define I2C_MASTER_NUM       I2C_NUM_0
    #define I2C_MASTER_SDA_IO    CONFIG_I2C0_PIN_SDA // I2C SDA pin
    #define I2C_MASTER_SCL_IO    CONFIG_I2C0_PIN_SCL // I2C SCL pin
    #define I2C_MASTER_FREQ_HZ   400000
    #define ESP_INTR_FLAG_DEFAULT 0
#endif


void imu_init(void);

bool imu_test(void);


// Max tilt allowed from the vertical (degrees)
#define IMU_ARM_MAX_TILT_DEG      10.0f

// Max angular velocity allowed (deg/s) to consider the drone "still"
#define IMU_ARM_MAX_GYRO_DPS      5.0f

// Allowed range of the module of the acceleration vector (in g). Out of
// this range there is too much vibration, handling or free fall.
#define IMU_ARM_ACCEL_MIN_G       0.9f
#define IMU_ARM_ACCEL_MAX_G       1.1f

// Number of consecutive samples that must meet all the conditions
#define IMU_ARM_CHECK_SAMPLES     20

// Period between samples, in ms
#define IMU_ARM_CHECK_PERIOD_MS   10

typedef enum {
    IMU_ARM_STATE_STABLE = 0, // Ready to arm
    IMU_ARM_STATE_TILTED, // Tilt out of range
    IMU_ARM_STATE_MOVING, // Angular velocity out of range
    IMU_ARM_STATE_ACCEL_ABNORMAL, // Acceleration module out of range
} imu_arm_state_t;

// Checks, with several consecutive samples, if the drone is stable
// (level, still and without abnormal vibration), so it is safe to
// arm it.
esp_err_t imu_check_stable_for_arming(imu_arm_state_t *state);


#endif // IMU_H

// TODO
//      * Review the stabilisation
