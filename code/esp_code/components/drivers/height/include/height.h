/**
 * Made by Rui B.S.
 * Date: 25/09/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Public interface and I2C configuration of the height module.
 *
 * Functions:
 *   - height_init(): starts the I2C bus and the task.
 *   - height_test(): checks that the module started correctly.
 */

#ifndef HEIGHT_H
#define HEIGHT_H

#include <stdbool.h>
#include <stdint.h>
#include <geometry_msgs/msg/pose_stamped.h>



#include "esp_err.h"
#include "driver/i2c.h"

#include "sdkconfig.h"
#include "bmp180.h"
#include "ros_coordinator.h"

// With simulation skip this params
#ifndef CONFIG_SIMULATION_ON
    // ---- I2C bus configuration ----
    #define I2C_MASTER_NUM       I2C_NUM_0
    #define I2C_MASTER_SDA_IO    CONFIG_I2C0_PIN_SDA   // I2C SDA pin
    #define I2C_MASTER_SCL_IO    CONFIG_I2C0_PIN_SCL   // I2C SCL pin
    #define I2C_MASTER_FREQ_HZ   400000
    #define ESP_INTR_FLAG_DEFAULT 0
#endif

void height_init(void);

bool height_test(void);

#endif // HEIGHT_H
