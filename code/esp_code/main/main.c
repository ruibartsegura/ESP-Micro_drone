/**
 * Made by Rui B.S.
 * Date: 22/07/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Entry point of the firmware. ESP-IDF calls app_main() when the chip
 *   boots, and it only starts the system task that runs the drone.
 *
 * Functions:
 *   - app_main(): starts the system with system_start().
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>



#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/portmacro.h"
#include "nvs_flash.h"

#include "system.h"




void app_main()
{
    // Start the system
    system_start();
}