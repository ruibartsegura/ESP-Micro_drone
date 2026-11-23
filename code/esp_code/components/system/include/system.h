/**
 * Made by Rui B.S.
 * Date: 22/07/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Public interface of the system module (initialisation, tests and
 *   state machine of the drone).
 *
 * Functions:
 *   - system_init(): initialises all the modules.
 *   - system_test(): checks that all the modules started correctly.
 *   - system_start(): creates the FreeRTOS task that runs the state machine.
 *   - get_attitude(): returns the target attitude (cmd_vel + height).
 *   - get_barometer_failing() / get_imu_failing(): failure of the emergency
 *     landing (none of them = link with the agent lost).
 */

#ifndef SYSTEM_H
#define SYSTEM_H

#include <stdbool.h>
#include <stdint.h>


#include "state.h"
#include "attitude_controller.h"

// Returns the target attitude (cmd_vel + height).
ATTITUDE_TARGET get_attitude();

// Initialises all the modules.
void system_init(void);

// Checks that all the modules started correctly.
bool system_test(void);

// Get if the barometer is failing.
bool get_barometer_failing(void);

// Get if the IMU is failing (emergency landing without stabilization).
bool get_imu_failing(void);

// Launch the FreeRTOS task, it will execute the state machine.
void system_start(void);

#endif //SYSTEM_H