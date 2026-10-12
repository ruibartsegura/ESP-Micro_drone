/**
 * Made by Rui B.S.
 * Date: 24/09/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Public interface of the odometry estimator.
 *
 * Functions:
 *   - odom_estimator_init(): creates the odometry task.
 *   - odom_estimator_test(): checks that the module started correctly.
 */

#ifndef ODOMETRY_ESTIMATOR_H
#define ODOMETRY_ESTIMATOR_H

#include <stdbool.h>
#include "sdkconfig.h"

void init_odom_estimator(void);
bool odom_estimator_test(void); // Check leds

#endif // ODOMETRY_ESTIMATOR_H
