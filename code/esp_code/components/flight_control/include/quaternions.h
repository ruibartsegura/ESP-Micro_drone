/**
 * Made by Rui B.S.
 * Date: 22/07/2026
 * email: rui.bartolome@gmail.com
 * Based on: "MPU6050 ESP32 Sensor Interface", original author Muhammad Idrees
 *
 * Description:
 *   Public interface of the Madgwick filter. Changes: normalisation, beta,
 *   integration factor and units fixed.
 *
 * Functions:
 *   - quaternion_init(): sets the identity quat_t.
 *   - quaternion_update(): runs one step of the filter (gyro in rad/s, dt in s).
 *   - quaternion_get_roll() / _pitch() / _yaw(): return the Euler angles.
 */

#ifndef QUATERNIONS_H
#define QUATERNIONS_H

#include <math.h>
#include <state.h>

void quaternion_init(quat_t *q);

/**
 * @brief Filtro Madgwick.
 * @param gx,gy,gz Velocidad angular en RAD/S
 * @param ax,ay,az Aceleración en cualquier unidad (se normaliza internamente),
 *                  típicamente en g o m/s².
 * @param dt        Delta de tiempo real en segundos desde la última llamada.
 */
void quaternion_update(quat_t *q, float gx, float gy, float gz,
                        float ax, float ay, float az, float dt);

float quaternion_get_roll(const quat_t *q);
float quaternion_get_pitch(const quat_t *q);
float quaternion_get_yaw(const quat_t *q);

#endif // QUATERNIONS_H
