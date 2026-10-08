/**
 * Made by Rui B.S.
 * Date: 22/07/2026
 * email: rui.bartolome@gmail.com
 * Based on: "MPU6050 ESP32 Sensor Interface", original author Muhammad Idrees
 *
 * Description:
 *   Madgwick filter with quaternions to estimate the orientation of the
 *   drone from the gyroscope and the accelerometer.
 *
 * Changes from the original version (see the "FIX:" comments):
 *   1. quaternion_init() set w=2.0 instead of 1.0 (identity quaternion).
 *   2. The normalisation applied sqrt() and then 1/sqrt(), which gives
 *      magnitude^-0.5 instead of magnitude^-1. Now inv_sqrt() is called
 *      directly on the sum of squares.
 *   3. BETA=2.0 was too high (usual values are 0.03-0.5): the filter almost
 *      ignored the gyroscope and trusted only the accelerometer.
 *   4. The quaternion derivative factor was 0.9 instead of 0.5
 *      (standard formula qDot = 0.5 * q x [0, gx, gy, gz]).
 *   5. gx, gy, gz must be in rad/s (before, imu.c passed them in deg/s).
 *
 * Functions:
 *   - inv_sqrt(): returns 1/sqrt(x).
 *   - wrap_angle(): keeps an angle inside [-pi, pi].
 *   - quaternion_init(): sets the identity quaternion.
 *   - quaternion_update(): runs one step of the Madgwick filter.
 *   - quaternion_get_roll() / _pitch() / _yaw(): return the Euler angles.
 */

#include "quaternions.h"
#include <math.h>

// FIX 3: beta razonable para un dron (más gyro, menos "tirón" del accel).
// Sube este valor si notas que el filtro no corrige el drift del gyro
// con el tiempo; bájalo si ves mucho jitter durante vibraciones.
#define BETA 0.08f

static float inv_sqrt(float x)
{
    // Nota: esto es una raíz inversa "normal" (no es el hack rápido de
    // Quake pese al nombre original). En ESP32-S2 con FPU de simple
    // precisión, 1.0f/sqrtf(x) ya es suficientemente rápido; no merece
    // la pena el truco de bits (y es menos preciso).
    if (x <= 0.0f) {
        return 0.0f; // evita div/0 o NaN si llega un vector nulo
    }
    return 1.0f / sqrtf(x);
}

static float wrap_angle(float angle)
{
    if (angle > 180.0f) {
        return angle - 360.0f;
    } else if (angle < -180.0f) {
        return angle + 360.0f;
    }
    return angle;
}

void quaternion_init(Quaternion* q)
{
    // FIX 1: cuaternión identidad real = [1,0,0,0].
    q->w = 1.0f;
    q->x = 0.0f;
    q->y = 0.0f;
    q->z = 0.0f;
}

void quaternion_update(Quaternion* q, float gx, float gy, float gz,
                        float ax, float ay, float az, float dt)
{
    // gx,gy,gz DEBEN venir en rad/s (conviértelos en el llamante si tu
    // sensor entrega °/s).

    float accel_norm_sq = ax * ax + ay * ay + az * az;

    // FIX 4: factor 0.5 estándar, no 0.9.
    float qDot1 = 0.5f * (-q->x * gx - q->y * gy - q->z * gz);
    float qDot2 = 0.5f * (q->w * gx + q->y * gz - q->z * gy);
    float qDot3 = 0.5f * (q->w * gy - q->x * gz + q->z * gx);
    float qDot4 = 0.5f * (q->w * gz + q->x * gy - q->y * gx);

    // Solo aplicamos la corrección del acelerómetro si la lectura es
    // válida (evita 0/0 al normalizar).
    if (accel_norm_sq > 1.0e-6f) {
        // FIX 2: normalizar accel directamente con inv_sqrt(suma^2),
        // no con sqrt()+inv_sqrt() encadenados.
        float accel_inv_norm = inv_sqrt(accel_norm_sq);
        float ax_n = ax * accel_inv_norm;
        float ay_n = ay * accel_inv_norm;
        float az_n = az * accel_inv_norm;

        float f1 = 2.0f * (q->x * q->z - q->w * q->y) - ax_n;
        float f2 = 2.0f * (q->w * q->x + q->y * q->z) - ay_n;
        float f3 = 2.0f * (0.5f - q->x * q->x - q->y * q->y) - az_n;

        float j11 = -2.0f * q->y;
        float j12 = 2.0f * q->z;
        float j13 = -2.0f * q->w;
        float j14 = 2.0f * q->x;
        float j21 = 2.0f * q->x;
        float j22 = 2.0f * q->w;
        float j23 = 2.0f * q->z;
        float j24 = 2.0f * q->y;
        float j32 = -4.0f * q->x;
        float j33 = -4.0f * q->y;

        float step1 = j11 * f1 + j21 * f2;
        float step2 = j12 * f1 + j22 * f2 + j32 * f3;
        float step3 = j13 * f1 + j23 * f2 + j33 * f3;
        float step4 = j14 * f1 + j24 * f2;

        // FIX 2 (de nuevo): normalizar el vector paso directamente.
        float step_norm_sq = step1 * step1 + step2 * step2 +
                              step3 * step3 + step4 * step4;
        float step_inv_norm = inv_sqrt(step_norm_sq);
        step1 *= step_inv_norm;
        step2 *= step_inv_norm;
        step3 *= step_inv_norm;
        step4 *= step_inv_norm;

        qDot1 -= BETA * step1;
        qDot2 -= BETA * step2;
        qDot3 -= BETA * step3;
        qDot4 -= BETA * step4;
    }

    q->w += qDot1 * dt;
    q->x += qDot2 * dt;
    q->y += qDot3 * dt;
    q->z += qDot4 * dt;

    // FIX 2 (de nuevo): normalización final del cuaternión, directa.
    float q_norm_sq = q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z;
    float q_inv_norm = inv_sqrt(q_norm_sq);
    q->w *= q_inv_norm;
    q->x *= q_inv_norm;
    q->y *= q_inv_norm;
    q->z *= q_inv_norm;
}

float quaternion_get_roll(const Quaternion* q)
{
    float roll = atan2f(2.0f * (q->w * q->x + q->y * q->z),
                         1.0f - 2.0f * (q->x * q->x + q->y * q->y)) * 180.0f / (float)M_PI;
    return wrap_angle(roll);
}

float quaternion_get_pitch(const Quaternion* q)
{
    float sinp = 2.0f * (q->w * q->y - q->z * q->x);

    if (fabsf(sinp) >= 1.0f) {
        sinp = copysignf(1.0f, sinp);
    }

    float pitch_deg = asinf(sinp) * 180.0f / (float)M_PI;
    return wrap_angle(pitch_deg);
}

float quaternion_get_yaw(const Quaternion* q)
{
    float yaw = atan2f(2.0f * (q->w * q->z + q->x * q->y),
                        1.0f - 2.0f * (q->y * q->y + q->z * q->z)) * 180.0f / (float)M_PI;
    return wrap_angle(yaw);
}
