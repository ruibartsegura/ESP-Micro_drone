/********************************************************************************************
 * Project: MPU6050 ESP32 Sensor Interface
 * Original author: Muhammad Idrees
 * Corregido: normalización, beta, factor de integración, unidades.
 ********************************************************************************************/

#ifndef QUATERNIONS_H
#define QUATERNIONS_H

#include <math.h>

// Quaternion structure
typedef struct {
    float w;
    float x;
    float y;
    float z;
} Quaternion;

void quaternion_init(Quaternion* q);

/**
 * @brief Filtro Madgwick.
 * @param gx,gy,gz Velocidad angular en RAD/S (¡no grados/s!).
 * @param ax,ay,az Aceleración en cualquier unidad (se normaliza internamente),
 *                  típicamente en g o m/s².
 * @param dt        Delta de tiempo real en segundos desde la última llamada.
 */
void quaternion_update(Quaternion* q, float gx, float gy, float gz,
                        float ax, float ay, float az, float dt);

float quaternion_get_roll(const Quaternion* q);
float quaternion_get_pitch(const Quaternion* q);
float quaternion_get_yaw(const Quaternion* q);

#endif // QUATERNIONS_H
