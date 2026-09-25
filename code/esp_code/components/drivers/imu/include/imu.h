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
    #define I2C_MASTER_SDA_IO    CONFIG_I2C0_PIN_SDA   // I2C SDA pin
    #define I2C_MASTER_SCL_IO    CONFIG_I2C0_PIN_SCL   // I2C SCL pin
    #define I2C_MASTER_FREQ_HZ   400000
    #define ESP_INTR_FLAG_DEFAULT 0
#endif

typedef struct imu_data {
    // Valores convertidos a unidades físicas
    float Acc_lin_X;
    float Acc_lin_Y;
    float Acc_lin_Z;
    float Vel_ang_X;
    float Vel_ang_Y;
    float Vel_ang_Z;
    int64_t Time_stamp;
} IMU;


void imu_init(void);

bool imu_test(void);

esp_err_t get_imu_data(IMU *data);


// ---- Comprobación de estabilidad previa al armado ----

// Inclinación máxima permitida respecto a la vertical (grados)
#define IMU_ARM_MAX_TILT_DEG      10.0f

// Velocidad angular máxima permitida (º/s) para considerar el dron "quieto"
#define IMU_ARM_MAX_GYRO_DPS      5.0f

// Rango aceptable del módulo del vector aceleración (en g). Fuera de este
// rango indica vibración excesiva, manipulación o caída libre.
#define IMU_ARM_ACCEL_MIN_G       0.9f
#define IMU_ARM_ACCEL_MAX_G       1.1f

// Nº de muestras consecutivas que deben cumplir todas las condiciones
#define IMU_ARM_CHECK_SAMPLES     20

// Periodo entre muestras, en ms
#define IMU_ARM_CHECK_PERIOD_MS   10

typedef enum {
    IMU_ARM_STATE_STABLE = 0,     // Apto para armar
    IMU_ARM_STATE_TILTED,         // Inclinación fuera de rango
    IMU_ARM_STATE_MOVING,         // Velocidad angular fuera de rango
    IMU_ARM_STATE_ACCEL_ABNORMAL, // Módulo de aceleración fuera de rango
} imu_arm_state_t;

/**
 * @brief Comprueba, mediante varias muestras consecutivas, si el dron está
 *        estable (nivelado, quieto y sin vibración anómala) y por tanto es
 *        seguro armarlo.
 *
 * Bloquea la tarea llamante durante aprox.
 * IMU_ARM_CHECK_SAMPLES * IMU_ARM_CHECK_PERIOD_MS ms.
 *
 * @param[out] state Resultado del análisis. Solo es válido si la función
 *                    devuelve ESP_OK. En caso de fallo real (IMU no
 *                    inicializada, error de I2C) *state no se modifica.
 * @return ESP_OK si el chequeo se ha podido realizar (independientemente
 *         del resultado, mirar *state).
 *         ESP_ERR_INVALID_ARG si state es NULL.
 *         ESP_ERR_INVALID_STATE si la IMU no está inicializada.
 *         Código de error de I2C/MPU6050 si falla una lectura.
 */
esp_err_t imu_check_stable_for_arming(imu_arm_state_t *state);


#endif // IMU_H

// TODO
//      * Revisar estabilización
