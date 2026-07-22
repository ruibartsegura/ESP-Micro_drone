/**
 * Made by Rui B.S.
 * Date: 23/05/2025
 * email: rui.bartolome@gmail.com
 *
 * CORRECCIONES (ver "FIX:"):
 *  1. dt real medido con esp_timer_get_time() en vez de 0.01f fijo.
 *  2. Gyro convertido de grados/s a rad/s antes de pasarlo al filtro
 *     (quaternion_update espera rad/s).
 *  3. Bias de calibracion pasado explicito a mpu6050_convert_* (antes
 *     se calculaba y se descartaba, ver mpu6050.c).
 *  4. NUEVO: el muestreo del sensor + la actualizacion del filtro ya
 *     no dependen de la frecuencia a la que otro modulo (ros_coordinator,
 *     a 1Hz) llame a imu_get_data(). Corren en su propia tarea a alta
 *     frecuencia (IMU_TASK_PERIOD_MS). imu_get_data() ahora solo copia
 *     de forma segura el ultimo resultado ya calculado.
 *
 *     Antes, con la IMU muestreada a 1Hz, el dt real (~1s) siempre
 *     superaba el clamp de seguridad de 0.5s y se forzaba a 0.01s en
 *     cada actualizacion -> el filtro integraba ~1% de la rotacion
 *     real por llamada y la correccion del acelerometro (escalada
 *     tambien por ese dt) casi no corregia nada. El cuaternion se
 *     quedaba pegado cerca de la identidad (orientacion ~0 siempre).
 *     Un filtro tipo Madgwick necesita correr a frecuencia alta
 *     (decenas-cientos de Hz), no al ritmo de publicacion ROS.
 */
#include "imu.h"
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"

#define DEG_TO_RAD (float)(M_PI / 180.0)

// Frecuencia de muestreo/filtrado interna, independiente de a que
// ritmo se publique por ROS. 100Hz es un buen punto de partida para
// un Madgwick con MPU6050 en un dron pequeno.
#define IMU_TASK_PERIOD_MS 10

static bool is_init = false;

// Bias de calibracion (constantes tras imu_init(), no necesitan lock)
static float accel_bias[3] = {0.0f, 0.0f, 0.0f};
static float gyro_bias[3]  = {0.0f, 0.0f, 0.0f};

// Estado del filtro: solo lo toca imu_task(), no necesita lock.
static Quaternion q;
static int64_t last_update_us = 0;

// Ultimo resultado calculado, compartido entre imu_task() (escritor)
// y cualquier tarea que llame a imu_get_data() (lectores). Protegido
// por una seccion critica corta (solo copia de struct, sin I/O).
static IMU latest_data;
static portMUX_TYPE data_mux = portMUX_INITIALIZER_UNLOCKED;

static void imu_sample_and_filter(void)
{
    esp_err_t ret;
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    float accel_x_g, accel_y_g, accel_z_g;
    float gyro_x_dps, gyro_y_dps, gyro_z_dps;

    ret = mpu6050_read_raw_data(I2C_MASTER_NUM,
                                 &accel_x, &accel_y, &accel_z,
                                 &gyro_x, &gyro_y, &gyro_z);
    if (ret != ESP_OK) {
        return; // se conserva el ultimo dato valido, no se pisa con basura
    }

    mpu6050_convert_accel(accel_x, accel_y, accel_z, accel_bias,
                           &accel_x_g, &accel_y_g, &accel_z_g);
    mpu6050_convert_gyro(gyro_x, gyro_y, gyro_z, gyro_bias,
                          &gyro_x_dps, &gyro_y_dps, &gyro_z_dps);

    int64_t now_us = esp_timer_get_time();
    float dt = (now_us - last_update_us) / 1000000.0f;
    last_update_us = now_us;

    // Salvaguarda real: aqui si tiene sentido, porque esta funcion se
    // llama cada IMU_TASK_PERIOD_MS (~10ms). Si dt se dispara muy por
    // encima de eso es una anomalia real (tarea bloqueada, I2C
    // colgado, etc.), no el ritmo normal de publicacion de otro modulo.
    if (dt <= 0.0f || dt > (5.0f * IMU_TASK_PERIOD_MS / 1000.0f)) {
        dt = IMU_TASK_PERIOD_MS / 1000.0f;
    }

    quaternion_update(&q,
                       gyro_x_dps * DEG_TO_RAD,
                       gyro_y_dps * DEG_TO_RAD,
                       gyro_z_dps * DEG_TO_RAD,
                       accel_x_g, accel_y_g, accel_z_g, dt);

    IMU sample = {
        .AcX_g = accel_x_g,
        .AcY_g = accel_y_g,
        .AcZ_g = accel_z_g,
        .GyX_dps = gyro_x_dps,
        .GyY_dps = gyro_y_dps,
        .GyZ_dps = gyro_z_dps,
        .q = q,
    };

    portENTER_CRITICAL(&data_mux);
    latest_data = sample;
    portEXIT_CRITICAL(&data_mux);
}

static void imu_task(void *arg) {
    (void)arg;

    int log_counter = 0;

    while (1) {
        imu_sample_and_filter();

        if (++log_counter >= 500) {
            log_counter = 0;
            UBaseType_t free_words = uxTaskGetStackHighWaterMark(NULL);
            ESP_LOGI("imu_task", "stack libre (min historico): %u bytes",
                     (unsigned)(free_words * sizeof(StackType_t)));
        }

        vTaskDelay(pdMS_TO_TICKS(IMU_TASK_PERIOD_MS));
    }
}

void imu_init(void) {
    if (is_init) {
        return;
    }

    esp_err_t ret;

    // Initialize I2C
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        return;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, ESP_INTR_FLAG_DEFAULT);
    if (ret != ESP_OK) {
        return;
    }

    // Initialize MPU6050
    ret = mpu6050_init(I2C_MASTER_NUM);
    if (ret != ESP_OK) {
        return;
    }

    // Calibrate the MPU6050 (el dron debe estar quieto y nivelado aqui)
    ret = mpu6050_calibrate(I2C_MASTER_NUM, accel_bias, gyro_bias);
    if (ret != ESP_OK) {
        return;
    }

    quaternion_init(&q);
    last_update_us = esp_timer_get_time();

    memset(&latest_data, 0, sizeof(latest_data));
    latest_data.q = q;

    // FIX 4: tarea dedicada a muestrear+filtrar a alta frecuencia,
    // desacoplada del ritmo de publicacion ROS.
    xTaskCreate(imu_task, "imu_task", CONFIG_IMU_TASK_STACK, NULL, CONFIG_IMU_TASK_PRIO, NULL);

    is_init = true;
}

bool imu_test(void)
{
    return is_init;
}

esp_err_t imu_get_data(IMU *data) {
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!is_init) {
        return ESP_ERR_INVALID_STATE;
    }

    // FIX 4: ya no se lee el sensor aqui. Solo copiamos el ultimo
    // resultado que calculo imu_task(), de forma segura.
    portENTER_CRITICAL(&data_mux);
    *data = latest_data;
    portEXIT_CRITICAL(&data_mux);

    return ESP_OK;
}

/**
 * @brief Angulo entre el eje Z del cuerpo y el eje Z del mundo (grados),
 *        es decir, la inclinacion respecto a la vertical, independiente
 *        del yaw.
 */
static float imu_quaternion_tilt_deg(const Quaternion *q_ptr)
{
    float cos_tilt = 1.0f - 2.0f * (q_ptr->x * q_ptr->x + q_ptr->y * q_ptr->y);

    if (cos_tilt > 1.0f) {
        cos_tilt = 1.0f;
    } else if (cos_tilt < -1.0f) {
        cos_tilt = -1.0f;
    }

    return acosf(cos_tilt) * (180.0f / (float)M_PI);
}

esp_err_t imu_check_stable_for_arming(imu_arm_state_t *state) {
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!is_init) {
        return ESP_ERR_INVALID_STATE;
    }

    for (int i = 0; i < IMU_ARM_CHECK_SAMPLES; i++) {
        IMU sample;
        esp_err_t ret = imu_get_data(&sample);
        if (ret != ESP_OK) {
            return ret;
        }

        float gyro_mag = sqrtf(sample.GyX_dps * sample.GyX_dps +
                                sample.GyY_dps * sample.GyY_dps +
                                sample.GyZ_dps * sample.GyZ_dps);
        if (gyro_mag > IMU_ARM_MAX_GYRO_DPS) {
            *state = IMU_ARM_STATE_MOVING;
            return ESP_OK;
        }

        float accel_mag = sqrtf(sample.AcX_g * sample.AcX_g +
                                 sample.AcY_g * sample.AcY_g +
                                 sample.AcZ_g * sample.AcZ_g);
        if (accel_mag < IMU_ARM_ACCEL_MIN_G || accel_mag > IMU_ARM_ACCEL_MAX_G) {
            *state = IMU_ARM_STATE_ACCEL_ABNORMAL;
            return ESP_OK;
        }

        float tilt_deg = imu_quaternion_tilt_deg(&sample.q);
        if (tilt_deg > IMU_ARM_MAX_TILT_DEG) {
            *state = IMU_ARM_STATE_TILTED;
            return ESP_OK;
        }

        vTaskDelay(pdMS_TO_TICKS(IMU_ARM_CHECK_PERIOD_MS));
    }

    *state = IMU_ARM_STATE_STABLE;
    return ESP_OK;
}