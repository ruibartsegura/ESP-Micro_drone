/**
 * Tests unitarios de attitude_controller.c  (host, sin hardware)   -- v2
 *
 *   1. Filtro complementario
 *   2. LAZO EXTERNO : get_roll_pitch() + cmd_vel_2_RP()   (angulo  -> consigna de velocidad angular)
 *   3. LAZO INTERNO : velocidad angular -> mezcla de motores (roll / pitch / yaw)
 *   4. INTERFAZ CON MOTORES : ids validos, rango 0..100 %, margen de throttle
 *   5. ALTURA       : get_h() + empuje colectivo + check_h_reached()
 *
 * Se compilan las CABECERAS REALES del proyecto (imu.h, height.h, motors.h, system.h,
 * ros_coordinator.h); solo se simulan ESP-IDF/FreeRTOS/micro-ROS (ver mocks/) y las
 * funciones de hardware (get_imu_data, get_height_data, get_attitude, set_motor_speed).
 * Si cambias una firma real, estos tests dejan de compilar.
 *
 * UNIDADES (segun los drivers reales):
 *   IMU.Acc_lin_*  : g, INCLUYE la gravedad (Z ~ +1 g en reposo)   [mpu6050.c: bias_z = mean - 1 g]
 *   IMU.Vel_ang_*  : grados/s
 *   IMU.Time_stamp : NANOSEGUNDOS                                  [mpu6050_read_raw_data]
 *   cmd_vel        : m/s y rad/s (ROS)
 *   altura         : metros
 *   motores        : 0..100 %   (set_motor_speed recibe uint8_t y satura a 100)
 *
 * El estimador de actitud (get_roll_pitch) trabaja en GRADOS, porque integra el giroscopio
 * en grados/s. Si migras todo el controlador a radianes, cambia ANG_UNIT a 1.0f.
 *
 * Los tests que fallan sobre el codigo actual llevan [BUG] en su comentario.
 *
 * Tecnica: se hace #include del .c para ver macros (KP, MAX_ANGLE, ...), funciones y globales
 * sin modificar el codigo de produccion. Se puede probar otra version con:
 *     make test CONTROLLER_SRC=ruta/al/attitude_controller.c
 */

#include "unity.h"
#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>

/* --- Cabeceras reales del proyecto --- */
#include "attitude_controller.h"
#include "imu.h"
#include "height.h"
#include "motors.h"
#include "system.h"
#include "ros_coordinator.h"

/* set_motor_speed real recibe uint8_t: aqui lo interceptamos para ver el valor float que
 * calcula el controlador ANTES de la conversion (asi se detectan negativos y > 100). */
static void mock_set_motor_speed(int id, float speed);
#define set_motor_speed(id, spd) mock_set_motor_speed((id), (spd))

#ifndef CONTROLLER_SRC
#define CONTROLLER_SRC "../attitude_controller.c"
#endif
#include CONTROLLER_SRC

/* Capturamos las constantes fisicas y quitamos las macros de una letra (A, C, g, m, rho). */
static const float K_RHO = rho;
static const float K_A   = A;
static const float K_CD  = C;
static const float K_G   = g;
static const float K_M   = m;
#undef A
#undef C
#undef g
#undef m
#undef rho

/* ------------------------------------------------------------------ */
/*                              MOCKS                                 */
/* ------------------------------------------------------------------ */
static IMU                              mock_imu;
static esp_err_t                        mock_imu_ret;
static geometry_msgs__msg__PoseStamped  mock_h;
static esp_err_t                        mock_h_ret;
static ATTITUDE_TARGET                  mock_target;

#define MAX_IDS 16
static float mval[MAX_IDS];       /* ultimo valor por motor_id */
static int   mcalls[MAX_IDS];     /* nº de llamadas por motor_id */
static int   m_bad_id_calls;

esp_err_t get_imu_data(IMU *data) {
    if (mock_imu_ret == ESP_OK) *data = mock_imu;   /* en fallo NO escribe (como el driver real) */
    return mock_imu_ret;
}
esp_err_t get_height_data(geometry_msgs__msg__PoseStamped *data) {
    if (mock_h_ret == ESP_OK) *data = mock_h;
    return mock_h_ret;
}
ATTITUDE_TARGET get_attitude() { return mock_target; }

static void mock_set_motor_speed(int id, float speed) {
    if (id >= 0 && id < MAX_IDS) { mval[id] = speed; mcalls[id]++; }
    else m_bad_id_calls++;
}

/* ------------------------------------------------------------------ */
/*                             HELPERS                                */
/* ------------------------------------------------------------------ */
#define DT_S        0.01f
#define DT_NS       10000000LL          /* 10 ms en nanosegundos (unidad del driver) */
#define EPS         1e-4f
#define G_MS2       9.80665f
#define RAD2DEG     (180.0f / (float)M_PI)
#define BASE        ((float)throttle_base)

/* Unidad en la que get_roll_pitch() entrega el angulo (grados). Ver cabecera. */
static const float ANG_UNIT = RAD2DEG;  /* grados por radian */

typedef struct { float coll, roll, pitch, yaw; } MixCmd;

/* Ids de motor usados en el ultimo ciclo, en orden ascendente. Devuelve cuantos. */
static int motor_ids(int ids[MAX_IDS]) {
    int n = 0;
    for (int i = 0; i < MAX_IDS; i++) if (mcalls[i] > 0) ids[n++] = i;
    return n;
}

/* Descompone los 4 motores (M1..M4 = ids ascendentes) en sus 4 "ordenes":
 * inversa exacta del mezclador. Funciona tanto con ids 1..4 como 0..3. */
static MixCmd decompose(void) {
    int ids[MAX_IDS];
    int n = motor_ids(ids);
    TEST_ASSERT_EQUAL_INT_MESSAGE(4, n, "el controlador debe comandar exactamente 4 motores");
    float m1 = mval[ids[0]], m2 = mval[ids[1]], m3 = mval[ids[2]], m4 = mval[ids[3]];
    MixCmd c;
    c.coll  = ( m1 + m2 + m3 + m4) / 4.0f;
    c.roll  = ( m1 - m2 - m3 + m4) / 4.0f;
    c.pitch = (-m1 - m2 + m3 + m4) / 4.0f;
    c.yaw   = (-m1 + m2 - m3 + m4) / 4.0f;
    return c;
}

/* Vector gravedad (en g) que mide el acelerometro para un roll/pitch dado (rad). */
static void set_gravity_tilt(IMU *imu, float roll, float pitch) {
    imu->Acc_lin_X = -sinf(pitch);
    imu->Acc_lin_Y =  cosf(pitch) * sinf(roll);
    imu->Acc_lin_Z =  cosf(pitch) * cosf(roll);
}

/* Un ciclo de control con dt = 10 ms. */
static void step(void) {
    mock_imu.Time_stamp += DT_NS;
    control_attitude();
}

/* Punto unico de acoplamiento con el tiempo "previo" que usa get_h():
 * hoy es last_rpy.t_stamp. Si mueves ese estado, solo hay que tocar esta funcion. */
static void prime_height_timebase(int64_t t_prev_ns) { last_rpy.t_stamp = t_prev_ns; }

static void assert_motors_safe(void) {
    for (int i = 0; i < MAX_IDS; i++)
        TEST_ASSERT_TRUE_MESSAGE(mcalls[i] == 0 || mval[i] == 0.0f,
            "con fallo de sensor los motores no deben recibir consignas basadas en datos basura");
}

void setUp(void) {
    memset(&mock_imu, 0, sizeof mock_imu);
    mock_imu.Acc_lin_Z = 1.0f;                 /* IMU real en reposo: +1 g en Z */
    memset(&mock_h, 0, sizeof mock_h);
    memset(&mock_target, 0, sizeof mock_target);
    memset(mval, 0, sizeof mval);
    memset(mcalls, 0, sizeof mcalls);
    m_bad_id_calls = 0;
    mock_imu_ret = ESP_OK;
    mock_h_ret   = ESP_OK;
    memset(&last_rpy, 0, sizeof last_rpy);
    last_h = 0; last_vel_Z = 0; err_h = 0;
}
void tearDown(void) {}

/* ================================================================== */
/*  1. FILTRO COMPLEMENTARIO                                          */
/* ================================================================== */
void test_complementary_alpha_1_returns_first(void) {
    TEST_ASSERT_FLOAT_WITHIN(EPS, 3.0f, complementary_filter(3.0f, -7.0f, 1.0f));
}
void test_complementary_alpha_0_returns_second(void) {
    TEST_ASSERT_FLOAT_WITHIN(EPS, -7.0f, complementary_filter(3.0f, -7.0f, 0.0f));
}
void test_complementary_blend_value(void) {
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.98f, complementary_filter(1.0f, 0.0f, 0.98f));
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.02f, complementary_filter(0.0f, 1.0f, 0.98f));
}

/* ================================================================== */
/*  2. LAZO EXTERNO                                                   */
/* ================================================================== */

/* ---- 2a. Estimacion de roll/pitch: get_roll_pitch() --------------- */

void test_ext_attitude_level_and_still_is_zero(void) {
    float r, p;
    mock_imu.Time_stamp = DT_NS;
    get_roll_pitch(&r, &p, &mock_imu);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, r);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, p);
}

void test_ext_attitude_gyro_integrated_with_nanosecond_timestamps(void) {
    float r, p;
    mock_imu.Time_stamp = DT_NS;          /* dt = 10 ms */
    mock_imu.Vel_ang_X = 30.0f;           /* deg/s */
    mock_imu.Vel_ang_Y = -20.0f;
    get_roll_pitch(&r, &p, &mock_imu);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f,  ALPHA * 30.0f * DT_S, r);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -ALPHA * 20.0f * DT_S, p);
}

/* El acelerometro (en g) da roll/pitch correctos; el filtro converge al angulo de gravedad. */
void test_ext_attitude_accel_tilt_converges_roll(void) {
    float r = 0, p = 0;
    set_gravity_tilt(&mock_imu, 0.2f, 0.0f);
    for (int i = 0; i < 500; i++) {
        mock_imu.Time_stamp += DT_NS;
        get_roll_pitch(&r, &p, &mock_imu);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.3f, 0.2f * ANG_UNIT, r);
    TEST_ASSERT_FLOAT_WITHIN(0.3f, 0.0f, p);
}

void test_ext_attitude_accel_tilt_converges_pitch(void) {
    float r = 0, p = 0;
    set_gravity_tilt(&mock_imu, 0.0f, 0.15f);
    for (int i = 0; i < 500; i++) {
        mock_imu.Time_stamp += DT_NS;
        get_roll_pitch(&r, &p, &mock_imu);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.3f, 0.0f, r);
    TEST_ASSERT_FLOAT_WITHIN(0.3f, 0.15f * ANG_UNIT, p);
}

void test_ext_attitude_updates_state_and_timestamp(void) {
    float r, p;
    mock_imu.Time_stamp = DT_NS;
    mock_imu.Vel_ang_X = 10.0f;
    get_roll_pitch(&r, &p, &mock_imu);
    TEST_ASSERT_FLOAT_WITHIN(EPS, r, last_rpy.roll);
    TEST_ASSERT_FLOAT_WITHIN(EPS, p, last_rpy.pitch);
    TEST_ASSERT_EQUAL_INT64(DT_NS, last_rpy.t_stamp);
}

void test_ext_attitude_first_sample_does_not_blow_up(void) {
    float r, p;
    mock_imu.Time_stamp = 5000000000LL;   /* 5 s desde el arranque, en ns */
    mock_imu.Vel_ang_X = 30.0f;
    get_roll_pitch(&r, &p, &mock_imu);
    TEST_ASSERT_TRUE_MESSAGE(fabsf(r) < 1.0f, "la primera muestra no debe integrar todo el uptime");
}

void test_ext_attitude_zero_dt_is_finite_and_keeps_state(void) {
    float r, p;
    last_rpy.roll = 5.0f;
    last_rpy.t_stamp = 1000000000LL;
    mock_imu.Time_stamp = 1000000000LL;   /* dt = 0 */
    mock_imu.Vel_ang_X = 100.0f;
    get_roll_pitch(&r, &p, &mock_imu);
    TEST_ASSERT_TRUE(isfinite(r) && isfinite(p));
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, ALPHA * 5.0f, r);   /* sin integracion; acel a 0 grados */
}

void test_ext_attitude_negative_dt_is_ignored(void) {
    float r, p;
    last_rpy.roll = 5.0f;
    last_rpy.t_stamp = 2000000000LL;
    mock_imu.Time_stamp = 1000000000LL;   /* 1 s en el pasado */
    mock_imu.Vel_ang_X = 100.0f;
    get_roll_pitch(&r, &p, &mock_imu);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, ALPHA * 5.0f, r);
}

/* ---- 2b. cmd_vel -> roll/pitch objetivo (rad): cmd_vel_2_RP() ----- */

void test_ext_target_zero_velocity_gives_zero_angles(void) {
    float tr = 1, tp = 1;
    cmd_vel_2_RP(&tr, &tp, mock_target.cmd_vel);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, tr);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, tp);
}

/* Modelo: tan(theta) = Cd*rho*A*v^2 / (2*m*g). */
void test_ext_target_matches_drag_model(void) {
    float tr, tp;
    mock_target.cmd_vel.linear.x = 2.0;
    float expected = atanf((K_CD * K_RHO * K_A * 4.0f) / (2.0f * K_M * K_G));
    cmd_vel_2_RP(&tr, &tp, mock_target.cmd_vel);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, expected, fabsf(tr) + fabsf(tp)); /* todo el angulo en un solo eje */
}

void test_ext_target_forward_velocity_maps_to_pitch_only(void) {
    float tr, tp;
    mock_target.cmd_vel.linear.x = 1.5;
    cmd_vel_2_RP(&tr, &tp, mock_target.cmd_vel);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, tr);
    TEST_ASSERT_TRUE(fabsf(tp) > 1e-3f);
}

void test_ext_target_lateral_velocity_maps_to_roll_only(void) {
    float tr, tp;
    mock_target.cmd_vel.linear.y = 1.5;
    cmd_vel_2_RP(&tr, &tp, mock_target.cmd_vel);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, tp);
    TEST_ASSERT_TRUE(fabsf(tr) > 1e-3f);
}

void test_ext_target_is_odd_symmetric(void) {
    float r1, p1, r2, p2;
    mock_target.cmd_vel.linear.x = 1.0;  mock_target.cmd_vel.linear.y = 0.5;
    cmd_vel_2_RP(&r1, &p1, mock_target.cmd_vel);
    mock_target.cmd_vel.linear.x = -1.0; mock_target.cmd_vel.linear.y = -0.5;
    cmd_vel_2_RP(&r2, &p2, mock_target.cmd_vel);
    TEST_ASSERT_FLOAT_WITHIN(EPS, -r1, r2);
    TEST_ASSERT_FLOAT_WITHIN(EPS, -p1, p2);
}

void test_ext_target_monotonic_with_speed(void) {
    float r1, p1, r2, p2;
    mock_target.cmd_vel.linear.x = 1.0; cmd_vel_2_RP(&r1, &p1, mock_target.cmd_vel);
    mock_target.cmd_vel.linear.x = 2.0; cmd_vel_2_RP(&r2, &p2, mock_target.cmd_vel);
    TEST_ASSERT_TRUE(fabsf(r2) + fabsf(p2) > fabsf(r1) + fabsf(p1));
}

void test_ext_target_clamps_at_max_angle_positive(void) {
    float tr, tp;
    mock_target.cmd_vel.linear.x = 50.0; mock_target.cmd_vel.linear.y = 50.0;
    cmd_vel_2_RP(&tr, &tp, mock_target.cmd_vel);
    TEST_ASSERT_TRUE(tr <= MAX_ANGLE + EPS && tp <= MAX_ANGLE + EPS);
    TEST_ASSERT_FLOAT_WITHIN(EPS, MAX_ANGLE, fmaxf(tr, tp));
}

void test_ext_target_clamps_at_max_angle_negative(void) {
    float tr, tp;
    mock_target.cmd_vel.linear.x = -50.0; mock_target.cmd_vel.linear.y = -50.0;
    cmd_vel_2_RP(&tr, &tp, mock_target.cmd_vel);
    TEST_ASSERT_TRUE_MESSAGE(tr >= -MAX_ANGLE - EPS, "roll sin saturar en negativo");
    TEST_ASSERT_TRUE_MESSAGE(tp >= -MAX_ANGLE - EPS, "pitch sin saturar en negativo");
}

/* ---- 2c. Salida del lazo externo -> entrada del interno ----------- */

void test_ext_cascade_angle_error_feeds_rate_loop(void) {
    float tr, tp;
    mock_target.cmd_vel.linear.x = 2.0;
    mock_target.cmd_vel.linear.y = 1.0;
    cmd_vel_2_RP(&tr, &tp, mock_target.cmd_vel);   /* rad */
    step();
    MixCmd c = decompose();
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, KP * KP * tr * ANG_UNIT, c.roll);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, KP * KP * tp * ANG_UNIT, c.pitch);
}

/* Si el dron esta inclinado y no se pide nada, el control debe oponerse a la inclinacion. */
void test_ext_cascade_tilt_is_opposed(void) {
    set_gravity_tilt(&mock_imu, 0.2f, 0.0f);      /* roll medido > 0 */
    step();
    TEST_ASSERT_TRUE_MESSAGE(decompose().roll < 0.0f, "roll medido > 0 => la orden de roll debe ser < 0");
}

void test_ext_cascade_zero_error_when_tilt_equals_target(void) {
    float tr, tp;
    mock_target.cmd_vel.linear.y = 3.0;
    mock_target.cmd_vel.linear.x = 2.0;
    cmd_vel_2_RP(&tr, &tp, mock_target.cmd_vel);   /* rad */
    set_gravity_tilt(&mock_imu, tr, tp);
    for (int i = 0; i < 500; i++) step();
    MixCmd c = decompose();
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, c.roll);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, c.pitch);
}

/* ================================================================== */
/*  3. LAZO INTERNO (velocidad angular -> motores)                    */
/* ================================================================== */

void test_int_hover_baseline_all_motors_equal_throttle_base(void) {
    step();
    int ids[MAX_IDS]; int n = motor_ids(ids);
    TEST_ASSERT_EQUAL_INT(4, n);
    for (int i = 0; i < n; i++)
        TEST_ASSERT_FLOAT_WITHIN(EPS, BASE, mval[ids[i]]);
}

void test_int_roll_rate_error_only_affects_roll_channel(void) {
    float gyro = -5.0f;                               /* deg/s medidos => error de tasa positivo */
    mock_imu.Vel_ang_X = gyro;
    step();
    float roll_est = ALPHA * gyro * DT_S;             /* grados, lo que integra el lazo externo */
    float expected = KP * (KP * (0.0f - roll_est) - gyro);
    MixCmd c = decompose();
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, expected, c.roll);
    TEST_ASSERT_TRUE(c.roll > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, c.pitch);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, c.yaw);
}

void test_int_pitch_rate_error_only_affects_pitch_channel(void) {
    float gyro = 5.0f;
    mock_imu.Vel_ang_Y = gyro;
    step();
    float pitch_est = ALPHA * gyro * DT_S;
    float expected = KP * (KP * (0.0f - pitch_est) - gyro);
    MixCmd c = decompose();
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, expected, c.pitch);
    TEST_ASSERT_TRUE(c.pitch < 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, c.roll);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, c.yaw);
}

void test_int_yaw_rate_tracks_cmd_vel_angular_z(void) {
    mock_target.cmd_vel.angular.z = 0.1;              /* rad/s = 5.73 deg/s */
    mock_imu.Vel_ang_Z = 2.0f;                        /* deg/s */
    step();
    MixCmd c = decompose();
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, KP * (0.1f * RAD2DEG - 2.0f), c.yaw);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, c.roll);
    TEST_ASSERT_FLOAT_WITHIN(EPS, 0.0f, c.pitch);
}

void test_int_yaw_zero_error_gives_zero_yaw_command(void) {
    mock_target.cmd_vel.angular.z = 0.1;
    mock_imu.Vel_ang_Z = 0.1f * RAD2DEG;
    step();
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 0.0f, decompose().yaw);
}

void test_int_response_is_proportional_to_rate_error(void) {
    mock_imu.Vel_ang_Z = -2.0f; step(); float y1 = decompose().yaw;
    setUp();
    mock_imu.Vel_ang_Z = -4.0f; step(); float y2 = decompose().yaw;
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2.0f * y1, y2);
}

/* El mezclador solo reparte: las ordenes diferenciales no deben cambiar el empuje total
 * (senales pequenas para no saturar). */
void test_int_mixer_preserves_collective_thrust(void) {
    mock_imu.Vel_ang_X = 1.0f; mock_imu.Vel_ang_Y = -2.0f; mock_imu.Vel_ang_Z = 0.5f;
    mock_target.cmd_vel.linear.x = 1.0; mock_target.cmd_vel.linear.y = -1.0;
    mock_target.cmd_vel.angular.z = 0.05;
    step();
    int ids[MAX_IDS]; int n = motor_ids(ids);
    float sum = 0; for (int i = 0; i < n; i++) sum += mval[ids[i]];
    TEST_ASSERT_FLOAT_WITHIN(1e-2f, 4.0f * BASE, sum);
}

void test_int_each_motor_commanded_once_per_cycle(void) {
    step();
    int ids[MAX_IDS]; int n = motor_ids(ids);
    for (int i = 0; i < n; i++)
        TEST_ASSERT_EQUAL_INT_MESSAGE(1, mcalls[ids[i]], "set_motor_speed duplicado en control_attitude");
}

void test_int_outputs_are_finite_with_zero_dt(void) {
    mock_imu.Vel_ang_X = 10.0f;
    control_attitude();               /* mismo timestamp => dt = 0 */
    control_attitude();
    int ids[MAX_IDS]; int n = motor_ids(ids);
    for (int i = 0; i < n; i++) TEST_ASSERT_TRUE(isfinite(mval[ids[i]]));
}

void test_int_imu_read_failure_does_not_drive_motors(void) {
    mock_imu_ret = ESP_ERR_INVALID_STATE;
    step();
    assert_motors_safe();
}

void test_int_height_read_failure_does_not_drive_motors(void) {
    mock_h_ret = ESP_ERR_INVALID_STATE;
    step();
    assert_motors_safe();
}

/* ================================================================== */
/*  4. INTERFAZ CON LOS MOTORES  (motors.h real)                      */
/* ================================================================== */

void test_motors_ids_are_within_driver_range(void) {
    step();
    int ids[MAX_IDS]; int n = motor_ids(ids);
    TEST_ASSERT_EQUAL_INT(0, m_bad_id_calls);
    TEST_ASSERT_EQUAL_INT_MESSAGE(N_MOTORS, n, "debe comandar N_MOTORS motores distintos");
    for (int i = 0; i < n; i++)
        TEST_ASSERT_TRUE_MESSAGE(ids[i] < N_MOTORS, "motor_id fuera de 0..N_MOTORS-1: el driver lo descarta");
    TEST_ASSERT_EQUAL_INT_MESSAGE(0, ids[0], "el driver empieza en motor_id = 0");
}

void test_motors_outputs_are_saturated_to_0_100(void) {
    mock_imu.Vel_ang_X = -400.0f; mock_imu.Vel_ang_Y = 400.0f; mock_imu.Vel_ang_Z = -400.0f;
    mock_target.cmd_vel.angular.z = 5.0;
    step();
    int ids[MAX_IDS]; int n = motor_ids(ids);
    TEST_ASSERT_EQUAL_INT(4, n);
    for (int i = 0; i < n; i++) {
        TEST_ASSERT_TRUE(isfinite(mval[ids[i]]));
        TEST_ASSERT_TRUE_MESSAGE(mval[ids[i]] >= 0.0f && mval[ids[i]] <= 100.0f,
                                 "consigna de motor fuera de 0..100 %");
    }
}

void test_motors_hover_throttle_leaves_headroom(void) {
    TEST_ASSERT_TRUE_MESSAGE(throttle_base <= 90, "throttle_base debe dejar margen respecto al 100 %");
    TEST_ASSERT_TRUE(throttle_base > 0);
}

/* ================================================================== */
/*  5. ALTURA                                                         */
/* ================================================================== */

void test_h_returns_baro_when_at_rest_and_consistent(void) {
    last_h = 1.0f;
    prime_height_timebase(0);
    mock_imu.Time_stamp = DT_NS;                       /* reposo: Acc_Z = 1 g */
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f, get_h(1.0f, &mock_imu));
}

void test_h_baro_step_first_sample_is_weighted_by_one_minus_alpha(void) {
    prime_height_timebase(0);
    mock_imu.Time_stamp = DT_NS;
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 1.0f - ALPHA, get_h(1.0f, &mock_imu));
}

/* a = +1 m/s^2 durante 50 ms (Acc_Z = 1 g + 1 m/s^2): v = 0.05 m/s, h = v*dt = 0.0025 m. */
void test_h_integrates_vertical_acceleration_in_si(void) {
    mock_imu.Acc_lin_Z = 1.0f + 1.0f / G_MS2;
    prime_height_timebase(0);
    mock_imu.Time_stamp = 50000000LL;                  /* 50 ms en ns */
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0025f, get_h(0.0025f, &mock_imu));
}

void test_h_persists_state_between_calls(void) {
    mock_imu.Acc_lin_Z = 1.0f + 1.0f / G_MS2;
    prime_height_timebase(0);
    mock_imu.Time_stamp = 50000000LL;
    float h = get_h(0.0025f, &mock_imu);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, h,      last_h);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.05f,  last_vel_Z);
}

void test_h_converges_to_baro_reading(void) {
    float h = 0;
    int64_t t = 0;
    for (int i = 0; i < 1000; i++) {
        prime_height_timebase(t);                      /* lo que dejaria el ciclo anterior */
        t += DT_NS;
        mock_imu.Time_stamp = t;
        h = get_h(1.0f, &mock_imu);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.0f, h);
}

void test_h_estimate_tracks_baro_inside_control_loop(void) {
    mock_h.pose.position.z = 1.0;
    mock_target.h = 1.0f;
    for (int i = 0; i < 300; i++) step();
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 1.0f, last_h);
}

/* En reposo con el barometro en 0 la altura estimada debe seguir en 0 (no derivar por g). */
void test_h_no_drift_at_rest(void) {
    mock_h.pose.position.z = 0.0;
    mock_target.h = 0.0f;
    for (int i = 0; i < 1000; i++) step();
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, last_h);
}

void test_h_below_target_increases_collective_thrust(void) {
    mock_h.pose.position.z = 0.0;
    mock_target.h = 1.0f;
    step();
    TEST_ASSERT_TRUE_MESSAGE(decompose().coll > BASE + EPS, "debe subir el empuje para ascender");
}

void test_h_above_target_decreases_collective_thrust(void) {
    mock_h.pose.position.z = 1.0;
    mock_target.h = 0.0f;
    for (int i = 0; i < 20; i++) step();
    TEST_ASSERT_TRUE_MESSAGE(decompose().coll < BASE - EPS, "debe bajar el empuje para descender");
}

void test_h_at_target_holds_hover_throttle(void) {
    mock_h.pose.position.z = 1.0;
    mock_target.h = 1.0f;
    for (int i = 0; i < 300; i++) step();
    TEST_ASSERT_FLOAT_WITHIN(0.5f, BASE, decompose().coll);
}

/* ---- check_h_reached(): tolerancia 5 % de la consigna (ALTITUDE_ERROR en system.c) ---- */

void test_h_reached_is_false_when_far_from_target(void) {
    mock_h.pose.position.z = 0.0;
    mock_target.h = 1.0f;
    step();
    TEST_ASSERT_FALSE(check_h_reached());
}

void test_h_reached_is_true_when_at_target(void) {
    mock_h.pose.position.z = 1.0;
    mock_target.h = 1.0f;
    for (int i = 0; i < 300; i++) step();
    TEST_ASSERT_TRUE(check_h_reached());
}

void test_h_reached_true_within_5_percent(void) {
    mock_h.pose.position.z = 0.97;                    /* 3 cm de error sobre 1 m */
    mock_target.h = 1.0f;
    for (int i = 0; i < 300; i++) step();
    TEST_ASSERT_TRUE(check_h_reached());
}

void test_h_reached_false_outside_5_percent(void) {
    mock_h.pose.position.z = 0.90;
    mock_target.h = 1.0f;
    for (int i = 0; i < 300; i++) step();
    TEST_ASSERT_FALSE(check_h_reached());
}

/* ================================================================== */
int main(void) {
    UNITY_BEGIN();

    printf("/* Filtro complementario */\n");
    RUN_TEST(test_complementary_alpha_1_returns_first);
    RUN_TEST(test_complementary_alpha_0_returns_second);
    RUN_TEST(test_complementary_blend_value);
    printf("\n");

    printf("/* Lazo externo */\n");
    RUN_TEST(test_ext_attitude_level_and_still_is_zero);
    RUN_TEST(test_ext_attitude_gyro_integrated_with_nanosecond_timestamps);
    RUN_TEST(test_ext_attitude_accel_tilt_converges_roll);
    RUN_TEST(test_ext_attitude_accel_tilt_converges_pitch);
    RUN_TEST(test_ext_attitude_updates_state_and_timestamp);
    RUN_TEST(test_ext_attitude_first_sample_does_not_blow_up);
    RUN_TEST(test_ext_attitude_zero_dt_is_finite_and_keeps_state);
    RUN_TEST(test_ext_attitude_negative_dt_is_ignored);
    RUN_TEST(test_ext_target_zero_velocity_gives_zero_angles);
    RUN_TEST(test_ext_target_matches_drag_model);
    RUN_TEST(test_ext_target_forward_velocity_maps_to_pitch_only);
    RUN_TEST(test_ext_target_lateral_velocity_maps_to_roll_only);
    RUN_TEST(test_ext_target_is_odd_symmetric);
    RUN_TEST(test_ext_target_monotonic_with_speed);
    RUN_TEST(test_ext_target_clamps_at_max_angle_positive);
    RUN_TEST(test_ext_target_clamps_at_max_angle_negative);
    RUN_TEST(test_ext_cascade_angle_error_feeds_rate_loop);
    RUN_TEST(test_ext_cascade_tilt_is_opposed);
    RUN_TEST(test_ext_cascade_zero_error_when_tilt_equals_target);
    printf("\n");

    printf("/* Lazo interno */\n");
    RUN_TEST(test_int_hover_baseline_all_motors_equal_throttle_base);
    RUN_TEST(test_int_roll_rate_error_only_affects_roll_channel);
    RUN_TEST(test_int_pitch_rate_error_only_affects_pitch_channel);
    RUN_TEST(test_int_yaw_rate_tracks_cmd_vel_angular_z);
    RUN_TEST(test_int_yaw_zero_error_gives_zero_yaw_command);
    RUN_TEST(test_int_response_is_proportional_to_rate_error);
    RUN_TEST(test_int_mixer_preserves_collective_thrust);
    RUN_TEST(test_int_each_motor_commanded_once_per_cycle);
    RUN_TEST(test_int_outputs_are_finite_with_zero_dt);
    RUN_TEST(test_int_imu_read_failure_does_not_drive_motors);
    RUN_TEST(test_int_height_read_failure_does_not_drive_motors);
    printf("\n");

    printf("/* Interfaz con motores */\n");
    RUN_TEST(test_motors_ids_are_within_driver_range);
    RUN_TEST(test_motors_outputs_are_saturated_to_0_100);
    RUN_TEST(test_motors_hover_throttle_leaves_headroom);
    printf("\n");

    printf("/* Altura */\n");
    RUN_TEST(test_h_returns_baro_when_at_rest_and_consistent);
    RUN_TEST(test_h_baro_step_first_sample_is_weighted_by_one_minus_alpha);
    RUN_TEST(test_h_integrates_vertical_acceleration_in_si);
    RUN_TEST(test_h_persists_state_between_calls);
    RUN_TEST(test_h_converges_to_baro_reading);
    RUN_TEST(test_h_estimate_tracks_baro_inside_control_loop);
    RUN_TEST(test_h_no_drift_at_rest);
    RUN_TEST(test_h_below_target_increases_collective_thrust);
    RUN_TEST(test_h_above_target_decreases_collective_thrust);
    RUN_TEST(test_h_at_target_holds_hover_throttle);
    RUN_TEST(test_h_reached_is_false_when_far_from_target);
    RUN_TEST(test_h_reached_is_true_when_at_target);
    RUN_TEST(test_h_reached_true_within_5_percent);
    RUN_TEST(test_h_reached_false_outside_5_percent);

    return UNITY_END();
}
