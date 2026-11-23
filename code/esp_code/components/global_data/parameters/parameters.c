/**
 * Made by Rui B.S.
 * Date: 28/06/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Parameters that can change at runtime (flight, controller gains and
 *   odometry). The default values come from menuconfig; ROS (drone/params)
 *   and the serial console (tuning.c) can change them. Every access is
 *   protected with a critical section, so it is thread safe (like state.c).
 *
 * Functions:
 *   - params_init(): goes back to the menuconfig values.
 *   - get_default_params(): returns the menuconfig values.
 *   - params_keep_all(): fills a params_t with PARAM_KEEP.
 *   - params_update(): changes the fields that are not PARAM_KEEP, only if
 *     all of them are in range.
 *   - params_find(): looks for a parameter by name.
 *   - get_all_params() / set_all_params(): read / write the whole struct.
 *   - set_max_velocity() / get_max_velocity(): maximum velocity.
 *   - get_gains() / set_gains(): controller gains.
 *   - get_odom_params() / set_odom_params(): odometry parameters.
 */

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <strings.h>

#include "freertos/FreeRTOS.h"

#include "parameters.h"

#define FIELD(p, i) ((float *)((char *)(p) + param_info[i].offset))
#define CFIELD(p, i) ((const float *)((const char *)(p) + param_info[i].offset))

// Menuconfig values (menuconfig only has integers: they are scaled here)
static const params_t defaults = {
    .flight = {
        .vel_max = CONFIG_VEL_MAX / 100.0f, // cm/s -> m/s
    },
    .gains = {
        .kp_h            = CONFIG_KP_H / 1000.0f,
        .kd_h            = CONFIG_KD_H / 1000.0f,
        .kp_vel          = CONFIG_KP_VEL / 1000.0f,
        .kp_angle        = CONFIG_KP_ANGLE / 1000.0f,
        .kp_rate         = CONFIG_KP_RATE / 1000.0f,
        .ki_rate         = CONFIG_KI_RATE / 1000.0f,
        .kp_yaw_rate     = CONFIG_KP_YAW_RATE / 1000.0f,
        .max_rate        = CONFIG_MAX_POWER_RATE,
        .max_pow_rp      = CONFIG_MAX_POWER_RP,
        .base            = CONFIG_THROTTLE_BASE,
        .alpha           = CONFIG_ALPHA / 100.0f,
        .max_angle       = CONFIG_MAX_TILT_ANGLE / 100.0f,
        .max_pow_yaw     = CONFIG_MAX_POWER_YAW,
        .pitch_deviation = CONFIG_PITCH_DEVIATION / 10.0f,
        .roll_deviation  = CONFIG_ROLL_DEVIATION / 10.0f,
    },
    .odom = {
        .k_drag = CONFIG_K_DRAG / 1000.0f,
    },
};

// Names (the same of my_msgs/msg/Params and of the serial console) and ranges
#define P(field) offsetof(params_t, field)

const param_info_t param_info[] = {
    { "v_max",           P(flight.vel_max),       0.0f,   10.0f },
    { "kp_h",            P(gains.kp_h),           0.0f, 5000.0f },
    { "kd_h",            P(gains.kd_h),           0.0f, 5000.0f },
    { "kp_vel",          P(gains.kp_vel),         0.0f,    1.0f },
    { "kp_angle",        P(gains.kp_angle),       0.0f,  100.0f },
    { "kp_rate",         P(gains.kp_rate),        0.0f,  500.0f },
    { "ki_rate",         P(gains.ki_rate),        0.0f,  100.0f },
    { "kp_yaw_rate",     P(gains.kp_yaw_rate),    0.0f,  100.0f },
    { "max_rate",        P(gains.max_rate),       0.0f, 2000.0f },
    { "max_pow_rp",      P(gains.max_pow_rp),     0.0f, 3000.0f },
    { "base",            P(gains.base),           0.0f, 3000.0f },
    { "alpha",           P(gains.alpha),          0.0f,    1.0f },
    { "max_angle",       P(gains.max_angle),      0.0f,   1.57f },
    { "max_pow_yaw",     P(gains.max_pow_yaw),    0.0f, 1000.0f },
    { "pitch_deviation", P(gains.pitch_deviation), 0.0f, 100.0f },
    { "roll_deviation",  P(gains.roll_deviation),  0.0f, 100.0f },
    { "k_drag",          P(odom.k_drag),          0.001f, 100.0f }, // > 0: v = a / k_drag
};
const size_t n_param_info = sizeof(param_info) / sizeof(param_info[0]);

// Every field of params_t must be in param_info[]
#define N_INFO (sizeof(param_info) / sizeof(param_info[0]))
_Static_assert(sizeof(params_t) == N_INFO * sizeof(float),
               "param_info[] must have one entry for each field of params_t");

static params_t params = defaults; // Valid before params_init()
static portMUX_TYPE p_lock = portMUX_INITIALIZER_UNLOCKED;


// ============================================================
//                      Whole struct
// ============================================================
void params_init(void) {
    portENTER_CRITICAL(&p_lock);
    params = defaults;
    portEXIT_CRITICAL(&p_lock);
}

void get_default_params(params_t *out) {
    *out = defaults;
}

void params_keep_all(params_t *p) {
    for (size_t i = 0; i < n_param_info; i++) {
        *FIELD(p, i) = PARAM_KEEP;
    }
}

const param_info_t *params_find(const char *name) {
    for (size_t i = 0; i < n_param_info; i++) {
        if (strcasecmp(name, param_info[i].name) == 0) {
            return &param_info[i];
        }
    }
    return NULL;
}

// Changes the fields of "in" that are not negative (PARAM_KEEP). If one of
// them is out of range (or NaN / inf) nothing changes and it returns its name
const char *params_update(const params_t *in) {
    // Check first, out of the critical section
    for (size_t i = 0; i < n_param_info; i++) {
        float v = *CFIELD(in, i);
        if (isfinite(v) && v < 0.0f) {
            continue; // PARAM_KEEP
        }
        if (!isfinite(v) || v < param_info[i].min || v > param_info[i].max) {
            return param_info[i].name;
        }
    }

    portENTER_CRITICAL(&p_lock);
    for (size_t i = 0; i < n_param_info; i++) {
        float v = *CFIELD(in, i);
        if (v >= 0.0f) {
            *FIELD(&params, i) = v;
        }
    }
    portEXIT_CRITICAL(&p_lock);
    return NULL;
}

void get_all_params(params_t *out) {
    portENTER_CRITICAL(&p_lock);
    *out = params;
    portEXIT_CRITICAL(&p_lock);
}

void set_all_params(const params_t *in) {
    portENTER_CRITICAL(&p_lock);
    params = *in;
    portEXIT_CRITICAL(&p_lock);
}


// ============================================================
//                          Flight
// ============================================================
void set_max_velocity(float vel) {
    portENTER_CRITICAL(&p_lock);
    params.flight.vel_max = vel;
    portEXIT_CRITICAL(&p_lock);
}

float get_max_velocity(void) {
    portENTER_CRITICAL(&p_lock);
    float v = params.flight.vel_max;
    portEXIT_CRITICAL(&p_lock);
    return v;
}


// ============================================================
//                    Controller / Odometry
// ============================================================
void get_gains(GAINS *out) {
    portENTER_CRITICAL(&p_lock);
    *out = params.gains;
    portEXIT_CRITICAL(&p_lock);
}

void set_gains(const GAINS *in) {
    portENTER_CRITICAL(&p_lock);
    params.gains = *in;
    portEXIT_CRITICAL(&p_lock);
}

void get_odom_params(odom_params_t *out) {
    portENTER_CRITICAL(&p_lock);
    *out = params.odom;
    portEXIT_CRITICAL(&p_lock);
}

void set_odom_params(const odom_params_t *in) {
    portENTER_CRITICAL(&p_lock);
    params.odom = *in;
    portEXIT_CRITICAL(&p_lock);
}
