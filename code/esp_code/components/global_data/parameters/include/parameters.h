/**
 * Made by Rui B.S.
 * Date: 28/06/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Public interface of the parameters that can change at runtime: flight
 *   parameters, controller gains and odometry parameters. They are in one
 *   struct (params_t) protected with a critical section, like state.c.
 *   Every field is a float, so param_info[] can describe all of them
 *   (name, place in params_t and allowed range). The serial console
 *   (tuning.c) and ROS (drone/params) use the same table.
 *
 *   A negative value in params_update() means "do not change this field"
 *   (PARAM_KEEP): no parameter can be negative.
 *
 * Functions:
 *   - params_init(): goes back to the menuconfig values.
 *   - get_default_params(): returns the menuconfig values.
 *   - params_keep_all(): fills a params_t with PARAM_KEEP.
 *   - params_update(): changes the fields that are not PARAM_KEEP.
 *   - params_find(): looks for a parameter by name.
 *   - get_all_params() / set_all_params(): read / write the whole struct.
 *   - set_max_velocity() / get_max_velocity(): maximum velocity.
 *   - get_gains() / set_gains(): controller gains.
 *   - get_odom_params() / set_odom_params(): odometry parameters.
 */

#ifndef PARAMETERS_H
#define PARAMETERS_H

#include "sdkconfig.h"
#include <stdbool.h>
#include <stddef.h>

// Value of a field that must not change in params_update()
#define PARAM_KEEP (-1.0f)

// Flight
typedef struct {
    float vel_max; // Max horizontal velocity of cmd_vel (m/s)
} flight_params_t;

// Controller (attitude_controller.c)
typedef struct gains {
    float kp_h;            // m error -> motor speed (rad/s)
    float kd_h;            // m/s vertical velocity -> motor speed (rad/s)
    float kp_vel;          // m/s error -> target roll / pitch (rad)
    float kp_angle;        // deg error -> deg/s target
    float kp_rate;         // deg/s error -> motor speed (roll / pitch)
    float ki_rate;         // integral of the deg/s error (deg) -> motor speed
    float kp_yaw_rate;     // deg/s error -> motor speed (yaw)
    float max_rate;        // Max target roll/pitch rate (deg/s)
    float max_pow_rp;      // Max speed correction of each roll / pitch term (rad/s)
    float base;            // Hover motor speed (rad/s)
    float alpha;           // Complementary filter of roll / pitch (0-1)
    float max_angle;       // Max target tilt (rad)
    float max_pow_yaw;     // Max speed correction of the yaw term (rad/s)
    float pitch_deviation; // Start value of the pitch integral (rad/s)
    float roll_deviation;  // Start value of the roll integral (rad/s)
} GAINS;

// Odometry (odometry_estimator.c)
typedef struct {
    float k_drag; // Rotor drag, a_body = -k_drag * v_body (1/s)
} odom_params_t;

typedef struct {
    flight_params_t flight;
    GAINS gains;
    odom_params_t odom;
} params_t;

// Name, place in params_t and allowed range of each parameter
typedef struct {
    const char *name;
    size_t offset; // offsetof(params_t, ...)
    float min;
    float max;
} param_info_t;

extern const param_info_t param_info[];
extern const size_t n_param_info;

void params_init(void);
void get_default_params(params_t *out);

void params_keep_all(params_t *p);
const char *params_update(const params_t *in); // NULL = ok, else the rejected name
const param_info_t *params_find(const char *name);

void get_all_params(params_t *out);
void set_all_params(const params_t *in);

void set_max_velocity(float vel);
float get_max_velocity(void);

void get_gains(GAINS *out);
void set_gains(const GAINS *in);

void get_odom_params(odom_params_t *out);
void set_odom_params(const odom_params_t *in);

#endif // PARAMETERS_H
