/**
 * Made by Rui B.S.
 * Date: 28/06/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Global state of the drone (position, orientation, velocities,
 *   acceleration, timestamps and state machine state). The sensor tasks
 *   write it and the controller and ROS read it. Every access is protected
 *   with a critical section, so it is thread safe.
 *
 * Functions:
 *   - state_init(): resets the state (identity orientation).
 *   - set_position(), set_h(), set_orientation(), set_vel_lin(), set_vel_ang(),
 *     set_acc_lin(), set_time_imu(), set_time_height(), set_sm_state(): setters.
 *   - get_position(), get_orientation(), get_vel_lin(), get_vel_ang(),
 *     get_acc_lin(), get_time_imu(), get_time_height(), get_sm_state(): getters.
 *   - get_global_state(): returns a copy of the whole state.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "state.h"

static state_t state;
static sm_states_t sm_state;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

// Setters
void state_init(void) {
    portENTER_CRITICAL(&s_lock);
    memset(&sm_state, 0, sizeof(sm_state));

    memset(&state, 0, sizeof(state));
    state.q.w = 1.0f; 
    portEXIT_CRITICAL(&s_lock);
}

void set_position(float x, float y) {
    portENTER_CRITICAL(&s_lock);
    state.pos.x = x;
    state.pos.y = y;
    portEXIT_CRITICAL(&s_lock);
}

void set_h(float z) {
    portENTER_CRITICAL(&s_lock);
    state.pos.z = z;
    portEXIT_CRITICAL(&s_lock);
}

void set_orientation(float qx, float qy, float qz, float qw) {
    portENTER_CRITICAL(&s_lock);
    state.q = (quat_t){qx, qy, qz, qw};
    portEXIT_CRITICAL(&s_lock);
}

void set_vel_lin(float vx, float vy, float vz) {
    portENTER_CRITICAL(&s_lock);
    state.vel_lin = (vec3_t){vx, vy, vz};
    portEXIT_CRITICAL(&s_lock);
}

void set_vel_ang(float wx, float wy, float wz) {
    portENTER_CRITICAL(&s_lock);
    state.vel_ang = (vec3_t){wx, wy, wz};
    portEXIT_CRITICAL(&s_lock);
}

void set_acc_lin(float ax, float ay, float az) {
    portENTER_CRITICAL(&s_lock);
    state.acc_lin = (vec3_t){ax, ay, az};
    portEXIT_CRITICAL(&s_lock);
}

void set_time_imu(int64_t t) {
    portENTER_CRITICAL(&s_lock);
    state.t_stamp_imu = t;
    portEXIT_CRITICAL(&s_lock);
}

void set_time_height(int64_t t) {
    portENTER_CRITICAL(&s_lock);
    state.t_stamp_h = t;
    portEXIT_CRITICAL(&s_lock);
}

void set_sm_state(sm_states_t sm) {
    portENTER_CRITICAL(&s_lock);
    sm_state = sm;
    portEXIT_CRITICAL(&s_lock);
}

// Getters
void get_position(vec3_t *out) {
    portENTER_CRITICAL(&s_lock);
    *out = state.pos;
    portEXIT_CRITICAL(&s_lock);
}

void get_orientation(quat_t *out) {
    portENTER_CRITICAL(&s_lock);
    *out = state.q;
    portEXIT_CRITICAL(&s_lock);
}

void get_vel_lin(vec3_t *out) {
    portENTER_CRITICAL(&s_lock);
    *out = state.vel_lin;
    portEXIT_CRITICAL(&s_lock);
}

void get_vel_ang(vec3_t *out) {
    portENTER_CRITICAL(&s_lock);
    *out = state.vel_ang;
    portEXIT_CRITICAL(&s_lock);
}

void get_acc_lin(vec3_t *out) {
    portENTER_CRITICAL(&s_lock);
    *out = state.acc_lin;
    portEXIT_CRITICAL(&s_lock);
}

void get_time_imu(int64_t *out) {
    portENTER_CRITICAL(&s_lock);
    *out = state.t_stamp_imu;
    portEXIT_CRITICAL(&s_lock);
}

void get_time_height(int64_t *out) {
    portENTER_CRITICAL(&s_lock);
    *out = state.t_stamp_h;
    portEXIT_CRITICAL(&s_lock);
}

void get_sm_state(sm_states_t *out) {
    portENTER_CRITICAL(&s_lock);
    *out = sm_state;
    portEXIT_CRITICAL(&s_lock);
}

void get_global_state(state_t *out) {
    portENTER_CRITICAL(&s_lock);
    *out = state;
    portEXIT_CRITICAL(&s_lock);
}