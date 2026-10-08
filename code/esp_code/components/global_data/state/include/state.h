/**
 * Made by Rui B.S.
 * Date: 28/06/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Data types (vectors, quaternion, drone state and state machine states)
 *   and public interface of the global state.
 *
 * Functions:
 *   - state_init(): resets the state.
 *   - set_*(): save one value of the state.
 *   - get_*(): read one value of the state.
 *   - get_global_state(): returns a copy of the whole state.
 */

#ifndef STATE_H
#define STATE_H

typedef struct {
    float x, y, z;
} vec3_t;

typedef struct {
    float x, y, z, w;
} quat_t;

typedef struct {
    vec3_t pos;      // Position (m)
    quat_t q;        // Orientation
    vec3_t vel_lin;  // Vel lineal (m/s)
    vec3_t vel_ang;  // Vel angular (rad/s)
    vec3_t acc_lin;  // Acc lineal (m/s^2)
    int64_t t_stamp_imu; // Time (ns)
    int64_t t_stamp_h; // Time (ns)
} state_t;

// States for the state machine(sm)
typedef enum {
    INIT               = 0,
    CHECKING           = 1,
    ARMING             = 2,
    TAKING_OFF         = 3,
    HOVERING           = 4,
    EXTERNAL_CONTROL   = 5,
    LANDING            = 6,
    DISARMING          = 7,
    ERROR              = 8
} sm_states_t;

void state_init(void);

// Setters
void set_position(float x, float y);
void set_h(float z);
void set_orientation(float qx, float qy, float qz, float qw);
void set_vel_lin(float vx, float vy, float vz);
void set_vel_ang(float wx, float wy, float wz);
void set_acc_lin(float ax, float ay, float az);
void set_time_imu(int64_t t);
void set_time_height(int64_t t);
void set_sm_state(sm_states_t sm);

// Getters
void get_position(vec3_t *out);
void get_orientation(quat_t *out);
void get_vel_lin(vec3_t *out);
void get_vel_ang(vec3_t *out);
void get_acc_lin(vec3_t *out);
void get_time_imu(int64_t *out);
void get_time_height(int64_t *out);
void get_sm_state(sm_states_t *out);
void get_global_state(state_t *out);

#endif // STATE_H