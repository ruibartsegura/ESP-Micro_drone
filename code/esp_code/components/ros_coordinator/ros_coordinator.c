/**
 * Made by Rui B.S.
 * Date: 23/05/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   micro-ROS interface of the drone. It connects to the micro-ROS agent
 *   over WiFi (UDP), creates the "rui_drone" node and runs the executor in
 *   its own FreeRTOS task. Shared variables are protected with a mutex.
 *     - Publishes:
 *                   State: drone/state (my_msgs/state) Current status of the state machine
 *                   Odometry drone/state (my_msgs/state) [x,y,z|qx,qy,qz,qw|vx,vy,vz|wx,wy,wz|ax,ay,az]
 *     - Subscribes:
 *                   drone/cmd_vel (geometry_msgs/msg/twist_stamped)
 *                   drone/params (my_msgs/msg/params)
 *     - Service:
 *                   drone/set_altitude (my_msgs/srv/set_altitude): take off, or
 *                   change the altitude in flight
 *                   drone/land_srv (std_srvs/srv/trigger)
 *
 *   In simulation mode, we add:
 *     - Publishes:
 *                   drone/command/motor_speed (actuator_msgs/msg/actuators)
 *     - Subscribes:
 *                   imu/data (sensor_msgs/msg/imu)
 *                   barometer/data (sensor_msgs/msg/fluid_pressure)
 *
 * Functions:
 *   - apply_pending_params(): applies the last received parameters (the
 *     negative fields, PARAM_KEEP, do not change).
 *   - param_callback(): saves the parameters received on drone/params (it
 *     merges them with the ones still pending).
 *   - get_cmd_vel(): returns the last velocity command.
 *   - clear_cmd_vel(): forgets the last velocity command.
 *   - cmd_vel_callback(): saves the velocity received on drone/cmd_vel.
 *   - get_take_off_ready(): returns true if a take-off request was accepted.
 *   - get_target_altitude(): returns the requested target altitude.
 *   - set_altitude_callback(): takes off (ARMING) or changes the target altitude
 *     in flight, and answers the request.
 *   - get_landing_ready(): returns true if a landing request was accepted.
 *   - land_callback(): checks a landing request and answers it.
 *   - stamp_now(): writes the current ROS time in a message stamp.
 *   - fill_state_msg(): fills the state message from the global state.
 *   - imu_callback(), height_callback(): (simulation) save the sensor data from Gazebo.
 *   - motors_msg_init(), pub_motor_speed(): (simulation) prepare and queue the motor speeds.
 *   - motors_timer_callback(): (simulation) publishes the last motor speeds.
 *   - timer_callback(): publishes the state message periodically (and applies
 *     the pending parameters in CHECKING).
 *   - micro_ros_task(): creates the node, publishers, subscribers, service and executor.
 *   - ros_init(): starts the network interface and the micro-ROS task.
 *   - ros_test(): checks that the module started correctly.
 *   - link_alive(): saves the time of an answered ping (or of the connection).
 *   - get_t_last_link(): returns the time of the last answered ping.
 *   - link_ping(): (real drone) pings the agent every LINK_PING_PERIOD_US.
 */


#include "ros_coordinator.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <math.h>

// ESP
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"

// Micro ROS
#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rmw_microros/timing.h>
#include <rmw_microros/time_sync.h>

// ROS msg types
#include <geometry_msgs/msg/twist_stamped.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <my_msgs/msg/params.h>
#include <my_msgs/msg/state.h>

// Services types
#include <my_msgs/srv/set_altitude.h>
#include <std_srvs/srv/trigger.h> // For landing

// My includes
#include "led.h"
#include "imu.h"
#include "system.h"
#include "parameters.h"
#include "state.h"
#include "cpu_stats.h"
#include "sdkconfig.h"

#include <pthread.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#include <rmw/qos_profiles.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ \
    /* printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); */ \
    vTaskDelete(NULL);}}

#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ \
    /* printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc); */ }}

static const char *TAG = "MICRO_ROS";

// ============================================================
//                          Parameters
// ============================================================
#ifdef CONFIG_SIMULATION_ON
    #include "freertos/queue.h"
    #include <actuator_msgs/msg/actuators.h>
    #include <sensor_msgs/msg/fluid_pressure.h>
    #include <sensor_msgs/msg/imu.h>

    #include "bmp180.h" // To transform the receiving barometer pressure to h(m)

    #define N_HANDLERS 8 // 5 Default + 2 for new subs(IMU, Height)

#else
    #define N_HANDLERS 5
#endif

static const float MAX_ALT = CONFIG_FLIGHT_H_MAX / 100.0f; // cm -> m (menuconfig)
static const float MIN_ALT = CONFIG_FLIGHT_H_MIN / 100.0f;

#define g 9.81f // Gravity (m/s^2)

// Time synchronization with the agent (only real drone)
#define TIME_SYNC_TIMEOUT_MS 1000
#define TIME_SYNC_PERIOD_US  60000000LL // 60 s, the ESP clock drifts

// ============================================================
//                       State pub & msg
// ============================================================
rcl_publisher_t state_pub;
my_msgs__msg__State state_msg;


// ============================================================
//                         Subscribers
// ============================================================
rcl_subscription_t params_sub;
my_msgs__msg__Params recv_msg;

rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__TwistStamped cmd_vel_msg;
geometry_msgs__msg__Twist cmd_vel;
static int64_t t_last_vel = -1; // last cmd_vel received


// ============================================================
//                            Params
// ============================================================
my_msgs__msg__Params param_msg;
static bool params_2_update = false;


// ============================================================
//                           Services
// ============================================================
// Take-off
rcl_service_t set_altitude_srv;
my_msgs__srv__SetAltitude_Request  set_altitude_req;
my_msgs__srv__SetAltitude_Response set_altitude_res;

static bool take_off_ready = false;
static float target_altitude = MIN_ALT;

// Land
rcl_service_t land_srv;
std_srvs__srv__Trigger_Request  land_req;
std_srvs__srv__Trigger_Response land_res;

static bool land_ready = false;


// ============================================================
//                            Mutex
// ============================================================
static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;


// ============================================================
//                    Simulation pubs & subs
// ============================================================
#ifdef CONFIG_SIMULATION_ON
    // Subscribers
    rcl_subscription_t imu_sub;
    sensor_msgs__msg__Imu imu_sub_msg;

    rcl_subscription_t height_sub;
    sensor_msgs__msg__FluidPressure h_sub_msg;

    // Publishers
    static double motors_buf[NUM_MOTORS];
    static char   frame_id_buf[] = "base_link";

    // Necesary because the publisher is called from another task
    static bool motors_pub_ready = false;

    // Queue of 1 for new motors msg
    typedef struct { double v[NUM_MOTORS]; } motors_cmd_t;
    static QueueHandle_t motors_q = NULL;

    rcl_publisher_t motors_pub;
    actuator_msgs__msg__Actuators motors_msg;
#endif


// ============================================================
//                      State msg checker
// ============================================================
// Check if there is a new state defined
#define CHECK_SM_STATE(s) \
    _Static_assert((int)my_msgs__msg__State__##s == (int)(s), \
    "State.msg & sm_states_t does not match: " #s)

CHECK_SM_STATE(INIT);
CHECK_SM_STATE(CHECKING);
CHECK_SM_STATE(ARMING);
CHECK_SM_STATE(TAKING_OFF);
CHECK_SM_STATE(HOVERING);
CHECK_SM_STATE(EXTERNAL_CONTROL);
CHECK_SM_STATE(LANDING);
CHECK_SM_STATE(DISARMING);
CHECK_SM_STATE(EMERGENCY_LANDING);
CHECK_SM_STATE(ERROR);
static bool is_init = false;


// ============================================================
//                           ROS Test
// ============================================================
// Pre-flight ping on demand
static bool ping_requested = false;
static int  ping_result    = 0; // 0 = pending, 1 = ok, -1 = fail

// Link with the agent: time of the last ping that was answered. The real
// drone pings every LINK_PING_PERIOD_US also in flight; system.c starts the
// failsafe if it is too old. Not in simulation: the ping would stop the
// executor (motors timer) and the IMU timeout already detects a lost link
static int64_t t_last_link_us = 0;

#ifndef CONFIG_SIMULATION_ON
    #define LINK_PING_PERIOD_US  500000LL // 500 ms
    #define LINK_PING_TIMEOUT_MS 100      // Max time that uros_task is stopped
    static int64_t t_last_ping_us = 0;
#endif

// ============================================================
//                        App parameters                       
// ============================================================
// Apply parameters if is necessary. The fields that were not filled in the
// message are -1 (default value of Params.msg) and keep their value. If one
// value is out of range the whole message is rejected
void apply_pending_params() {
    pthread_mutex_lock(&lock);
    if (!params_2_update) {
        pthread_mutex_unlock(&lock);
        return;
    }
    my_msgs__msg__Params m = param_msg;
    params_2_update = false;
    pthread_mutex_unlock(&lock);

    params_t p = {
        .flight = {
            .vel_max = m.v_max,
        },
        .gains = {
            .kp_h            = m.kp_h,
            .kd_h            = m.kd_h,
            .kp_vel          = m.kp_vel,
            .kp_angle        = m.kp_angle,
            .kp_rate         = m.kp_rate,
            .ki_rate         = m.ki_rate,
            .kp_yaw_rate     = m.kp_yaw_rate,
            .max_rate        = m.max_rate,
            .max_pow_rp      = m.max_pow_rp,
            .base            = m.base,
            .alpha           = m.alpha,
            .max_angle       = m.max_angle,
            .max_pow_yaw     = m.max_pow_yaw,
            .pitch_deviation = m.pitch_deviation,
            .roll_deviation  = m.roll_deviation,
        },
        .odom = {
            .k_drag = m.k_drag,
        },
    };

    const char *bad = params_update(&p);
    if (bad != NULL) {
        ESP_LOGW(TAG, "drone/params rejected: %s out of range", bad);
    }
}

// Params.msg has one float32 for each parameter (the merge goes field by field)
_Static_assert(sizeof(my_msgs__msg__Params) == sizeof(params_t),
               "my_msgs/Params must have one float32 for each field of params_t");

// Saves the message until it is applied (CHECKING). If there is already one
// pending (sent while flying), they are merged: the filled fields of the new
// one overwrite, the -1 ones keep the pending value. Only runs when a message
// arrives (18 compares)
void param_callback(const void *msgin) {
    const my_msgs__msg__Params *new_msg = (const my_msgs__msg__Params *)msgin;
    pthread_mutex_lock(&lock);
    if (!params_2_update) {
        param_msg = *new_msg;
    } else {
        const float *in = (const float *)new_msg;
        float *pend = (float *)&param_msg;
        for (size_t i = 0; i < sizeof(param_msg) / sizeof(float); i++) {
            // Not PARAM_KEEP. NaN / inf also overwrite, so they are rejected later
            if (!(isfinite(in[i]) && in[i] < 0.0f)) {
                pend[i] = in[i];
            }
        }
    }
    params_2_update = true;
    pthread_mutex_unlock(&lock);
}


// ============================================================
//                           CMD_VEL
// ============================================================
// Return true and update the var out, if there is a new cmd_vel,
// if the max_age_ms has passed return false
bool get_cmd_vel(geometry_msgs__msg__Twist *out, int max_age_ms) {
    pthread_mutex_lock(&lock);
    bool fresh =
        (t_last_vel > 0) && ((esp_timer_get_time() - t_last_vel) < max_age_ms * 1000LL);

    if (fresh) {
        *out = cmd_vel;
    }
    pthread_mutex_unlock(&lock);
    return fresh;
}

void clear_cmd_vel(void) {
    pthread_mutex_lock(&lock);
    t_last_vel = 0;
    pthread_mutex_unlock(&lock);
}

void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__TwistStamped *new_msg =
        (const geometry_msgs__msg__TwistStamped *)msgin;

    pthread_mutex_lock(&lock);
    cmd_vel = new_msg->twist;
    t_last_vel = esp_timer_get_time();
    pthread_mutex_unlock(&lock);
}


// ============================================================
//                           Take off
// ============================================================
// Getter if the take off is ready
bool get_take_off_ready() {
    pthread_mutex_lock(&lock);
    bool res = take_off_ready;
    take_off_ready = false;
    pthread_mutex_unlock(&lock);
    return res;
}

// Getter altitude of the take off
float get_target_altitude() {
    pthread_mutex_lock(&lock);
    float alt = target_altitude;
    pthread_mutex_unlock(&lock);
    return alt;
}

// Take off / altitude callback.
// - ARMING: takes off to the requested altitude.
// - TAKING_OFF, HOVERING, EXTERNAL_CONTROL: only changes the target altitude
//   (system.c reads get_target_altitude() every cycle). take_off_ready is not
//   set, so it does not take off again after the next landing.
// - Other states: rejected.
void set_altitude_callback(const void *req_msg, void *res_msg) {
    my_msgs__srv__SetAltitude_Request  *req =
        (my_msgs__srv__SetAltitude_Request *)req_msg;

    my_msgs__srv__SetAltitude_Response *res =
        (my_msgs__srv__SetAltitude_Response *)res_msg;

    sm_states_t state;
    get_sm_state(&state);

    // Check drone state
    bool take_off = (state == ARMING);
    bool flying = (state == TAKING_OFF || state == HOVERING
                   || state == EXTERNAL_CONTROL);

    if (state == ERROR) { // Error state
        res->accepted = false;
        rosidl_runtime_c__String__assign(&res->reason, "Status error");
        return;
    } else if (!take_off && !flying) { // Not armed and not flying
        res->accepted = false;
        rosidl_runtime_c__String__assign(&res->reason, "Not armed");
        return;
    }

    // Check requested altitude
    if (isnan(req->altitude)
        || req->altitude < MIN_ALT
        || req->altitude > MAX_ALT) {
        res->accepted = false;
        rosidl_runtime_c__String__assign(&res->reason, "Invalid altitude");
        return;
    }

    pthread_mutex_lock(&lock);
    target_altitude = req->altitude;
    if (take_off) {
        take_off_ready = true; // Take off -> ready
    }
    pthread_mutex_unlock(&lock);

    res->accepted = true;
    rosidl_runtime_c__String__assign(&res->reason,
                                     take_off ? "OK" : "Altitude changed");
}


// ============================================================
//                             Land
// ============================================================
// Getter if the landing is ready
bool get_landing_ready() {
    pthread_mutex_lock(&lock);
    bool res = land_ready;
    land_ready = false;
    pthread_mutex_unlock(&lock);
    return res;
}

// Take off callback
void land_callback(const void * req_msg, void * res_msg) {
    // std_srvs__srv__Trigger_Request  * land_req =
    //     (std_srvs__srv__Trigger_Request *)req_msg;

    std_srvs__srv__Trigger_Response * land_res =
        (std_srvs__srv__Trigger_Response *)res_msg;

    sm_states_t state;
    get_sm_state(&state);

    // Check drone state, to just accept the land order when the drone is flying
    if (state == ERROR) { // Error state
        land_res->success = false;
        rosidl_runtime_c__String__assign(&land_res->message, "Status error");
        return;
    } else if (state != HOVERING && state != EXTERNAL_CONTROL) {
        land_res->success = false;
        rosidl_runtime_c__String__assign(&land_res->message, "Not flying");
        return;
    }

    // Land -> ready
    pthread_mutex_lock(&lock);
    land_ready = true;
    pthread_mutex_unlock(&lock);

    land_res->success = true;
    rosidl_runtime_c__String__assign(&land_res->message, "OK");
}

// ============================================================
//                           ROS time
// ============================================================
// Current ROS time in a stamp:
//   - Simulation: time of the last IMU sample of Gazebo (sim time).
//   - Real drone: time of the agent (epoch) once synchronized, before
//     that the time since boot.
void stamp_now(builtin_interfaces__msg__Time * stamp) {
    int64_t t_ns;
#ifdef CONFIG_SIMULATION_ON
    get_time_imu(&t_ns);
#else
    if (rmw_uros_epoch_synchronized()) {
        t_ns = rmw_uros_epoch_nanos();
    } else {
        t_ns = esp_timer_get_time() * 1000LL; // us -> ns
    }
#endif
    stamp->sec     = (int32_t)(t_ns / 1000000000LL);
    stamp->nanosec = (uint32_t)(t_ns % 1000000000LL);
}


// Fill state msg

void fill_state_msg(my_msgs__msg__State *out) {
    stamp_now(&out->header.stamp);

    state_t st;
    get_global_state(&st);

    out->pos.x = st.pos.x;
    out->pos.y = st.pos.y;
    out->pos.z = st.pos.z;

    out->q.x = st.q.x;
    out->q.y = st.q.y;
    out->q.z = st.q.z;
    out->q.w = st.q.w;

    out->vel_lin.x = st.vel_lin.x;
    out->vel_lin.y = st.vel_lin.y;
    out->vel_lin.z = st.vel_lin.z;

    out->vel_ang.x = st.vel_ang.x * M_PI / 180;
    out->vel_ang.y = st.vel_ang.y * M_PI / 180;
    out->vel_ang.z = st.vel_ang.z * M_PI / 180;

    out->acc_lin.x = st.acc_lin.x * g;
    out->acc_lin.y = st.acc_lin.y * g;
    out->acc_lin.z = st.acc_lin.z * g;

    out->sm_state = (uint8_t)st.sm_state;
}


// ============================================================
//                     Simulation Subs/Pubs
// ============================================================
#ifdef CONFIG_SIMULATION_ON
    // Imu Callback
    void imu_callback(const void *msgin) {
        const sensor_msgs__msg__Imu *new_msg = (const sensor_msgs__msg__Imu *)msgin;
        pthread_mutex_lock(&lock);
        imu_sub_msg = *new_msg;
        pthread_mutex_unlock(&lock);

        // Gazebo sends rad/s and m/s^2, the real driver (and the controller)
        // uses deg/s and g
        set_vel_ang(
            imu_sub_msg.angular_velocity.x * 180 / M_PI,
            imu_sub_msg.angular_velocity.y * 180 / M_PI,
            imu_sub_msg.angular_velocity.z * 180 / M_PI
        );

        set_acc_lin(
            imu_sub_msg.linear_acceleration.x / g,
            imu_sub_msg.linear_acceleration.y / g,
            imu_sub_msg.linear_acceleration.z / g
        );

        // Full time in ns (only nanosec goes back to 0 every second)
        set_time_imu((int64_t)imu_sub_msg.header.stamp.sec * 1000000000LL
                     + imu_sub_msg.header.stamp.nanosec);
    }

    // Height Callback
    void height_callback(const void *msgin) {
        const sensor_msgs__msg__FluidPressure *new_msg =
            (const sensor_msgs__msg__FluidPressure *)msgin;
        pthread_mutex_lock(&lock);
        h_sub_msg = *new_msg;
        pthread_mutex_unlock(&lock);

        set_h(bmp180_pressure_to_altitude(h_sub_msg.fluid_pressure, 101325.0f));
        set_time_height((int64_t)h_sub_msg.header.stamp.sec * 1000000000LL
                        + h_sub_msg.header.stamp.nanosec);
    }

    // Motors
    void motors_msg_init(void) {
        actuator_msgs__msg__Actuators__init(&motors_msg);

        motors_msg.velocity.data     = motors_buf;
        motors_msg.velocity.size     = NUM_MOTORS;
        motors_msg.velocity.capacity = NUM_MOTORS;

        motors_msg.header.frame_id.data     = frame_id_buf;
        motors_msg.header.frame_id.size     = strlen(frame_id_buf);
        motors_msg.header.frame_id.capacity = sizeof(frame_id_buf);

        motors_q = xQueueCreate(1, sizeof(motors_cmd_t));
    }

    // Update value for the motors
    void pub_motor_speed(const double power[NUM_MOTORS]) {
        if (motors_q == NULL) return;

        motors_cmd_t cmd;
        memcpy(cmd.v, power, sizeof(cmd.v));
        xQueueOverwrite(motors_q, &cmd);
    }

    // Timer for motors, independent
    static void motors_timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
        RCLC_UNUSED(last_call_time);
        motors_cmd_t cmd;

        if (timer == NULL || !motors_pub_ready) return;
        if (xQueueReceive(motors_q, &cmd, 0) != pdTRUE) return;  // nada nuevo

        for (int i = 0; i < NUM_MOTORS; i++) {
            motors_msg.velocity.data[i] = cmd.v[i];

        }
        stamp_now(&motors_msg.header.stamp);
        RCSOFTCHECK(rcl_publish(&motors_pub, &motors_msg, NULL));
    }
#endif


// ============================================================
//                Main timer of ROS Coordinator
// ============================================================
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        sm_states_t state;
        get_sm_state(&state);
        if (state == CHECKING && params_2_update) {
            apply_pending_params();
        }

        fill_state_msg(&state_msg);
        RCSOFTCHECK(rcl_publish(&state_pub, &state_msg, NULL));

    }
}


// ============================================================
//                     Ros task initializer
// ============================================================
// Initialize timers, callback...
// ============================================================
//                         Link check
// ============================================================
static void link_alive(int64_t now_us) {
    pthread_mutex_lock(&lock);
    t_last_link_us = now_us;
    pthread_mutex_unlock(&lock);
}

int64_t get_t_last_link(void) {
    pthread_mutex_lock(&lock);
    int64_t t = t_last_link_us;
    pthread_mutex_unlock(&lock);
    return t;
}

#ifndef CONFIG_SIMULATION_ON
    // Short timeout and 1 attempt: with the link lost it stops uros_task
    // LINK_PING_TIMEOUT_MS at most (uros_task does not fly the real drone)
    static void link_ping(int64_t now_us) {
        if (now_us - t_last_ping_us < LINK_PING_PERIOD_US) {
            return;
        }
        t_last_ping_us = now_us;

        if (rmw_uros_ping_agent(LINK_PING_TIMEOUT_MS, 1) == RMW_RET_OK) {
            link_alive(now_us);
        }
    }
#endif


void micro_ros_task(void * arg) {
    // Micro ros initialization
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    RCCHECK(rcl_init_options_init(&init_options, allocator));


    #ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
        rmw_init_options_t *rmw_options =
            rcl_init_options_get_rmw_init_options(&init_options);

        RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP,
                                                 CONFIG_MICRO_ROS_AGENT_PORT,
                                                 rmw_options));
    #endif

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    led_on(LED_ESP); // Micro ros has connected to the net

    // Take the time of the agent for the stamps (in simulation the time
    // comes from Gazebo)
    #ifndef CONFIG_SIMULATION_ON
        RCSOFTCHECK(rmw_uros_sync_session(TIME_SYNC_TIMEOUT_MS));
        int64_t last_sync_us = esp_timer_get_time();
        link_alive(last_sync_us); // Connected
    #else
        link_alive(esp_timer_get_time()); // Connected
    #endif

    // Init main node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "rui_drone", "", &support));

    // Init state publisher
    // Save valid mem for frame_id (the position is relative to the odometry
    // origin)
    my_msgs__msg__State__init(&state_msg);
    rosidl_runtime_c__String__assign(&state_msg.header.frame_id, "odom");

    RCCHECK(rclc_publisher_init_best_effort(
        &state_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(my_msgs, msg, State),
        "drone/state"));

    // QoS params sub
    rmw_qos_profile_t params_qos = rmw_qos_profile_default;
    // Reliable: a lost message over WiFi is sent again by the agent. Only
    // reliable publishers wait for the ACK; a subscriber does not block the
    // executor (motors / imu in simulation use the best effort stream)
    params_qos.reliability  = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    params_qos.durability   = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    params_qos.history      = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    params_qos.depth        = 2;

    // Init param sub
    RCCHECK(rclc_subscription_init(
        &params_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(my_msgs, msg, Params),
        "drone/params", &params_qos));

    // Init cmd vel sub
    RCCHECK(rclc_subscription_init_best_effort(
        &cmd_vel_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped),
        "drone/cmd_vel"
    ));

    // Init take off service
    RCCHECK(rclc_service_init_default(
        &set_altitude_srv, &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(my_msgs, srv, SetAltitude),
        "drone/set_altitude"
    ));

    // Initialitation for simulation pub/sub
    #ifdef CONFIG_SIMULATION_ON
        // QoS
        rmw_qos_profile_t sim_qos = rmw_qos_profile_sensor_data;
        sim_qos.depth = 1;

        // Init imu sub
        RCCHECK(rclc_subscription_init(&imu_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
                "imu/data", &sim_qos));

        // Init barometer sub
        RCCHECK(rclc_subscription_init(&height_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, FluidPressure),
            "barometer/data", &sim_qos));

        // Init motors pub
        RCCHECK(rclc_publisher_init_best_effort(&motors_pub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(actuator_msgs, msg, Actuators),
            "/drone/command/motor_speed"));

        motors_pub_ready = true;
        rmw_uros_set_publisher_session_timeout(
            rcl_publisher_get_rmw_handle(&motors_pub), 50);
    #endif

    // Init land service
    RCCHECK(rclc_service_init_default(
        &land_srv, &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
        "drone/land_srv"
    ));

    // Init main timer
    rcl_timer_t timer;
    const unsigned int timer_timeout = 300;
    RCCHECK(rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback,
        true));

    // Add everything to the executor, the order matter
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, N_HANDLERS, &allocator));

    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    RCCHECK(rclc_executor_add_service(&executor, &set_altitude_srv, &set_altitude_req,
        &set_altitude_res, set_altitude_callback));

    RCCHECK(rclc_executor_add_service(&executor, &land_srv, &land_req,
        &land_res, land_callback));

    RCCHECK(rclc_executor_add_subscription(&executor, &params_sub, &recv_msg,
        &param_callback, ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg,
        &cmd_vel_callback, ON_NEW_DATA));

    #ifdef CONFIG_SIMULATION_ON
        // Motors timer
        rcl_timer_t motors_timer;
        RCCHECK(rclc_timer_init_default2(&motors_timer, &support,
                RCL_MS_TO_NS(15), motors_timer_callback, true));   // ~66 Hz
        RCCHECK(rclc_executor_add_timer(&executor, &motors_timer));

        // Simulation subs
        RCCHECK(rclc_executor_add_subscription(&executor, &imu_sub,
            &imu_sub_msg, &imu_callback, ON_NEW_DATA));

        RCCHECK(rclc_executor_add_subscription(&executor, &height_sub,
            &h_sub_msg, &height_callback, ON_NEW_DATA));
    #endif

    // The period is the timeout of spin_some (it returns before if there
    // is work): "late" means that one spin took more than 500 ms
    static task_period_t period = TASK_PERIOD("uros_task", 250);

    // int64_t last_log_us = esp_timer_get_time();
    while (1) {
        task_period_tick(&period); // CPU / period log (CONFIG_CPU_STATS_ON)
        rcl_ret_t rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(250));

        if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) {
            // ESP_LOGW(TAG, "spin_some error: %d", (int)rc);
        }

        #ifndef CONFIG_SIMULATION_ON
            int64_t now_us = esp_timer_get_time();
            if (now_us - last_sync_us >= TIME_SYNC_PERIOD_US) {
                last_sync_us = now_us;
                RCSOFTCHECK(rmw_uros_sync_session(TIME_SYNC_TIMEOUT_MS));
            }

            link_ping(now_us);
        #endif

        pthread_mutex_lock(&lock);
        bool do_ping = ping_requested;
        pthread_mutex_unlock(&lock);

        if (do_ping) {
            bool ok = (rmw_uros_ping_agent(200, 3) == RMW_RET_OK);
            if (ok) {
                link_alive(esp_timer_get_time());
            }

            pthread_mutex_lock(&lock);
            if (ok) {
                ping_result = 1;
            } else {
                ping_result = -1;
            }
            ping_requested = false;
            pthread_mutex_unlock(&lock);
        }
    }
}

void ros_init(void) {
    if (is_init) {
        return;
    }

    #ifdef CONFIG_SIMULATION_ON
        motors_msg_init();
    #endif

    #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
        ESP_ERROR_CHECK(uros_network_interface_initialize());
    #endif

    #if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN)
        ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));   // sin ahorro de energía
    #endif

    xTaskCreate(micro_ros_task,
            "uros_task",
            CONFIG_MICRO_ROS_STACK,
            NULL,
            CONFIG_MICRO_ROS_TASK_PRIO,
            NULL);

    is_init = true;
}

bool ros_test(void) {
    if (!is_init) return false;

    pthread_mutex_lock(&lock);
    ping_result    = 0;
    ping_requested = true;
    pthread_mutex_unlock(&lock);

    // spin_some wakes up at most every 250 ms, then the ping takes up to ~600 ms
    for (int i = 0; i < 100; i++) { // 100 * 10 ms = 1 s max
        pthread_mutex_lock(&lock);
        int r = ping_result;
        pthread_mutex_unlock(&lock);

        if (r != 0) return (r == 1);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // uros_task did not answer -> dead task or stuck
    pthread_mutex_lock(&lock);
    ping_requested = false;
    pthread_mutex_unlock(&lock);
    return false;
}