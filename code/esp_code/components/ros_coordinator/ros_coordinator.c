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
 *                   drone/state (my_msgs/state) [x,y,z|qx,qy,qz,qw|vx,vy,vz|wx,wy,wz|ax,ay,az]
 *                   drone/odom (my_msgs/state)
 *     - Subscribes:
 *                   drone/cmd_vel (geometry_msgs/TwistStamped),
 *                   drone/params (my_msgs/Params)
 *     - Service:
 *                   drone/takeoff_srv (my_msgs/Takeoff)
 *                   drone/land_srv (my_msgs/Land)
 *   In simulation mode it also subscribes to imu/data and barometer/data
 *   and publishes the motor speeds to /drone/command/motor_speed.
 *
 * Functions:
 *   - apply_pending_params(): applies the last received parameters.
 *   - param_callback(): saves the parameters received on drone/params.
 *   - get_cmd_vel(): returns the last velocity command.
 *   - cmd_vel_callback(): saves the velocity received on drone/cmd_vel.
 *   - get_take_off_ready(): returns true if a take-off request was accepted.
 *   - get_take_off_alt(): returns the requested take-off altitude.
 *   - takeoff_callback(): checks a take-off request and answers it.
 *   - stamp_now(): writes the current ROS time in a message stamp.
 *   - fill_state_msg(): fills the state message from the global state.
 *   - imu_callback(), height_callback(): (simulation) save the sensor data from Gazebo.
 *   - motors_msg_init(), pub_motor_speed(): (simulation) prepare and send the motor speeds.
 *   - timer_callback(): publishes the IMU data periodically.
 *   - micro_ros_task(): creates the node, publishers, subscribers, service and executor.
 *   - ros_init(): starts the network interface and the micro-ROS task.
 *   - ros_test(): checks that the module started correctly.
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

// Msg types
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist_stamped.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <my_msgs/msg/params.h>
#include <my_msgs/msg/state.h>

// Services types
#include <my_msgs/srv/takeoff.h>
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
//                       Parameters
// ============================================================
#ifdef CONFIG_SIMULATION_ON
    #include "freertos/queue.h"
    #include <actuator_msgs/msg/actuators.h>
    #include <sensor_msgs/msg/fluid_pressure.h>
    #include "bmp180.h" // To transform the receiving barometer pressure to h(m)

    #define N_HANDLERS 8 // 5 Default + 2 for new subs(IMU, Height)

#else
    #define N_HANDLERS 5
#endif

static const float MAX_ALT = 3.0f; // TODO PONER PARAM
static const float MIN_ALT = 0.5f;

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
//                           Params
// ============================================================
my_msgs__msg__Params param_msg;
static bool params_2_update = false;


// ============================================================
//                          Services
// ============================================================
// Take-off
rcl_service_t takeoff_srv;
my_msgs__srv__Takeoff_Request  takeoff_req;
my_msgs__srv__Takeoff_Response takeoff_res;

static bool take_off_ready = false;
static float altitude = MIN_ALT;

// Land
rcl_service_t land_srv;
std_srvs__srv__Trigger_Request  land_req;
std_srvs__srv__Trigger_Response land_res;

static bool land_ready = false;


// ============================================================
//                           Mutex
// ============================================================
static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;


// ============================================================
//                  Simulation pubs & subs
// ============================================================
#ifdef CONFIG_SIMULATION_ON
    // Subscribers
    rcl_subscription_t imu_sub;
    sensor_msgs__msg__Imu imu_sub_msg;

    rcl_subscription_t height_sub;
    sensor_msgs__msg__FluidPressure h_sub_msg;

    // Publishers
    //#define NUM_MOTORS 4
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
//                         Checker
// ============================================================
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
CHECK_SM_STATE(ERROR);
static bool is_init = false;


// ============================================================
//                       App parameters
// ============================================================
// Apply parameters if is necessary
void apply_pending_params() {
    if (params_2_update) {
        pthread_mutex_lock(&lock);

        set_hovering_h(param_msg.h_max);
        set_max_velocity(param_msg.v_max);
        set_land_on_site(param_msg.land_on_site);

        params_2_update = false;

        pthread_mutex_unlock(&lock);
    }
}

void param_callback(const void *msgin) {
    const my_msgs__msg__Params *new_msg = (const my_msgs__msg__Params *)msgin;
    pthread_mutex_lock(&lock);
    param_msg = *new_msg;
    params_2_update = true;
    pthread_mutex_unlock(&lock);
}


// ============================================================
//                          CMD_VEL
// ============================================================
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
float get_take_off_alt() {
    pthread_mutex_lock(&lock);
    float alt = altitude;
    pthread_mutex_unlock(&lock);
    return alt;
}

// Take off callback
void takeoff_callback(const void *req_msg, void *res_msg) {
    my_msgs__srv__Takeoff_Request  *takeoff_req =
        (my_msgs__srv__Takeoff_Request *)req_msg;

    my_msgs__srv__Takeoff_Response *takeoff_res =
        (my_msgs__srv__Takeoff_Response *)res_msg;

    sm_states_t state;
    get_sm_state(&state);

    // Check drone state.
    // Just accept the order when the drone is armed and waiting to take off
    if (state == ERROR) { // Error state
        takeoff_res->accepted = false;
        rosidl_runtime_c__String__assign(&takeoff_res->reason, "Status error");
        return;
    } else if (state != ARMING) { // Not Arming
        takeoff_res->accepted = false;
        rosidl_runtime_c__String__assign(&takeoff_res->reason, "Not armed");
        return;
    }

    // Check requested altitude
    if (isnan(takeoff_req->altitude)
        || takeoff_req->altitude < MIN_ALT
        || takeoff_req->altitude > MAX_ALT) {
        takeoff_res->accepted = false;
        rosidl_runtime_c__String__assign(&takeoff_res->reason, "Invalid altitude");
        return;
    }

    // Take off -> ready
    pthread_mutex_lock(&lock);
    take_off_ready = true;
    altitude = takeoff_req->altitude;
    pthread_mutex_unlock(&lock);

    takeoff_res->accepted = true;
    rosidl_runtime_c__String__assign(&takeoff_res->reason, "OK");
}


// ============================================================
//                           Land
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
    std_srvs__srv__Trigger_Request  * land_req =
        (std_srvs__srv__Trigger_Request *)req_msg;

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
//                         IMU publisher
// ============================================================

// ============================================================
//                          ROS time
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
// All the global state is read at once, so every field is from the same
// moment. ROS uses SI units (REP 103): the angular velocity goes from deg/s
// to rad/s and the acceleration from g to m/s^2.
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
//                    Simulation Subs/Pubs
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
//                 Main timer of ROS Coordinator
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
//                       Ros task initializer
//
// Initialize timers, callback...
// ============================================================
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
    params_qos.reliability  = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
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
        &takeoff_srv, &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(my_msgs, srv, Takeoff),
        "drone/takeoff_srv"
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

    RCCHECK(rclc_executor_add_service(&executor, &takeoff_srv, &takeoff_req,
        &takeoff_res, takeoff_callback));

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
        #endif

        // if (now_us - last_log_us >= 5000000) {
        //     last_log_us = now_us;
        //     UBaseType_t free_words = uxTaskGetStackHighWaterMark(NULL);
        //     #ifdef CONFIG_SIMULATION_ON
        //         ESP_LOGI(TAG, "alive, stack_min=%u bytes",
        //                  (unsigned)(free_words * sizeof(StackType_t)));
        //     #else
        //         ESP_LOGI(TAG, "alive, stack_min=%u bytes",
        //                  (unsigned)(free_words * sizeof(StackType_t)));
        //     #endif
        // }
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
    if (!is_init) {
        return false;
    }
    return true;
}