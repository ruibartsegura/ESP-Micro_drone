/**
 * Made by Rui B.S.
 * Date: 23/05/2026
 * email: rui.bartolome@gmail.com
 */

#include <stdbool.h>

#include "ros_coordinator.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rosidl_runtime_c/string_functions.h>

// Msg types
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist_stamped.h>
#include <geometry_msgs/msg/pose_stamped.h>
#include <my_msgs/msg/params.h>

// Services types
#include <my_msgs/srv/takeoff.h>

#include "led.h"
#include "imu.h"
#include "system.h"
#include "parameters.h"
#include "sdkconfig.h"

#include <pthread.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#include <rmw/qos_profiles.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// ============================================================
//                       Parameters
// ============================================================
#ifdef CONFIG_SIMULATION_ON
    #define N_HANDLERS 7 // 5 Default + 2 for new subs(IMU, Height)

#else
    #define N_HANDLERS 5

#endif


// ============================================================
//                       Parameters
// ============================================================
static const float MAX_ALT = 3.0f;
static const float MIN_ALT = 0.5f;
#define DEG_TO_RAD  0.017453293f  // pi / 180
#define GRAVITY_MS2 9.80665f

// ============================================================
//                       IMU pub & msg
// ============================================================
rcl_publisher_t imu_pub;
sensor_msgs__msg__Imu imu_msg;

// ============================================================
//                         Subscribers
// ============================================================
rcl_subscription_t params_sub;
std_msgs__msg__Int32 recv_msg;

rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__TwistStamped cmd_vel_msg;

// ============================================================
//                           Params
// ============================================================
my_msgs__msg__Params param_msg;
static bool params_2_update = false;

// ============================================================
//                          Services
// ============================================================
rcl_service_t takeoff_srv;
my_msgs__srv__Takeoff_Request  takeoff_req;
my_msgs__srv__Takeoff_Response takeoff_res;

// ============================================================
//                          Mutex
// ============================================================
pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;

// ============================================================
//                  Take off service params
// ============================================================
bool take_off_ready = false;
float altitude = MIN_ALT;
// ============================================================
//                  Simulation pubs & subs
// ============================================================

#ifdef CONFIG_SIMULATION_ON
    // Subscribers
    rcl_subscription_t imu_sub;
    sensor_msgs__msg__Imu imu_sub_msg;

    rcl_subscription_t height_sub;
    geometry_msgs__msg__PoseStamped h_sub_msg;

    // Publishers
    #define N_MOTORS 4

    rcl_publisher_t motors_pub[N_MOTORS];
    geometry_msgs__msg__TwistStamped motors_msg[N_MOTORS];
#endif


// ============================================================
//                         Checker
// ============================================================
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

void param_callback(const void * msgin)
{
    const my_msgs__msg__Params * new_msg = (const my_msgs__msg__Params *)msgin;
    pthread_mutex_lock(&lock);
    param_msg = *new_msg;
    params_2_update = true;
    pthread_mutex_unlock(&lock);
}



// ============================================================
//                          CMD_VEL
// ============================================================
geometry_msgs__msg__Twist get_cmd_vel() {
    return cmd_vel_msg.twist;
}

void cmd_vel_callback(const void * msgin)
{
    const geometry_msgs__msg__TwistStamped * new_msg = (const geometry_msgs__msg__TwistStamped *)msgin;
    pthread_mutex_lock(&lock);
    cmd_vel_msg = *new_msg;
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
void takeoff_callback(const void * req_msg, void * res_msg) {
    my_msgs__srv__Takeoff_Request  * takeoff_req =
        (my_msgs__srv__Takeoff_Request *)req_msg;
    my_msgs__srv__Takeoff_Response * takeoff_res =
        (my_msgs__srv__Takeoff_Response *)res_msg;

    int state = get_state();

    // Check drone state, to just accept the take off when the drone is armed and waiting to take off
    if (state == 8) { // Error state
        takeoff_res->accepted = false;
        rosidl_runtime_c__String__assign(&takeoff_res->reason, "Status error");
        return;
    } else if (state != 2) { // Not Arming
        takeoff_res->accepted = false;
        rosidl_runtime_c__String__assign(&takeoff_res->reason, "Not armed");
        return;
    }

    // Check requested altitude
    if (takeoff_req->altitude < MIN_ALT || takeoff_req->altitude > MAX_ALT) {
        takeoff_res->accepted = false;
        rosidl_runtime_c__String__assign(&takeoff_res->reason, "Invalid altitude");
        return;
    }

    // Take off -> ready
    pthread_mutex_lock(&lock);
    take_off_ready = true;
    altitude = takeoff_req->altitude;
    pthread_mutex_unlock(&lock);

    // Service answer
    takeoff_res->accepted = true;
    rosidl_runtime_c__String__assign(&takeoff_res->reason, "OK");
}

// ============================================================
//                         IMU publisher
// ============================================================
// Fill IMU msg
void fill_imu_msg(const IMU * msg, sensor_msgs__msg__Imu * out) {
    int64_t now_ms = esp_timer_get_time();

    out->header.stamp.sec = now_ms / 1000;
    out->header.stamp.nanosec = (now_ms % 1000) * 1000000;

    // FIX: estaban intercambiados (aceleración en angular_velocity y
    // giro en linear_acceleration) y sin convertir unidades.
    // sensor_msgs/Imu espera angular_velocity en rad/s y
    // linear_acceleration en m/s².
    out->angular_velocity.x = msg->Vel_ang_X * DEG_TO_RAD;
    out->angular_velocity.y = msg->Vel_ang_Y * DEG_TO_RAD;
    out->angular_velocity.z = msg->Vel_ang_Z * DEG_TO_RAD;

    out->linear_acceleration.x = msg->Acc_lin_X * GRAVITY_MS2;
    out->linear_acceleration.y = msg->Acc_lin_Y * GRAVITY_MS2;
    out->linear_acceleration.z = msg->Acc_lin_Z * GRAVITY_MS2;
}


// ============================================================
//                    Simulation Subs/Pubs
// ============================================================
#ifdef CONFIG_SIMULATION_ON
    // Imu Callback
    sensor_msgs__msg__Imu get_imu_sim(){
        pthread_mutex_lock(&lock);
        sensor_msgs__msg__Imu msg = imu_sub_msg;
        pthread_mutex_unlock(&lock);

        return msg;
    }

    void imu_callback(const void * msgin) {
        const sensor_msgs__msg__Imu * new_msg = (const sensor_msgs__msg__Imu *)msgin;
        pthread_mutex_lock(&lock);
        imu_sub_msg = *new_msg;
        pthread_mutex_unlock(&lock);
    }

    // Height Callback
    geometry_msgs__msg__PoseStamped get_height_sim(){
        pthread_mutex_lock(&lock);
        geometry_msgs__msg__PoseStamped msg = h_sub_msg;
        pthread_mutex_unlock(&lock);

        return msg;
    }

    void height_callback(const void * msgin) {
        const geometry_msgs__msg__PoseStamped * new_msg = (const geometry_msgs__msg__PoseStamped *)msgin;
        pthread_mutex_lock(&lock);
        h_sub_msg = *new_msg;
        pthread_mutex_unlock(&lock);
    }

    void pub_motor_speed(uint8_t motor_id, uint8_t motor_spd) {
        int64_t now_ms = esp_timer_get_time();

        motors_msg[motor_id].twist.linear.x = motor_spd;
        motors_msg[motor_id].header.stamp.nanosec = now_ms * 1000;
        RCSOFTCHECK(rcl_publish(&motors_pub[motor_id], &motors_msg[motor_id], NULL));      
    }
#endif


// ============================================================
//                 Main timer of ROS Coordinator
// ============================================================
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    RCLC_UNUSED(last_call_time);
    if (timer != NULL) {
        if (get_state() == 1 && params_2_update) {
            apply_pending_params();
        }

        IMU imu_data;
        esp_err_t err = get_imu_data(&imu_data);
        if (err == ESP_OK) {
            fill_imu_msg(&imu_data, &imu_msg);
            RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
        } else {
            led_on(LED_RED);
        }
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
        rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
        RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT, rmw_options));
    #endif

    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    led_on(LED_ESP); // Micro ros has connected to the net

    // Init main node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "rui_drone", "", &support));

    // Init imu publisher
    // Save valid mem for frame_id
    sensor_msgs__msg__Imu__init(&imu_msg);
    rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu_link");
    imu_msg.orientation_covariance[0] = -1.0;

    RCCHECK(rclc_publisher_init_default(
        &imu_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu_data"));
    
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
        "params", &params_qos));

    // Init cmd vel sub
    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped),
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped),
        "cmd_vel"));

    // Init take off service
    RCCHECK(rclc_service_init_default(
        &takeoff_srv, &node,
        ROSIDL_GET_SRV_TYPE_SUPPORT(my_msgs, srv, Takeoff),
        "takeoff_srv"
    ));

    // Initialitation for simulation pub/sub
    #ifdef CONFIG_SIMULATION_ON
        // Init imu sub
        RCCHECK(rclc_subscription_init_default(
            &imu_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
            "imu_4_drone"));

        // Init height sub
        RCCHECK(rclc_subscription_init_default(
            &height_sub, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped),
            "h_4_drone"));

        // Init motors pubs
        char topic_name[16];
        for (int x = 0; x < N_MOTORS; x++) {
            // vel_m1, vel_m2...
            snprintf(topic_name, sizeof(topic_name), "vel_m%d", x + 1); 

            RCCHECK(rclc_publisher_init_default(
                &motors_pub[x],
                &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TwistStamped),
                topic_name));
        }

    #endif

    // Init main timer
    rcl_timer_t timer;
    const unsigned int timer_timeout = 1000;
    RCCHECK(rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(timer_timeout),
        timer_callback,
        true));

    // Add everything to the executor
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, N_HANDLERS, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &params_sub, &recv_msg,
        &param_callback, ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg,
        &cmd_vel_callback, ON_NEW_DATA));

    RCCHECK(rclc_executor_add_service(&executor, &takeoff_srv, &takeoff_req,
        &takeoff_res, takeoff_callback));

    #ifdef CONFIG_SIMULATION_ON
    RCCHECK(rclc_executor_add_subscription(&executor, &imu_sub,
        &imu_sub_msg, &imu_callback, ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(&executor, &height_sub,
        &h_sub_msg, &height_callback, ON_NEW_DATA));
    #endif


    int64_t last_log_us = esp_timer_get_time();

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(500));
        usleep(10000);

        int64_t now_us = esp_timer_get_time();
        if (now_us - last_log_us >= 5000000) {
            last_log_us = now_us;
            UBaseType_t free_words = uxTaskGetStackHighWaterMark(NULL);
            ESP_LOGI("uros_task", "stack libre (min historico): %u bytes",
                     (unsigned)(free_words * sizeof(StackType_t)));
        }
    }
}

void ros_init(void) {
    if (is_init) {
        return;
    }

#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
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