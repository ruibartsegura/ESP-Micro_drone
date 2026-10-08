/**
 * Made by Rui B.S.
 * Date: 23/05/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   micro-ROS interface of the drone. It connects to the micro-ROS agent
 *   over WiFi (UDP), creates the "rui_drone" node and runs the executor in
 *   its own FreeRTOS task. Shared variables are protected with a mutex.
 *     - Publishes:  drone/imu_data (sensor_msgs/Imu)
 *     - Subscribes: drone/cmd_vel (geometry_msgs/TwistStamped),
 *                   drone/params (my_msgs/Params)
 *     - Service:    drone/takeoff_srv (my_msgs/Takeoff)
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
 *   - fill_imu_msg(): fills the IMU message from the global state.
 *   - imu_callback(), height_callback(): (simulation) save the sensor data from Gazebo.
 *   - motors_msg_init(), pub_motor_speed(): (simulation) prepare and send the motor speeds.
 *   - timer_callback(): publishes the IMU data periodically.
 *   - micro_ros_task(): creates the node, publishers, subscribers, service and executor.
 *   - ros_init(): starts the network interface and the micro-ROS task.
 *   - ros_test(): checks that the module started correctly.
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
#include "esp_wifi.h"

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

// My includes
#include "led.h"
#include "imu.h"
#include "system.h"
#include "parameters.h"
#include "state.h"
#include "sdkconfig.h"

#include <pthread.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#include <rmw/qos_profiles.h>
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ \
    printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); \
    vTaskDelete(NULL);}}

#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){ \
        printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

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


// ============================================================
//                       Parameters
// ============================================================
static const float MAX_ALT = 3.0f; // TODO PONER PARAM
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
my_msgs__msg__Params recv_msg;

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
//                           Mutex
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
    sensor_msgs__msg__FluidPressure h_sub_msg;

    // Publishers
    //#define NUM_MOTORS 4
    static double motors_buf[NUM_MOTORS];
    static char   frame_id_buf[] = "base_link";

    // Necesary because the publisher is called from another task
    static volatile bool motors_pub_ready = false;

    // Queue of 1 for new motors msg
    typedef struct { double v[NUM_MOTORS]; } motors_cmd_t;
    static QueueHandle_t motors_q = NULL;

    rcl_publisher_t motors_pub;
    actuator_msgs__msg__Actuators motors_msg;
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

    sm_states_t state;
    get_sm_state(&state);

    ESP_LOGI(TAG, "TAKEOFF: received alt=%.2f state=%d", takeoff_req->altitude, (int)state);

    // Check drone state, to just accept the take off when the drone is armed and waiting to take off
    if (state == ERROR) { // Error state
        ESP_LOGW(TAG, "TAKEOFF: rejected - ERROR state");
        takeoff_res->accepted = false;
        rosidl_runtime_c__String__assign(&takeoff_res->reason, "Status error");
        return;
    } else if (state != ARMING) { // Not Arming
        ESP_LOGW(TAG, "TAKEOFF: rejected - state=%d not ARMING(%d)", (int)state, (int)ARMING);
        takeoff_res->accepted = false;
        rosidl_runtime_c__String__assign(&takeoff_res->reason, "Not armed");
        return;
    }

    // Check requested altitude
    if (takeoff_req->altitude < MIN_ALT || takeoff_req->altitude > MAX_ALT) {
        ESP_LOGW(TAG, "TAKEOFF: rejected - alt=%.2f out of [%.2f, %.2f]",
                 takeoff_req->altitude, MIN_ALT, MAX_ALT);
        takeoff_res->accepted = false;
        rosidl_runtime_c__String__assign(&takeoff_res->reason, "Invalid altitude");
        return;
    }

    // Take off -> ready
    pthread_mutex_lock(&lock);
    take_off_ready = true;
    altitude = takeoff_req->altitude;
    pthread_mutex_unlock(&lock);

    ESP_LOGI(TAG, "TAKEOFF: accepted alt=%.2f", takeoff_req->altitude);
    takeoff_res->accepted = true;
    rosidl_runtime_c__String__assign(&takeoff_res->reason, "OK");
}


// ============================================================
//                         IMU publisher
// ============================================================

// Fill IMU msg
void fill_imu_msg(sensor_msgs__msg__Imu * out) {
    int64_t now_ms = esp_timer_get_time();

    out->header.stamp.sec = now_ms / 1000;
    out->header.stamp.nanosec = (now_ms % 1000) * 1000000;

    vec3_t vel_ang, acc_lin;
    get_acc_lin(&acc_lin);
    get_vel_ang(&vel_ang);


    // FIX: estaban intercambiados (aceleración en angular_velocity y
    // giro en linear_acceleration) y sin convertir unidades.
    // sensor_msgs/Imu espera angular_velocity en rad/s y
    // linear_acceleration en m/s².
    out->angular_velocity.x = vel_ang.x * DEG_TO_RAD;
    out->angular_velocity.y = vel_ang.y * DEG_TO_RAD;
    out->angular_velocity.z = vel_ang.z * DEG_TO_RAD;

    out->angular_velocity.x = vel_ang.x * GRAVITY_MS2;
    out->angular_velocity.y = vel_ang.y * GRAVITY_MS2;
    out->angular_velocity.z = vel_ang.z * GRAVITY_MS2;
}


// ============================================================
//                    Simulation Subs/Pubs
// ============================================================
#ifdef CONFIG_SIMULATION_ON
    static volatile uint32_t imu_recv_cnt = 0;

    // Imu Callback
    void imu_callback(const void * msgin) {
        const sensor_msgs__msg__Imu * new_msg = (const sensor_msgs__msg__Imu *)msgin;
        imu_recv_cnt++;
        pthread_mutex_lock(&lock);
        imu_sub_msg = *new_msg;
        pthread_mutex_unlock(&lock);

        // Gazebo sends rad/s and m/s^2, the real driver (and the controller)
        // uses deg/s and g
        set_vel_ang(
            imu_sub_msg.angular_velocity.x / DEG_TO_RAD,
            imu_sub_msg.angular_velocity.y / DEG_TO_RAD,
            imu_sub_msg.angular_velocity.z / DEG_TO_RAD
        );

        set_acc_lin(
            imu_sub_msg.linear_acceleration.x / GRAVITY_MS2,
            imu_sub_msg.linear_acceleration.y / GRAVITY_MS2,
            imu_sub_msg.linear_acceleration.z / GRAVITY_MS2
        );

        // Full time in ns (only nanosec goes back to 0 every second)
        set_time_imu((int64_t)imu_sub_msg.header.stamp.sec * 1000000000LL
                     + imu_sub_msg.header.stamp.nanosec);
    }

    // Height Callback
    void height_callback(const void * msgin) {
        const sensor_msgs__msg__FluidPressure * new_msg = (const sensor_msgs__msg__FluidPressure *)msgin;
        pthread_mutex_lock(&lock);
        h_sub_msg = *new_msg;
        pthread_mutex_unlock(&lock);

        set_h(bmp180_pressure_to_altitude(h_sub_msg.fluid_pressure, 101325.0f));
        set_time_height((int64_t)h_sub_msg.header.stamp.sec * 1000000000LL
                        + h_sub_msg.header.stamp.nanosec);
    }

    // Motors
    void motors_msg_init(void) {
       actuator_msgs__msg__Actuators__init(&motors_msg);  // opcional, pero deja todo en estado válido

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

        // ESP_LOGI(TAG, "VEL LLEGA");
        if (timer == NULL || !motors_pub_ready) return;
        
        if (xQueueReceive(motors_q, &cmd, 0) != pdTRUE) return;  // nada nuevo
        // ESP_LOGI(TAG, "VEL PASA");

        int64_t now_us = esp_timer_get_time();
        for (int i = 0; i < NUM_MOTORS; i++) {
            motors_msg.velocity.data[i] = cmd.v[i];
            
        }
        motors_msg.header.stamp.sec     = (int32_t)(now_us / 1000000);
        motors_msg.header.stamp.nanosec = (uint32_t)((now_us % 1000000) * 1000);
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

        fill_imu_msg(&imu_msg);
        RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));

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

    RCCHECK(rclc_publisher_init_best_effort(
        &imu_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "drone/imu_data"));
    
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
        // Init imu sub - depth=1: only keep latest, avoid executor callback bursts
        {
            rmw_qos_profile_t sim_qos = rmw_qos_profile_sensor_data;
            sim_qos.depth = 1;
            RCCHECK(rclc_subscription_init(&imu_sub, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data", &sim_qos));
        }

        // Init barometer sub - depth=1: same reason
        {
            rmw_qos_profile_t sim_qos = rmw_qos_profile_sensor_data;
            sim_qos.depth = 1;
            RCCHECK(rclc_subscription_init(&height_sub, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, FluidPressure), "barometer/data", &sim_qos));
        }
        
        // Init motors pub - RELIABLE: Gazebo bridge subscriber requires RELIABLE
        {
            rmw_qos_profile_t motors_qos = rmw_qos_profile_default;
            motors_qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
            motors_qos.depth   = 2;
            RCCHECK(rclc_publisher_init(&motors_pub, &node,
                ROSIDL_GET_MSG_TYPE_SUPPORT(actuator_msgs, msg, Actuators),
                "/drone/command/motor_speed", &motors_qos));
        }
        motors_pub_ready = true;
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

    // Add everything to the executor, the order matter
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, N_HANDLERS, &allocator));
    
    RCCHECK(rclc_executor_add_timer(&executor, &timer));

    RCCHECK(rclc_executor_add_service(&executor, &takeoff_srv, &takeoff_req,
        &takeoff_res, takeoff_callback));

    RCCHECK(rclc_executor_add_subscription(&executor, &params_sub, &recv_msg,
        &param_callback, ON_NEW_DATA));

    RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg,
        &cmd_vel_callback, ON_NEW_DATA));

    #ifdef CONFIG_SIMULATION_ON
        // Motors timer
        rcl_timer_t motors_timer;
        RCCHECK(rclc_timer_init_default2(&motors_timer, &support,
                RCL_MS_TO_NS(30), motors_timer_callback, true));   // ~33 Hz
        RCCHECK(rclc_executor_add_timer(&executor, &motors_timer));

        // Simulation subs
        RCCHECK(rclc_executor_add_subscription(&executor, &imu_sub,
            &imu_sub_msg, &imu_callback, ON_NEW_DATA));

        RCCHECK(rclc_executor_add_subscription(&executor, &height_sub,
            &h_sub_msg, &height_callback, ON_NEW_DATA));
    #endif


    ESP_LOGI(TAG, "Executor ready. Service valid=%d",
             (int)rcl_service_is_valid(&takeoff_srv));

    int64_t last_log_us = esp_timer_get_time();

    while (1) {
        rcl_ret_t rc = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(250));

        if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) {
            ESP_LOGW(TAG, "spin_some error: %d", (int)rc);
        }

        int64_t now_us = esp_timer_get_time();
        if (now_us - last_log_us >= 5000000) {
            last_log_us = now_us;
            UBaseType_t free_words = uxTaskGetStackHighWaterMark(NULL);
            #ifdef CONFIG_SIMULATION_ON
                // ESP_LOGI(TAG, "alive, imu_recv=%u stack_min=%u bytes",
                //          (unsigned)imu_recv_cnt,
                //          (unsigned)(free_words * sizeof(StackType_t)));
            #else
                // ESP_LOGI(TAG, "alive, stack_min=%u bytes",
                //          (unsigned)(free_words * sizeof(StackType_t)));
            #endif
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
    if (!is_init) {
        return false;
    }
    return true;
}