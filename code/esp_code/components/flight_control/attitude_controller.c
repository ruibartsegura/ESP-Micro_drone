/**
 * Made by Rui B.S.
 * Date: 29/07/2026
 * email: rui.bartolome@gmail.com
 *
 */

#include <math.h>

#include "attitude_controller.h"

#include "imu.h"
#include "system.h"
#include "ros_coordinator.h"
#include "motors.h"

#include "esp_log.h"



#define SYSTEM_TASK_PERIOD_MS 15 // 150HZ*

// Complementay
#define ALPHA 0.98 // Complementary filter constant

#define p 1121 // Air pressure at N.C. & 600m over sea level
#define A 0.003 // frontal cross-sectional area approx
#define C 1.0 // drag coefficient approx
#define g 9.8 // Gravity
#define m 16.5 // Drone mass
#define MAX_ANGLE 0.52 // 30º in rad

// PID
#define KP 1 // Proportional
#define KI 1 // Integrative
#define KD 1 // Derivative

#define throttle_base 100 // Min throttle to hover

RPY last_rpy;

float complementary_filter(float angle, float gyro, float alpha){
  return (alpha * angle + (1.0 - alpha) * gyro);
}

// ============================================================
//                  Getting Roll Pitch Yaw
// ============================================================
void get_roll_pitch(float *roll, float *pitch, IMU *imu_d) {
  float dt = (imu_d->Time_stamp - last_rpy.t_stamp) / 1000000.0f; // Time difference

  float roll_acc  = atan2f(imu_d->AcY_g, imu_d->AcZ_g) * 180.0f / M_PI;
  float pitch_acc = atan2f(-imu_d->AcX_g, sqrtf(imu_d->AcY_g*imu_d->AcY_g + imu_d->AcZ_g*imu_d->AcZ_g));


  float roll_angle = (last_rpy.roll + imu_d->GyX_dps * dt);
  float pitch_angle = (last_rpy.pitch  + imu_d->GyY_dps * dt);


  *roll = complementary_filter(roll_angle, roll_acc, ALPHA);
  *pitch = complementary_filter(pitch_angle, pitch_acc, ALPHA);

  last_rpy.roll = *roll;
  last_rpy.pitch = *pitch;
  last_rpy.t_stamp = imu_d->Time_stamp;
  return;
}


// ============================================================
//               CMD_VEL -> desired Roll/Pitch
// ============================================================
void cmd_vel_2_RP(float *targ_roll, float *targ_pitch, float *targ_yaw, geometry_msgs__msg__Twist cmd_vel) {
    float roll, pitch, vx, vy, sign_x, sign_y;

    // Get input velocity
    vx = cmd_vel.linear.x;
    vy = cmd_vel.linear.y;

    // Check forward o backward
    if (vx >= 0) {
      sign_x = 1.0;
    } else {
      sign_x = -1.0;
    }

    if (vy >= 0) {
      sign_y = 1.0;
    } else {
      sign_y = -1.0;
    }

    // Get the roll and pitch for the input vel
    roll  = sign_x * atan2f((C*rho*A*vx*vx), (2*m*g));
    pitch = sign_y * atan2f((C*rho*A*vy*vy), (2*m*g));

    // Clamp result at 30º max
    if (roll > MAX_ANGLE) {
        roll = MAX_ANGLE;
    }
    if (pitch > MAX_ANGLE) {
        pitch = MAX_ANGLE;
    }

    *targ_roll = roll;
    *targ_pitch = pitch;
    *targ_yaw = cmd_vel.angular.z;
    return;
}


// ============================================================
//                        Attitude Main
// ============================================================
void control_attitude(ATTITUDE_TARGET attitude_target) {
  IMU imu_d;

  // External loop
  float roll, pitch;
  float targ_roll, targ_pitch, targ_yaw;
  float err_roll, err_pitch, targ_yaw;
  
  // Internal loop
  float roll_rate, pitch_rate, yaw_rate;
  float targ_roll_rate, targ_pitch_rate, targ_yaw_rate;
  float err_roll_rate, err_pitch_rate, err_yaw_rate;

  // Power for the motors
  float pow_roll, pow_pitch, pow_yaw;
  float motor1, motor2, motor3, motor4;


  // Get roll & pitch
  esp_err_t err = imu_get_data(&imu_d);

  get_roll_pitch(&roll, &pitch, &imu_d);

  // Get target roll & pitch
  cmd_vel_2_RP(&targ_roll, &targ_pitch, &targ_yaw, attitude_target.cmd_vel);

  // Get the error in the roll and pitch
  err_roll = targ_roll - roll;
  err_pitch = targ_pitch - pitch;

  // Get the rate of roll & pitch
  targ_roll_rate = KP * err_roll;
  targ_pitch_rate = KP * err_pitch; // TODO: CLAMP | Cambiar nombre IMU acc_lin & roll_rate

  // Get measured roll, pitch, yaw rate
  roll_rate = imu_d.GyX_dps_;
  pitch_rate = imu_d.GyY_dps; 
  yaw_rate = imu_d.GyZ_dps;

  // Get diff between measured and desired
  err_roll_rate = targ_roll_rate - roll_rate;
  err_pitch_rate = targ_pitch_rate - pitch_rate;
  err_yaw_rate = targ_yaw_rate - yaw_rate;


  pow_roll = KP * err_roll_rate;
  pow_pitch = KP * err_pitch_rate;
  pow_yaw = KP * err_yaw_rate; // TODO hacer bien PID

  // TODO añadir h al controlador

  // Motors power
  motor1 = throttle_base + pow_roll - pow_pitch - pow_yaw;
  motor2 = throttle_base - pow_roll - pow_pitch + pow_yaw;
  motor3 = throttle_base - pow_roll + pow_pitch - pow_yaw;
  motor4 = throttle_base + pow_roll + pow_pitch + pow_yaw;

  // TODO convertir potencia a porcentaje
  motor_set_speed(1, motor1);
  motor_set_speed(2, motor2);
  motor_set_speed(3, motor3);
  motor_set_speed(4, motor4);
}

static void attitude_task(void *arg) {
    (void)arg;
    int log_counter = 0;

    while (1) {
      control_attitude();

      if (++log_counter >= 250) {
          log_counter = 0;
          UBaseType_t free_words = uxTaskGetStackHighWaterMark(NULL);
          ESP_LOGI("system_task", "stack libre (min historico): %u bytes",
                    (unsigned)(free_words * sizeof(StackType_t)));
      }


      // When the drone is disarmed the attitude controller is no longer needed
      if (get_state() == 7) { // 7 = Disarming
        vTaskDelete(NULL); // Delete task
      }

      vTaskDelay(pdMS_TO_TICKS(SYSTEM_TASK_PERIOD_MS));

  }
}

// It will be initialized when the drone is armed
void init_attitude_controller() {
   xTaskCreate(attitude_task, "attitude_task", CONFIG_SYSTEM_TASK_STACK, NULL, CONFIG_SYSTEM_TASK_PRIO, NULL);
}


// TODO
//  Revisar lógica de attitude_controller, que no dependa de system pero tome los calores de ahí