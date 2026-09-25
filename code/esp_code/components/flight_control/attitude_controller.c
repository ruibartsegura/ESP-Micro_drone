/**
 * Made by Rui B.S.
 * Date: 29/07/2026
 * email: rui.bartolome@gmail.com
 *
 */

#include <math.h>

#include "attitude_controller.h"

#include "imu.h"
#include "height.h"
#include "system.h"
#include "ros_coordinator.h"
#include "motors.h"

#include "esp_log.h"


#define SYSTEM_TASK_PERIOD_MS 15 // 150HZ*

// Complementay
#define ALPHA 0.98 // Complementary filter constant

// Constants for formula (Vel->Position (Roll/Pitch))
#define rho 1121 // Air pressure at N.C. & 600m over sea level
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
float last_h, last_vel_Z;
float err_h;


bool check_h_reached() {
  return true; // TODO terminar
}



float complementary_filter(float a, float b, float alpha) {
  return (alpha * a + (1.0 - alpha) * b);
}

// ============================================================
//              Getting height & filtering it
// ============================================================
float get_h(float h_d, IMU *imu_d) {
  float dt = (imu_d->Time_stamp - last_rpy.t_stamp) / 1000000.0f; // Time difference

  float vel_z = imu_d->Acc_lin_Z * dt + last_vel_Z;
  float h = vel_z * dt + last_h;
  
  // Calculate the power for altitude | a = calculated h, b = sensed h
  return complementary_filter(h, h_d, ALPHA);
}

// ============================================================
//                  Getting Roll Pitch Yaw
// ============================================================
void get_roll_pitch(float *roll, float *pitch, IMU *imu_d) {
  float dt = (imu_d->Time_stamp - last_rpy.t_stamp) / 1000000.0f; // Time difference

  float roll_acc  = atan2f(imu_d->Acc_lin_Y, imu_d->Acc_lin_Z) * 180.0f / M_PI;
  float pitch_acc =
    atan2f(-imu_d->Acc_lin_X, sqrtf(imu_d->Acc_lin_Y*imu_d->Acc_lin_Y + imu_d->Acc_lin_Z*imu_d->Acc_lin_Z)) *
    180.0f / M_PI;


  float roll_angle = (last_rpy.roll + imu_d->Vel_ang_X * dt);
  float pitch_angle = (last_rpy.pitch  + imu_d->Vel_ang_Y * dt);

  // a = angle, b = angle acceleration
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
void cmd_vel_2_RP(float *targ_roll, float *targ_pitch, geometry_msgs__msg__Twist cmd_vel) {
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
    return;
}


// ============================================================
//                        Attitude Main
// ============================================================
void control_attitude() {
  // Sensors data
  IMU imu_d;
  geometry_msgs__msg__PoseStamped h_d;

  // External loop
  float roll, pitch;
  float targ_roll, targ_pitch;
  float err_roll, err_pitch;
  
  // Internal loop
  float roll_rate, pitch_rate, yaw_rate;
  float targ_roll_rate, targ_pitch_rate, targ_yaw_rate;
  float err_roll_rate, err_pitch_rate, err_yaw_rate;

  // Height
  float pow_h, err_h;

  // Power for the motors
  float pow_roll, pow_pitch, pow_yaw;
  float motor1, motor2, motor3, motor4;


  // Get the desired attitude of the drone
  ATTITUDE_TARGET attitude_target = get_attitude();

  // Get sensors data
  esp_err_t err_imu = get_imu_data(&imu_d);
  esp_err_t err_height = get_height_data(&h_d);

  // EXTERNAL LOOP
  // Get Roll & Pitch
  get_roll_pitch(&roll, &pitch, &imu_d);

  // Get target roll & pitch
  cmd_vel_2_RP(&targ_roll, &targ_pitch, attitude_target.cmd_vel);
  cmd_vel_2_RP(&targ_roll, &targ_pitch, attitude_target.cmd_vel);

  // Get the error in the roll and pitch
  err_roll = targ_roll - roll;
  err_pitch = targ_pitch - pitch;


  // INTERNAL LOOP
  // Get the rate of roll & pitch
  targ_roll_rate = KP * err_roll;
  targ_pitch_rate = KP * err_pitch; // TODO: CLAMP 
  targ_yaw_rate = attitude_target.cmd_vel.angular.z;

  // Get measured roll, pitch, yaw rate
  roll_rate = imu_d.Vel_ang_X;
  pitch_rate = imu_d.Vel_ang_Y; 
  yaw_rate = imu_d.Vel_ang_Z;

  // Get diff between measured and desired
  err_roll_rate = targ_roll_rate - roll_rate;
  err_pitch_rate = targ_pitch_rate - pitch_rate;
  err_yaw_rate = targ_yaw_rate - yaw_rate;

  // Calculate the power needed for the motors
  pow_roll = KP * err_roll_rate;
  pow_pitch = KP * err_pitch_rate;
  pow_yaw = KP * err_yaw_rate; // TODO hacer bien PID


  // ALTITUDE
  float h = get_h(h_d.pose.position.z, &imu_d);
  pow_h = throttle_base + h;
  
  err_h = attitude_target.h - h; // Value to check in check_h_reached.

  // TODO unidades de throttle_base(Se puede hacer parametro del kconfig)

  // Motors power.
  motor1 = pow_h + pow_roll - pow_pitch - pow_yaw;
  motor2 = pow_h - pow_roll - pow_pitch + pow_yaw;
  motor3 = pow_h - pow_roll + pow_pitch - pow_yaw;
  motor4 = pow_h + pow_roll + pow_pitch + pow_yaw;

  // Set mottor speed
  set_motor_speed(1, motor1);
  set_motor_speed(2, motor2);
  set_motor_speed(3, motor3);
  set_motor_speed(4, motor4);
  // Set mottor speed
  set_motor_speed(1, motor1);
  set_motor_speed(2, motor2);
  set_motor_speed(3, motor3);
  set_motor_speed(4, motor4);
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

      vTaskDelay(pdMS_TO_TICKS(SYSTEM_TASK_PERIOD_MS));

  }
}


void init_attitude_controller() {
   xTaskCreate(attitude_task, "attitude_task", CONFIG_SYSTEM_TASK_STACK, NULL, CONFIG_SYSTEM_TASK_PRIO, NULL);
}


// TODO:
//   *  Cambiar controll atitude para hacer "PID" en funciones
//
//   *  Hacer test para h, R, P, Y, motor final(rpm??)
