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
#include "state.h"

#include "esp_log.h"

#include "sdkconfig.h"


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

// PID -> will be /100
#define KP 85 // Proportional
#define KI 100 // Integrative
#define KD 100 // Derivative

#define throttle_base 2387 // Min throttle to hover

static const char *TAG = "ATTITUDE";


// Allowed diference between target adn actual height in 
float DIFF_H_ALLOWED = 0.01; // In meters

RPY last_rpy;
float last_h, last_vel_Z;
float err_h;

bool check_h_reached() {
  if (fabsf(err_h) <= DIFF_H_ALLOWED) {
    return true;
  } else {
    return false;
  }
}

float complementary_filter(float a, float b, float alpha) {
  return (alpha * a + (1.0 - alpha) * b);
}

// ============================================================
//                  Getting Roll Pitch Yaw
// ============================================================
void get_roll_pitch(float *roll, float *pitch) {
  // Get the time of the sample
  int64_t t;
  get_time_imu(&t);

  vec3_t acc_lin = {0};
  vec3_t vel_ang = {0};

  get_acc_lin(&acc_lin);
  get_vel_ang(&vel_ang);

  float dt = (t - last_rpy.t_stamp) / 1000000000.0f; // Time difference (ns->s)
  if (dt <= 0.0f || dt > 0.1f) {
    dt = 0.0f;
  }

  float roll_acc  = atan2f(acc_lin.y, acc_lin.z) * 180.0f / M_PI;
  float pitch_acc = atan2f(-acc_lin.x, sqrtf(acc_lin.y*acc_lin.y + acc_lin.z*acc_lin.z)) * 180.0f / M_PI;


  float roll_angle = (last_rpy.roll + vel_ang.x * dt);
  float pitch_angle = (last_rpy.pitch  + vel_ang.y * dt);

  // a = angle, b = angle acceleration
  *roll = complementary_filter(roll_angle, roll_acc, ALPHA);
  *pitch = complementary_filter(pitch_angle, pitch_acc, ALPHA);

  last_rpy.roll = *roll;
  last_rpy.pitch = *pitch;
  last_rpy.t_stamp = t;
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
    roll  = sign_x * atan2f((C*rho*A*vy*vy), (2*m*g));
    pitch = sign_y * atan2f((C*rho*A*vx*vx), (2*m*g));

    // Clamp result at 30º max
    if (roll > MAX_ANGLE) {
      roll = MAX_ANGLE;
    } else if (roll < -MAX_ANGLE) {
      roll = -MAX_ANGLE;
    }
    if (pitch > MAX_ANGLE) {
      pitch = MAX_ANGLE;
    } else if (pitch < -MAX_ANGLE) {
      pitch = -MAX_ANGLE;
    }

    *targ_roll = roll;
    *targ_pitch = pitch;
    return;
}


// ============================================================
//                        Attitude Main
// ============================================================
void control_attitude() {
  // External loop
  float roll, pitch;
  float targ_roll, targ_pitch;
  float err_roll, err_pitch;
  
  // Internal loop
  float roll_rate, pitch_rate, yaw_rate;
  float targ_roll_rate, targ_pitch_rate, targ_yaw_rate;
  float err_roll_rate, err_pitch_rate, err_yaw_rate;

  // Height
  float pow_h;

  // Power for the motors
  float pow_roll, pow_pitch, pow_yaw;

  // Get the desired attitude of the drone
  ATTITUDE_TARGET attitude_target = get_attitude();

  // EXTERNAL LOOP
  // Get Roll & Pitch
  get_roll_pitch(&roll, &pitch);

  // Get target roll & pitch
  cmd_vel_2_RP(&targ_roll, &targ_pitch, attitude_target.cmd_vel);
  cmd_vel_2_RP(&targ_roll, &targ_pitch, attitude_target.cmd_vel);

  // Get the error in the roll and pitch
  err_roll = targ_roll * 180.0f/M_PI - roll;
  err_pitch = targ_pitch * 180.0f/M_PI - pitch;


  // INTERNAL LOOP
  // Get the rate of roll & pitch
  targ_roll_rate = KP * err_roll;
  targ_pitch_rate = KP * err_pitch; // TODO: CLAMP 
  targ_yaw_rate = attitude_target.cmd_vel.angular.z * 180.0f/M_PI;

  // Get measured roll, pitch, yaw rate
  vec3_t vel_ang = {0};
  get_vel_ang(&vel_ang);

  roll_rate = vel_ang.x;
  pitch_rate = vel_ang.y; 
  yaw_rate = vel_ang.z;

  // Get diff between measured and desired
  err_roll_rate = targ_roll_rate - roll_rate;
  err_pitch_rate = targ_pitch_rate - pitch_rate;
  err_yaw_rate = targ_yaw_rate - yaw_rate;

  // Calculate the power needed for the motors
  pow_roll = KP * err_roll_rate;
  pow_pitch = KP * err_pitch_rate;
  pow_yaw = KP * err_yaw_rate; // TODO hacer bien PID


  // ALTITUDE
  vec3_t pos = {0};
  get_position(&pos);
  
  // ESP_LOGI(TAG, "Final h = %f", pos.z);
  // ESP_LOGI(TAG, "Target h = %f", attitude_target.h);
  
  err_h = attitude_target.h - pos.z; // Distance between target and actual h.
  pow_h = err_h * KP;

  double power[N_MOTORS];

  // Motors power.
  power[0] = pow_h + pow_roll - pow_pitch - pow_yaw;
  power[1] = pow_h - pow_roll - pow_pitch + pow_yaw;
  power[2] = pow_h - pow_roll + pow_pitch - pow_yaw;
  power[3] = pow_h + pow_roll + pow_pitch + pow_yaw;

  // Set mottor speed
  set_motor_speed(power);

}

static void attitude_task(void *arg) {
    (void)arg;
    int log_counter = 0;

    while (1) {
      control_attitude();

      if (++log_counter >= 250) {
          log_counter = 0;
          UBaseType_t free_words = uxTaskGetStackHighWaterMark(NULL);
          // ESP_LOGI("system_task", "stack libre (min historico): %u bytes",
                    // (unsigned)(free_words * sizeof(StackType_t)));
      }

      vTaskDelay(pdMS_TO_TICKS(SYSTEM_TASK_PERIOD_MS));

  }
}


void init_attitude_controller() {
   xTaskCreate(attitude_task, "attitude_task", CONFIG_ATTITUDE_TASK_STACK, NULL, CONFIG_ATTITUDE_TASK_PRIO, NULL);
}


// TODO:
//   *  Cambiar controll atitude para hacer "PID" en funciones
//
//   *  Hacer test para h, R, P, Y, motor final(rpm??)
