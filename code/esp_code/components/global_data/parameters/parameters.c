/**
 * Made by Rui B.S.
 * Date: 28/06/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Flight parameters that can change at runtime (hovering height, maximum
 *   velocity and land-on-site mode). The default values come from
 *   menuconfig and ROS can update them with the drone/params topic.
 *
 * Functions:
 *   - set_hovering_h() / get_hovering_h(): hovering height.
 *   - set_max_velocity() / get_max_velocity(): maximum velocity.
 *   - set_land_on_site() / get_land_on_site(): land-on-site mode.
 */

#include <stdbool.h>

#include "parameters.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>

float hov_h = CONFIG_HOVERING_H / 100.0f; // cm -> m
float vel_max = CONFIG_VEL_MAX / 100.0f; // ms (float division: 2 / 100 was 0)
bool land_on_site = CONFIG_LAND_ON_SITE;

void set_hovering_h(float h) {
    hov_h = h;
}

float get_hovering_h() {
    return hov_h;
}

void set_max_velocity(float vel) {
    vel_max = vel;
}

float get_max_velocity() {
    return vel_max;
}

void set_land_on_site(bool mode) {
    land_on_site = mode;
}

bool get_land_on_site() {
    return land_on_site;
}