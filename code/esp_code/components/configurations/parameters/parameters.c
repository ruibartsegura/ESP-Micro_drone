/**
 * Made by Rui B.S.
 * Date: 28/06/2026
 * email: rui.bartolome@gmail.com
 */

#include <stdbool.h>

#include "parameters.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>

extern float hov_h = CONFIG_HOVERING_H / 100;
extern float vel_max = CONFIG_VEL_MAX / 100;
extern bool land_on_site = CONFIG_LAND_ON_SITE;

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