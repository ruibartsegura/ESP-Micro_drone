/**
 * Made by Rui B.S.
 * Date: 28/06/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Public interface of the flight parameters.
 *
 * Functions:
 *   - set_hovering_h() / get_hovering_h(): hovering height.
 *   - set_max_velocity() / get_max_velocity(): maximum velocity.
 *   - set_land_on_site() / get_land_on_site(): land-on-site mode.
 */

#ifndef PARAMETERS_H
#define PARAMETERS_H

#include "sdkconfig.h"
#include <stdbool.h>

// X/100 due to the Configuration it is in cm
extern float hov_h;
extern float vel_max;
extern bool land_on_site;

void set_hovering_h(float h);
float get_hovering_h();

void set_max_velocity(float vel);
float get_max_velocity();

void set_land_on_site(bool mode);
bool get_land_on_site();


#endif // PARAMETERS_H
