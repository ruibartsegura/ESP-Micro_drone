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
