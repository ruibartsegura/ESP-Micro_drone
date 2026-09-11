/**
 * Made by Rui B.S.
 * Date: 27/07/2026
 * email: rui.bartolome@gmail.com
 */

#include <stdio.h>
#include <stdlib.h>
#include "imu.h"

long tiempo_prev; 

float data_filter(float a, float b, float alpha = ALPHA){
  result = alpha * a + (1.0 - alpha) * b;
  return result;
}
