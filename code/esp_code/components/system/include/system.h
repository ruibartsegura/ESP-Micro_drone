#ifndef SYSTEM_H
#define SYSTEM_H

#include <stdbool.h>
#include <stdint.h>


#include "state.h"
#include "attitude_controller.h"

void system_init(void);

bool system_test(void);

//void system_launch(void);

/**
 * @brief Lanza la tarea FreeRTOS que ejecuta la máquina de estados en
 *        bucle (llama a system_launch() repetidamente). Usar esta
 *        función desde app_main(), NO llamar a system_launch() directo
 *        una sola vez, o el estado se quedará congelado en el primer
 *        caso que ejecute.
 */
void system_start(void);

ATTITUDE_TARGET get_attitude();

// Checkers
bool check_takeOff_2_hov(float h);

#endif //SYSTEM_H