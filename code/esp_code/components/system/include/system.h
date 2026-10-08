/**
 * Made by Rui B.S.
 * Date: 22/07/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Public interface of the system module (initialisation, tests and
 *   state machine of the drone).
 *
 * Functions:
 *   - system_init(): initialises all the modules.
 *   - system_test(): checks that all the modules started correctly.
 *   - system_start(): creates the FreeRTOS task that runs the state machine.
 *   - get_attitude(): returns the target attitude (cmd_vel + height).
 *   - check_takeOff_2_hov(): checks if the take-off height has been reached.
 */

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