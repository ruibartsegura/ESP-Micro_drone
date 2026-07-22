#ifndef SYSTEM_H
#define SYSTEM_H

#include <stdbool.h>
#include <stdint.h>

void system_init(void);

bool system_test(void);

void system_launch(void);

/**
 * @brief Lanza la tarea FreeRTOS que ejecuta la máquina de estados en
 *        bucle (llama a system_launch() repetidamente). Usar esta
 *        función desde app_main(), NO llamar a system_launch() directo
 *        una sola vez, o el estado se quedará congelado en el primer
 *        caso que ejecute.
 */
void system_start(void);

// State machine
static const int INIT               = 0;
static const int CHECKING           = 1;
static const int ARMING             = 2;
static const int TAKING_OFF         = 3;
static const int HOVERING           = 4;
static const int EXTERNAL_CONTROL   = 5;
static const int LANDING            = 6;
static const int DISARMING          = 7;
static const int ERROR              = 8;

extern int state;
void change_state(int new_state);
int get_state();

// Checkers
bool check_takeOff_2_hov(float h);

#endif //SYSTEM_H