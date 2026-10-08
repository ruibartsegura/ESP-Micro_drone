/**
 * Made by Rui B.S.
 * Date: 08/10/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Tuning task: changes the gains of the attitude controller while the
 *   drone (or the simulation) is running, with text commands on the serial
 *   console (the same USB cable of idf.py monitor, no WiFi).
 *
 * Functions:
 *   - tuning_parse_line(): runs one command ("kp_rate 8", "get", "help"...).
 *   - init_tuning(): creates the tuning task.
 */

#ifndef TUNING_H
#define TUNING_H

#include <stdbool.h>

// Runs one command line. Returns true if the command was valid.
bool tuning_parse_line(const char *line);

// Start the task
void init_tuning(void);

#endif // TUNING_H
