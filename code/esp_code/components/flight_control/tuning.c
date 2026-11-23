/**
 * Made by Rui B.S.
 * Date: 08/10/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Tuning task: changes the parameters (gains of the attitude controller,
 *   flight and odometry parameters) while the drone (or the simulation) is
 *   running. It reads text lines from the serial console (USB-CDC, the same
 *   cable of idf.py monitor), so it does not use WiFi or ROS. Just type in
 *   the monitor and press Enter:
 *
 *     help                 list of commands
 *     get                  print the current parameters
 *     kp_rate 8            change one parameter ("kp_rate=8" also works)
 *     reset                go back to the menuconfig values
 *
 *   The names and the allowed ranges are the ones of param_info[]
 *   (parameters.c), the same of the drone/params ROS topic.
 *
 * Functions:
 *   - print_params(): logs the current parameters.
 *   - print_help(): logs the list of commands.
 *   - tuning_parse_line(): runs one command line.
 *   - tuning_task(): FreeRTOS task that reads the console.
 *   - init_tuning(): creates the tuning task.
 */

#include "sdkconfig.h"

#ifdef CONFIG_GAINS_TUNE_ON
#include <ctype.h>
#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <unistd.h>

#include "tuning.h"
#include "parameters.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"


#define TUNING_TASK_PERIOD_MS 20 // Time between reads when there is no input
#define LINE_LEN 64

static const char *TAG = "TUNING";

static bool is_init = false;


static void print_params(void) {
    params_t p;
    get_all_params(&p);
    for (size_t i = 0; i < n_param_info; i++) {
        float v = *(const float *)((const char *)&p + param_info[i].offset);
        ESP_LOGI(TAG, "%-16s = %.4f", param_info[i].name, v);
    }
}

static void print_help(void) {
    ESP_LOGI(TAG, "Commands: help | get | reset | <name> <value>");
    for (size_t i = 0; i < n_param_info; i++) {
        ESP_LOGI(TAG, "  %-16s %.3f .. %.3f", param_info[i].name,
                 param_info[i].min, param_info[i].max);
    }
    ESP_LOGI(TAG, "Example: \"kp_vel 0.08\"");
}

bool tuning_parse_line(const char *line) {
    char buf[LINE_LEN];
    strncpy(buf, line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    // "name=value" is the same as "name value"
    for (char *c = buf; *c; c++) {
        if (*c == '=') {
            *c = ' ';
        }
    }

    char *save = NULL;
    char *name = strtok_r(buf, " \t", &save);
    char *value = strtok_r(NULL, " \t", &save);

    if (name == NULL) {
        return false;
    }

    if (strcasecmp(name, "help") == 0) {
        print_help();
        return true;
    }
    if (strcasecmp(name, "get") == 0) {
        print_params();
        return true;
    }
    if (strcasecmp(name, "reset") == 0) {
        params_init(); // menuconfig values
        print_params();
        return true;
    }

    const param_info_t *info = params_find(name);
    if (info == NULL) {
        ESP_LOGW(TAG, "Unknown command \"%s\" (type help)", name);
        return false;
    }

    char *end = NULL;
    float v = value ? strtof(value, &end) : NAN;
    if (value == NULL || end == value || *end != '\0' || !isfinite(v)
        || v < info->min || v > info->max) {
        ESP_LOGW(TAG, "%s: invalid value (allowed %.3f .. %.3f)",
                 info->name, info->min, info->max);
        return false;
    }

    // Only this field changes: the others are PARAM_KEEP
    params_t p;
    get_all_params(&p);
    float old = *(const float *)((const char *)&p + info->offset);

    params_keep_all(&p);
    *(float *)((char *)&p + info->offset) = v;
    if (params_update(&p) != NULL) {
        return false;
    }

    ESP_LOGI(TAG, "%s: %.4f -> %.4f", info->name, old, v);
    return true;
}

static void tuning_task(void *arg) {
    (void)arg;
    char line[LINE_LEN];
    int len = 0;

    print_help();
    print_params();

    while (1) {
        // read() of 1 byte, not fgetc(): stdio asks the USB-CDC driver for a
        // full buffer (128 B) and, in non-blocking mode, the driver only answers
        // when all those bytes are there (its RX buffer is 64 B), so fgetc()
        // never gets anything.
        unsigned char ch;
        if (read(STDIN_FILENO, &ch, 1) != 1) {
            // No input: the console is non-blocking, wait a bit
            vTaskDelay(pdMS_TO_TICKS(TUNING_TASK_PERIOD_MS));
            continue;
        }
        int c = ch;

        // End of line (the monitor can send \r, \n or both)
        if (c == '\r' || c == '\n') {
            if (len > 0) {
                line[len] = '\0';
                tuning_parse_line(line);
                len = 0;
            }
            continue;
        }

        // Backspace
        if (c == '\b' || c == 0x7f) {
            if (len > 0) {
                len--;
            }
            continue;
        }

        if (isprint(c) && len < LINE_LEN - 1) {
            line[len++] = (char)c;
        }
    }
}

void init_tuning(void) {
    if (is_init) {
        return;
    }

    xTaskCreate(tuning_task,
        "tuning_task",
        CONFIG_TUNING_TASK_STACK,
        NULL,
        CONFIG_TUNING_TASK_PRIO,
        NULL);

    is_init = true;
}

#endif // CONFIG_TUNE_ON