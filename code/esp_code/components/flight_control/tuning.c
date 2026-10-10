/**
 * Made by Rui B.S.
 * Date: 08/10/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Tuning task: changes the gains of the attitude controller while the
 *   drone (or the simulation) is running. It reads text lines from the
 *   serial console (USB-CDC, the same cable of idf.py monitor), so it does
 *   not use WiFi or ROS. Just type in the monitor and press Enter:
 *
 *     help                 list of commands
 *     get                  print the current gains
 *     kp_rate 8            change one gain ("kp_rate=8" also works)
 *     reset                go back to the default gains
 *
 *   Gains: kp_h, kp_angle, kp_rate, max_rate, max_pow, base.
 *
 * Functions:
 *   - print_gains(): logs the current gains.
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
#include "attitude_controller.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"


#define TUNING_TASK_PERIOD_MS 20 // Time between reads when there is no input
#define LINE_LEN 64

static const char *TAG = "TUNING";

static bool is_init = false;
static GAINS default_gains;

// Name of each gain, its place in GAINS and the allowed values
typedef struct {
    const char *name;
    size_t offset;
    float min;
    float max;
} GAIN_INFO;

static const GAIN_INFO gain_info[] = {
    { "kp_h",     offsetof(GAINS, kp_h),        0.0f, 5000.0f },
    { "kp_angle", offsetof(GAINS, kp_angle),    0.0f,  100.0f },
    { "kp_rate",  offsetof(GAINS, kp_rate),     0.0f,  500.0f },
    { "max_rate", offsetof(GAINS, max_rate),    0.0f, 2000.0f },
    { "max_pow",  offsetof(GAINS, max_pow_rpy), 0.0f, 3000.0f },
    { "base",     offsetof(GAINS, base),        0.0f, 3000.0f },
};
#define N_GAINS (sizeof(gain_info) / sizeof(gain_info[0]))


static void print_gains(void) {
    GAINS k;
    get_gains(&k);
    ESP_LOGI(TAG, "kp_h=%.3f kp_angle=%.3f kp_rate=%.3f max_rate=%.1f max_pow=%.1f base=%.1f",
             k.kp_h, k.kp_angle, k.kp_rate, k.max_rate, k.max_pow_rpy, k.base);
}

static void print_help(void) {
    ESP_LOGI(TAG, "Commands: help | get | reset | <gain> <value>");
    ESP_LOGI(TAG, "Gains: kp_h kp_angle kp_rate max_rate max_pow base  (e.g. \"kp_rate 8\")");
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
        print_gains();
        return true;
    }
    if (strcasecmp(name, "reset") == 0) {
        set_gains(&default_gains);
        print_gains();
        return true;
    }

    for (size_t i = 0; i < N_GAINS; i++) {
        if (strcasecmp(name, gain_info[i].name) != 0) {
            continue;
        }

        char *end = NULL;
        float v = value ? strtof(value, &end) : NAN;
        if (value == NULL || end == value || *end != '\0' || !isfinite(v)
            || v < gain_info[i].min || v > gain_info[i].max) {
            ESP_LOGW(TAG, "%s: invalid value (allowed %.1f .. %.1f)",
                     gain_info[i].name, gain_info[i].min, gain_info[i].max);
            return false;
        }

        // Read - change - write, so the other gains are not lost
        GAINS k;
        get_gains(&k);
        float *field = (float *)((char *)&k + gain_info[i].offset);
        float old = *field;
        *field = v;
        set_gains(&k);

        ESP_LOGI(TAG, "%s: %.3f -> %.3f", gain_info[i].name, old, v);
        return true;
    }

    ESP_LOGW(TAG, "Unknown command \"%s\" (type help)", name);
    return false;
}

static void tuning_task(void *arg) {
    (void)arg;
    char line[LINE_LEN];
    int len = 0;

    print_help();
    print_gains();

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
    get_gains(&default_gains);

    xTaskCreate(tuning_task, "tuning_task", CONFIG_TUNING_TASK_STACK, NULL, CONFIG_TUNING_TASK_PRIO, NULL);
    is_init = true;
}

#endif // CONFIG_TUNE_ON