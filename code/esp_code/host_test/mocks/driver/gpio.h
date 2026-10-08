/* Mock of driver/gpio.h (host tests). Every call is recorded. */
#pragma once
#include <stdint.h>
#include "esp_err.h"

typedef int gpio_num_t;
typedef enum { GPIO_MODE_DISABLE = 0, GPIO_MODE_INPUT = 1, GPIO_MODE_OUTPUT = 2 } gpio_mode_t;
typedef enum { GPIO_PULLUP_DISABLE = 0, GPIO_PULLUP_ENABLE = 1 } gpio_pullup_t;
typedef enum { GPIO_PULLDOWN_DISABLE = 0, GPIO_PULLDOWN_ENABLE = 1 } gpio_pulldown_t;
typedef enum { GPIO_INTR_DISABLE = 0 } gpio_int_type_t;

typedef struct {
    uint64_t        pin_bit_mask;
    gpio_mode_t     mode;
    gpio_pullup_t   pull_up_en;
    gpio_pulldown_t pull_down_en;
    gpio_int_type_t intr_type;
} gpio_config_t;

#define MOCK_GPIO_PINS 64
#define MOCK_GPIO_LOG  512
typedef struct { int pin; int level; } mock_gpio_event_t;

extern int               mock_gpio_level[MOCK_GPIO_PINS];      /* -1 = never written */
extern int               mock_gpio_set_calls[MOCK_GPIO_PINS];
extern int               mock_gpio_mode[MOCK_GPIO_PINS];       /* -1 = not configured */
extern int               mock_gpio_config_calls;
extern mock_gpio_event_t mock_gpio_log[MOCK_GPIO_LOG];
extern int               mock_gpio_log_len;

esp_err_t gpio_config(const gpio_config_t *conf);
esp_err_t gpio_set_level(gpio_num_t pin, uint32_t level);
