/* Mock of driver/ledc.h (host tests). Every call is recorded. */
#pragma once
#include <stdint.h>
#include "esp_err.h"
#include "driver/gpio.h"

typedef enum { LEDC_LOW_SPEED_MODE = 0, LEDC_SPEED_MODE_MAX } ledc_mode_t;
typedef enum { LEDC_TIMER_0 = 0, LEDC_TIMER_1, LEDC_TIMER_2, LEDC_TIMER_3 } ledc_timer_t;
typedef enum { LEDC_CHANNEL_0 = 0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3,
               LEDC_CHANNEL_4, LEDC_CHANNEL_5, LEDC_CHANNEL_6, LEDC_CHANNEL_7,
               LEDC_CHANNEL_MAX } ledc_channel_t;
typedef enum { LEDC_TIMER_8_BIT = 8, LEDC_TIMER_10_BIT = 10, LEDC_TIMER_12_BIT = 12,
               LEDC_TIMER_14_BIT = 14 } ledc_timer_bit_t;
typedef enum { LEDC_AUTO_CLK = 0 } ledc_clk_cfg_t;
typedef enum { LEDC_INTR_DISABLE = 0 } ledc_intr_type_t;

typedef struct {
    ledc_mode_t      speed_mode;
    ledc_timer_bit_t duty_resolution;
    ledc_timer_t     timer_num;
    uint32_t         freq_hz;
    ledc_clk_cfg_t   clk_cfg;
} ledc_timer_config_t;

typedef struct {
    int              gpio_num;
    ledc_mode_t      speed_mode;
    ledc_channel_t   channel;
    ledc_intr_type_t intr_type;
    ledc_timer_t     timer_sel;
    uint32_t         duty;
    int              hpoint;
} ledc_channel_config_t;

extern int                   mock_ledc_timer_config_calls;
extern ledc_timer_config_t   mock_ledc_timer_conf;
extern int                   mock_ledc_channel_config_calls;
extern ledc_channel_config_t mock_ledc_channel_conf[LEDC_CHANNEL_MAX];
extern int                   mock_ledc_channel_configured[LEDC_CHANNEL_MAX];
extern uint32_t              mock_ledc_duty[LEDC_CHANNEL_MAX];      /* last ledc_set_duty()            */
extern uint32_t              mock_ledc_out_duty[LEDC_CHANNEL_MAX];  /* duty applied by ledc_update_duty */
extern int                   mock_ledc_set_calls[LEDC_CHANNEL_MAX];
extern int                   mock_ledc_update_calls[LEDC_CHANNEL_MAX];

esp_err_t ledc_timer_config(const ledc_timer_config_t *conf);
esp_err_t ledc_channel_config(const ledc_channel_config_t *conf);
esp_err_t ledc_set_duty(ledc_mode_t mode, ledc_channel_t channel, uint32_t duty);
esp_err_t ledc_update_duty(ledc_mode_t mode, ledc_channel_t channel);
