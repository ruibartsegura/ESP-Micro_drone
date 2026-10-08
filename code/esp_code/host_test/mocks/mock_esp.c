/**
 * Made by Rui B.S.
 * Date: 08/10/2026
 * email: rui.bartolome@gmail.com
 *
 * Description:
 *   Implementation of the ESP-IDF and FreeRTOS mocks for the host tests:
 *   FreeRTOS tasks, delays, critical sections and queues, esp_timer, GPIO,
 *   LEDC, I2C (fake register map), WiFi and the network interface.
 *   Every function records what it receives so the tests can check it.
 *
 * Functions:
 *   - mock_reset_esp(): resets all the ESP-IDF / FreeRTOS mocks.
 *   - mock_run_task(): runs a task loop and stops it after N delays.
 *   - mock_find_task(): finds a task created with xTaskCreate().
 *   - mock_i2c_set_be16(), mock_i2c_count_writes(), mock_i2c_last_write(): I2C helpers.
 *   - The rest are the mocked ESP-IDF / FreeRTOS functions.
 */
#include <setjmp.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mock_reset.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "uros_network_interfaces.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"

/* ================= esp_err ================= */
int mock_esp_error_check_fails;
int mock_esp_error_check_last;

const char *esp_err_to_name(esp_err_t code) { (void)code; return "MOCK_ERR"; }

void mock_esp_error_check_fail(esp_err_t err, const char *file, int line) {
    (void)file; (void)line;
    mock_esp_error_check_fails++;
    mock_esp_error_check_last = err;
}

/* ================= esp_timer ================= */
int64_t mock_time_us;
int64_t mock_time_step_us;

int64_t esp_timer_get_time(void) {
    int64_t t = mock_time_us;
    mock_time_us += mock_time_step_us;
    return t;
}

/* ================= WiFi / netif ================= */
int            mock_wifi_set_ps_calls;
wifi_ps_type_t mock_wifi_ps_mode;
esp_err_t      mock_wifi_set_ps_ret;
int            mock_netif_init_calls;
esp_err_t      mock_netif_init_ret;

esp_err_t esp_wifi_set_ps(wifi_ps_type_t type) {
    mock_wifi_set_ps_calls++;
    mock_wifi_ps_mode = type;
    return mock_wifi_set_ps_ret;
}

esp_err_t uros_network_interface_initialize(void) {
    mock_netif_init_calls++;
    return mock_netif_init_ret;
}

/* ================= FreeRTOS ================= */
int mock_critical_depth;
int mock_critical_enter_count;

void mock_port_enter_critical(portMUX_TYPE *mux) {
    mux->locked++;
    mock_critical_depth++;
    mock_critical_enter_count++;
}

void mock_port_exit_critical(portMUX_TYPE *mux) {
    mux->locked--;
    mock_critical_depth--;
}

mock_task_t mock_tasks[MOCK_MAX_TASKS];
int         mock_task_count;
int         mock_delay_calls;
uint64_t    mock_delay_total_ms;
int         mock_task_delete_calls;
void      (*mock_delay_hook)(TickType_t ticks);

static jmp_buf task_jmp;
static int     task_running;     /* inside mock_run_task() */
static int     task_delays;      /* delays inside the running task */
static int     task_max_delays;

BaseType_t xTaskCreate(TaskFunction_t fn, const char *name, uint32_t stack,
                       void *arg, UBaseType_t prio, TaskHandle_t *handle) {
    if (mock_task_count < MOCK_MAX_TASKS) {
        mock_task_t *t = &mock_tasks[mock_task_count++];
        t->fn = fn;
        snprintf(t->name, sizeof t->name, "%s", name ? name : "");
        t->stack = stack;
        t->prio = prio;
        t->arg = arg;
    }
    if (handle) *handle = (TaskHandle_t)(intptr_t)mock_task_count;
    return pdPASS;
}

void vTaskDelay(TickType_t ticks) {
    mock_delay_calls++;
    mock_delay_total_ms += ticks;
    if (mock_delay_hook) mock_delay_hook(ticks);
    if (task_running && ++task_delays >= task_max_delays) {
        longjmp(task_jmp, 1);
    }
}

void vTaskDelete(TaskHandle_t handle) {
    mock_task_delete_calls++;
    /* vTaskDelete(NULL) never returns in FreeRTOS: leave the task. */
    if (handle == NULL && task_running) {
        longjmp(task_jmp, 2);
    }
}

UBaseType_t uxTaskGetStackHighWaterMark(TaskHandle_t handle) { (void)handle; return 1000; }

const mock_task_t *mock_find_task(const char *name) {
    for (int i = 0; i < mock_task_count; i++)
        if (strcmp(mock_tasks[i].name, name) == 0) return &mock_tasks[i];
    return NULL;
}

int mock_run_task(TaskFunction_t fn, void *arg, int max_delays) {
    task_delays = 0;
    task_max_delays = max_delays > 0 ? max_delays : 1;
    task_running = 1;
    if (setjmp(task_jmp) == 0) {
        fn(arg);   /* the task returned by itself (it should never do it) */
    }
    task_running = 0;
    return task_delays;
}

/* Used by the micro-ROS mock to leave the executor loop. */
void mock_task_exit(void) {
    if (task_running) longjmp(task_jmp, 3);
}

/* ---- queue (length 1, enough for xQueueOverwrite) ---- */
struct mock_queue { size_t item_size; int full; unsigned char item[256]; };

QueueHandle_t xQueueCreate(UBaseType_t length, UBaseType_t item_size) {
    (void)length;
    if (item_size > 256) return NULL;
    QueueHandle_t q = calloc(1, sizeof *q);
    q->item_size = item_size;
    return q;
}

BaseType_t xQueueOverwrite(QueueHandle_t q, const void *item) {
    memcpy(q->item, item, q->item_size);
    q->full = 1;
    return pdPASS;
}

BaseType_t xQueueReceive(QueueHandle_t q, void *item, TickType_t ticks) {
    (void)ticks;
    if (!q->full) return pdFALSE;
    memcpy(item, q->item, q->item_size);
    q->full = 0;
    return pdTRUE;
}

void vQueueDelete(QueueHandle_t q) { free(q); }

/* ================= GPIO ================= */
int               mock_gpio_level[MOCK_GPIO_PINS];
int               mock_gpio_set_calls[MOCK_GPIO_PINS];
int               mock_gpio_mode[MOCK_GPIO_PINS];
int               mock_gpio_config_calls;
mock_gpio_event_t mock_gpio_log[MOCK_GPIO_LOG];
int               mock_gpio_log_len;

esp_err_t gpio_config(const gpio_config_t *conf) {
    mock_gpio_config_calls++;
    for (int p = 0; p < MOCK_GPIO_PINS; p++)
        if (conf->pin_bit_mask & (1ULL << p)) mock_gpio_mode[p] = conf->mode;
    return ESP_OK;
}

esp_err_t gpio_set_level(gpio_num_t pin, uint32_t level) {
    if (pin < 0 || pin >= MOCK_GPIO_PINS) return ESP_ERR_INVALID_ARG;
    mock_gpio_level[pin] = (int)level;
    mock_gpio_set_calls[pin]++;
    if (mock_gpio_log_len < MOCK_GPIO_LOG)
        mock_gpio_log[mock_gpio_log_len++] = (mock_gpio_event_t){ pin, (int)level };
    return ESP_OK;
}

/* ================= LEDC ================= */
int                   mock_ledc_timer_config_calls;
ledc_timer_config_t   mock_ledc_timer_conf;
int                   mock_ledc_channel_config_calls;
ledc_channel_config_t mock_ledc_channel_conf[LEDC_CHANNEL_MAX];
int                   mock_ledc_channel_configured[LEDC_CHANNEL_MAX];
uint32_t              mock_ledc_duty[LEDC_CHANNEL_MAX];
uint32_t              mock_ledc_out_duty[LEDC_CHANNEL_MAX];
int                   mock_ledc_set_calls[LEDC_CHANNEL_MAX];
int                   mock_ledc_update_calls[LEDC_CHANNEL_MAX];

esp_err_t ledc_timer_config(const ledc_timer_config_t *conf) {
    mock_ledc_timer_config_calls++;
    mock_ledc_timer_conf = *conf;
    return ESP_OK;
}

esp_err_t ledc_channel_config(const ledc_channel_config_t *conf) {
    mock_ledc_channel_config_calls++;
    if (conf->channel >= LEDC_CHANNEL_MAX) return ESP_ERR_INVALID_ARG;
    mock_ledc_channel_conf[conf->channel] = *conf;
    mock_ledc_channel_configured[conf->channel] = 1;
    mock_ledc_duty[conf->channel] = conf->duty;
    mock_ledc_out_duty[conf->channel] = conf->duty;
    return ESP_OK;
}

esp_err_t ledc_set_duty(ledc_mode_t mode, ledc_channel_t channel, uint32_t duty) {
    (void)mode;
    if (channel >= LEDC_CHANNEL_MAX) return ESP_ERR_INVALID_ARG;
    mock_ledc_duty[channel] = duty;
    mock_ledc_set_calls[channel]++;
    return ESP_OK;
}

esp_err_t ledc_update_duty(ledc_mode_t mode, ledc_channel_t channel) {
    (void)mode;
    if (channel >= LEDC_CHANNEL_MAX) return ESP_ERR_INVALID_ARG;
    mock_ledc_out_duty[channel] = mock_ledc_duty[channel];
    mock_ledc_update_calls[channel]++;
    return ESP_OK;
}

/* ================= I2C ================= */
int          mock_i2c_param_config_calls;
i2c_config_t mock_i2c_conf[I2C_NUM_MAX];
int          mock_i2c_driver_install_calls;
int          mock_i2c_installed_mode[I2C_NUM_MAX];
esp_err_t    mock_i2c_param_config_ret;
esp_err_t    mock_i2c_driver_install_ret;

uint8_t          mock_i2c_regs[128][256];
int              mock_i2c_transactions;
int              mock_i2c_fail_at;
int              mock_i2c_fail_from;
esp_err_t        mock_i2c_fail_err;
mock_i2c_event_t mock_i2c_log[MOCK_I2C_LOG];
int              mock_i2c_log_len;
void (*mock_i2c_write_hook)(uint8_t addr, uint8_t reg, const uint8_t *data, size_t len);
void (*mock_i2c_read_hook)(uint8_t addr, uint8_t reg, size_t len);

static void i2c_log(uint8_t addr, uint8_t reg, uint8_t value, bool is_read, size_t len) {
    if (mock_i2c_log_len < MOCK_I2C_LOG)
        mock_i2c_log[mock_i2c_log_len++] = (mock_i2c_event_t){ addr, reg, value, is_read, len };
}

/* Count the transaction and say if it must fail. */
static bool i2c_must_fail(void) {
    mock_i2c_transactions++;
    if (mock_i2c_fail_at && mock_i2c_transactions == mock_i2c_fail_at) return true;
    if (mock_i2c_fail_from && mock_i2c_transactions >= mock_i2c_fail_from) return true;
    return false;
}

esp_err_t i2c_param_config(i2c_port_t port, const i2c_config_t *conf) {
    mock_i2c_param_config_calls++;
    if (port >= 0 && port < I2C_NUM_MAX) mock_i2c_conf[port] = *conf;
    return mock_i2c_param_config_ret;
}

esp_err_t i2c_driver_install(i2c_port_t port, i2c_mode_t mode, size_t rx_len,
                             size_t tx_len, int intr_flags) {
    (void)rx_len; (void)tx_len; (void)intr_flags;
    mock_i2c_driver_install_calls++;
    if (port < 0 || port >= I2C_NUM_MAX) return ESP_ERR_INVALID_ARG;
    if (mock_i2c_driver_install_ret != ESP_OK) return mock_i2c_driver_install_ret;
    /* Like the real driver: a port can only be installed once. */
    if (mock_i2c_installed_mode[port] != -1) return ESP_FAIL;
    mock_i2c_installed_mode[port] = mode;
    return ESP_OK;
}

esp_err_t i2c_master_write_to_device(i2c_port_t port, uint8_t addr,
                                     const uint8_t *buf, size_t len, TickType_t ticks) {
    (void)port; (void)ticks;
    if (i2c_must_fail()) return mock_i2c_fail_err ? mock_i2c_fail_err : ESP_FAIL;
    if (len == 0 || addr >= 128) return ESP_ERR_INVALID_ARG;
    uint8_t reg = buf[0];
    for (size_t i = 1; i < len; i++) {
        mock_i2c_regs[addr][(uint8_t)(reg + i - 1)] = buf[i];
        i2c_log(addr, (uint8_t)(reg + i - 1), buf[i], false, len - 1);
    }
    if (len == 1) i2c_log(addr, reg, 0, false, 0);
    if (mock_i2c_write_hook) mock_i2c_write_hook(addr, reg, buf + 1, len - 1);
    return ESP_OK;
}

esp_err_t i2c_master_write_read_device(i2c_port_t port, uint8_t addr,
                                       const uint8_t *wbuf, size_t wlen,
                                       uint8_t *rbuf, size_t rlen, TickType_t ticks) {
    (void)port; (void)ticks;
    if (i2c_must_fail()) return mock_i2c_fail_err ? mock_i2c_fail_err : ESP_FAIL;
    if (wlen == 0 || addr >= 128) return ESP_ERR_INVALID_ARG;
    uint8_t reg = wbuf[0];
    if (mock_i2c_read_hook) mock_i2c_read_hook(addr, reg, rlen);
    for (size_t i = 0; i < rlen; i++) rbuf[i] = mock_i2c_regs[addr][(uint8_t)(reg + i)];
    i2c_log(addr, reg, 0, true, rlen);
    return ESP_OK;
}

void mock_i2c_set_be16(uint8_t addr, uint8_t reg, int16_t value) {
    mock_i2c_regs[addr][reg]     = (uint8_t)(((uint16_t)value) >> 8);
    mock_i2c_regs[addr][reg + 1] = (uint8_t)(((uint16_t)value) & 0xFF);
}

int mock_i2c_count_writes(uint8_t addr, uint8_t reg) {
    int n = 0;
    for (int i = 0; i < mock_i2c_log_len; i++)
        if (!mock_i2c_log[i].is_read && mock_i2c_log[i].addr == addr && mock_i2c_log[i].reg == reg) n++;
    return n;
}

int mock_i2c_last_write(uint8_t addr, uint8_t reg) {
    for (int i = mock_i2c_log_len - 1; i >= 0; i--)
        if (!mock_i2c_log[i].is_read && mock_i2c_log[i].addr == addr && mock_i2c_log[i].reg == reg
            && mock_i2c_log[i].len > 0)
            return mock_i2c_log[i].value;
    return -1;
}

/* ================= reset ================= */
void mock_reset_esp(void) {
    mock_esp_error_check_fails = 0;
    mock_esp_error_check_last = ESP_OK;
    mock_time_us = 0;
    mock_time_step_us = 0;
    mock_wifi_set_ps_calls = 0;
    mock_wifi_ps_mode = WIFI_PS_MIN_MODEM;
    mock_wifi_set_ps_ret = ESP_OK;
    mock_netif_init_calls = 0;
    mock_netif_init_ret = ESP_OK;

    mock_critical_depth = 0;
    mock_critical_enter_count = 0;
    memset(mock_tasks, 0, sizeof mock_tasks);
    mock_task_count = 0;
    mock_delay_calls = 0;
    mock_delay_total_ms = 0;
    mock_task_delete_calls = 0;
    mock_delay_hook = NULL;
    task_running = 0;

    for (int p = 0; p < MOCK_GPIO_PINS; p++) {
        mock_gpio_level[p] = -1;
        mock_gpio_set_calls[p] = 0;
        mock_gpio_mode[p] = -1;
    }
    mock_gpio_config_calls = 0;
    mock_gpio_log_len = 0;

    mock_ledc_timer_config_calls = 0;
    memset(&mock_ledc_timer_conf, 0, sizeof mock_ledc_timer_conf);
    mock_ledc_channel_config_calls = 0;
    memset(mock_ledc_channel_conf, 0, sizeof mock_ledc_channel_conf);
    memset(mock_ledc_channel_configured, 0, sizeof mock_ledc_channel_configured);
    memset(mock_ledc_duty, 0, sizeof mock_ledc_duty);
    memset(mock_ledc_out_duty, 0, sizeof mock_ledc_out_duty);
    memset(mock_ledc_set_calls, 0, sizeof mock_ledc_set_calls);
    memset(mock_ledc_update_calls, 0, sizeof mock_ledc_update_calls);

    mock_i2c_param_config_calls = 0;
    memset(mock_i2c_conf, 0, sizeof mock_i2c_conf);
    mock_i2c_driver_install_calls = 0;
    for (int i = 0; i < I2C_NUM_MAX; i++) mock_i2c_installed_mode[i] = -1;
    mock_i2c_param_config_ret = ESP_OK;
    mock_i2c_driver_install_ret = ESP_OK;
    memset(mock_i2c_regs, 0, sizeof mock_i2c_regs);
    mock_i2c_transactions = 0;
    mock_i2c_fail_at = 0;
    mock_i2c_fail_from = 0;
    mock_i2c_fail_err = ESP_FAIL;
    mock_i2c_log_len = 0;
    mock_i2c_write_hook = NULL;
    mock_i2c_read_hook = NULL;
}

/* The ROS mock is linked only by the tests that need it. */
__attribute__((weak)) void mock_reset_ros(void) {}

void mock_reset_all(void) {
    mock_reset_esp();
    mock_reset_ros();
}
