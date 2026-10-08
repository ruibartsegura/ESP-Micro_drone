/* Mock of driver/i2c.h (legacy driver, host tests).
 *
 * It simulates the I2C bus with a register map per device address:
 *   - a write of [reg, value...] stores the values from reg on.
 *   - a write-read of [reg] reads from reg on (auto increment).
 * A test can add a hook to simulate a real chip (for example the BMP180,
 * that changes its data registers after a conversion command) and can
 * make any transaction fail. */
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"   /* the real driver also includes FreeRTOS */
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

typedef int i2c_port_t;
#define I2C_NUM_0   0
#define I2C_NUM_1   1
#define I2C_NUM_MAX 2

typedef enum { I2C_MODE_SLAVE = 0, I2C_MODE_MASTER = 1, I2C_MODE_MAX } i2c_mode_t;
/* Copy of the real values, for the asserts: a project header could redefine
 * I2C_MODE_MASTER with a macro. */
#define MOCK_I2C_MODE_SLAVE  0
#define MOCK_I2C_MODE_MASTER 1

typedef struct {
    i2c_mode_t    mode;
    int           sda_io_num;
    int           scl_io_num;
    gpio_pullup_t sda_pullup_en;
    gpio_pullup_t scl_pullup_en;
    union {
        struct { uint32_t clk_speed; } master;
        struct { uint8_t addr_10bit_en; uint16_t slave_addr; } slave;
    };
    uint32_t clk_flags;
} i2c_config_t;

/* ---- Recorded configuration ---- */
extern int          mock_i2c_param_config_calls;
extern i2c_config_t mock_i2c_conf[I2C_NUM_MAX];
extern int          mock_i2c_driver_install_calls;
extern int          mock_i2c_installed_mode[I2C_NUM_MAX];
extern esp_err_t    mock_i2c_param_config_ret;
extern esp_err_t    mock_i2c_driver_install_ret;

/* ---- Fake devices ---- */
extern uint8_t mock_i2c_regs[128][256];      /* [7-bit address][register] */
extern int     mock_i2c_transactions;        /* write + write_read calls  */
extern int     mock_i2c_fail_at;             /* fail transaction number N (1..), 0 = never */
extern int     mock_i2c_fail_from;           /* fail every transaction >= N, 0 = never     */
extern esp_err_t mock_i2c_fail_err;

/* Last write per device (reg + data), useful to check commands. */
#define MOCK_I2C_LOG 1024
typedef struct { uint8_t addr; uint8_t reg; uint8_t value; bool is_read; size_t len; } mock_i2c_event_t;
extern mock_i2c_event_t mock_i2c_log[MOCK_I2C_LOG];
extern int              mock_i2c_log_len;

/* Called after a write is stored, to simulate the chip behaviour. */
extern void (*mock_i2c_write_hook)(uint8_t addr, uint8_t reg, const uint8_t *data, size_t len);
/* Called before a read is served (can update the registers). */
extern void (*mock_i2c_read_hook)(uint8_t addr, uint8_t reg, size_t len);

esp_err_t i2c_param_config(i2c_port_t port, const i2c_config_t *conf);
esp_err_t i2c_driver_install(i2c_port_t port, i2c_mode_t mode, size_t rx_len,
                             size_t tx_len, int intr_flags);
esp_err_t i2c_master_write_to_device(i2c_port_t port, uint8_t addr,
                                     const uint8_t *write_buffer, size_t write_size,
                                     TickType_t ticks);
esp_err_t i2c_master_write_read_device(i2c_port_t port, uint8_t addr,
                                       const uint8_t *write_buffer, size_t write_size,
                                       uint8_t *read_buffer, size_t read_size,
                                       TickType_t ticks);

/* Helpers for the tests. */
void mock_i2c_set_be16(uint8_t addr, uint8_t reg, int16_t value);
int  mock_i2c_count_writes(uint8_t addr, uint8_t reg);
int  mock_i2c_last_write(uint8_t addr, uint8_t reg);   /* -1 if never written */
