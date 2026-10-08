/* Mock of esp_err.h (host tests). */
#pragma once
#include <stddef.h>
typedef int esp_err_t;
#define ESP_OK                 0
#define ESP_FAIL              -1
#define ESP_ERR_NO_MEM         0x101
#define ESP_ERR_INVALID_ARG    0x102
#define ESP_ERR_INVALID_STATE  0x103
#define ESP_ERR_INVALID_SIZE   0x104
#define ESP_ERR_NOT_FOUND      0x105
#define ESP_ERR_TIMEOUT        0x107
#define ESP_ERR_INVALID_RESPONSE 0x108

const char *esp_err_to_name(esp_err_t code);

/* The real ESP_ERROR_CHECK aborts. Here it only records the failure, so the
 * tests can check that it happened (mock_esp_error_check_fails). */
void mock_esp_error_check_fail(esp_err_t err, const char *file, int line);
#define ESP_ERROR_CHECK(x) do {                                   \
        esp_err_t __mock_err = (x);                               \
        if (__mock_err != ESP_OK)                                 \
            mock_esp_error_check_fail(__mock_err, __FILE__, __LINE__); \
    } while (0)
