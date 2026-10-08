/* Mock of queue.h (host tests). Simple queue of length 1. */
#pragma once
#include "freertos/FreeRTOS.h"
typedef struct mock_queue *QueueHandle_t;
QueueHandle_t xQueueCreate(UBaseType_t length, UBaseType_t item_size);
BaseType_t    xQueueOverwrite(QueueHandle_t q, const void *item);
BaseType_t    xQueueReceive(QueueHandle_t q, void *item, TickType_t ticks);
void          vQueueDelete(QueueHandle_t q);
