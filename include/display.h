#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include <stdint.h>
#include <stddef.h>

#include "freertos/queue.h"

typedef struct
{
    int8_t rssi_value;
    float freq_value;
    char *modulation_type;
} display_t;

QueueHandle_t display_queue;

void task_display(void *ignore);

#endif