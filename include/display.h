#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include <stdint.h>
#include <stddef.h>

typedef struct
{
    int16_t rssi_value;
    int8_t snr_value;
    float freq_value;
    char *status;
} display_t;

void task_display(void *ignore);

#endif