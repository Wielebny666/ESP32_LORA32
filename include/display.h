#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include <stdint.h>
#include <stddef.h>

typedef struct
{
    int16_t rssi_value;
    int8_t snr_value;
    float freq_value;
    struct
    {
        uint8_t rfid_rssi_1;
        uint8_t rfid_rssi_2;
        uint8_t rfid_rssi_3;
    } rfid;
    char *status;
} display_t;

void task_display(void *ignore);

#endif