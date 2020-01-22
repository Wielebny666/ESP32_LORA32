#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include <stdint.h>
#include <stddef.h>

typedef struct DisplayStatus *DisplayStatus;

struct __attribute__((__packed__, aligned(1))) DisplayStatus
{
    int16_t rssi_value;
    int8_t snr_value;
    float freq_value;
    struct
    {
        uint8_t rssi_1;
        uint8_t rssi_2;
        uint8_t rssi_3;
    } rfid;
    char *status;
};

DisplayStatus DisplayStatus_Create(void);
void DisplayStatus_Destroy(DisplayStatus const me);
void task_display(void *pvParameters);

#endif