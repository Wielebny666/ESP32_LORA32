#ifndef _RFID_H_
#define _RFID_H_

typedef struct
{
    bool manchester;
    bool pattern_16bit;
    int16_t wake_up_pattern;
    uint8_t bitrate;
    uint32_t frequency;
    void (*wake_up_callback)(uint8_t rssi1, uint8_t rssi2, uint8_t rssi3);
    void (*calibrate_callback)(bool result);
} rfid_configuration_t;

void rfid_task(void *pvParameter);

#endif