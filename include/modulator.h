#ifndef _MODULATOR_H_
#define _MODULATOR_H_

#include "driver/rmt.h"

typedef struct
{
    uint32_t baudrate;
    gpio_num_t rmt_gpio;
    rmt_channel_t rmt_channel;
    uint8_t rmt_clk_div;
    void (*end_of_transmission_callback)(void);

} modulator_configuration_t;

void rmt_tx_task(void *pvParameter);

#endif