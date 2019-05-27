#ifndef _DEMODULATOR_H_
#define _DEMODULATOR_H_

#include "driver/rmt.h"

typedef struct
{
    gpio_num_t rmt_gpio;
    rmt_channel_t rmt_channel;
    uint8_t rmt_clk_div;
    size_t budrate_filter;
    size_t rx_buff_size;
    void (*begin_measure_callback)(void);
    void (*finish_measure_callback)(rmt_item32_t *items, size_t qty);
} demodulator_configuration_t;

void rmt_rx_task(void *pvParameter);
void print_rx_data(rmt_item32_t *item, size_t elements);
#endif