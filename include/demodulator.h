#ifndef _DEMODULATOR_H_
#define _DEMODULATOR_H_

#include "driver/rmt.h"

typedef enum
{
    IDDLE_TIME,
    TIME_WINDOW
} end_measure_type_t;

typedef struct
{
    gpio_num_t rmt_gpio;
    gpio_num_t ctrl_gpio_num;
    rmt_channel_t rmt_channel;
    uint8_t rmt_clk_div;
    size_t budrate_filter;
    size_t rx_buff_items;
    size_t min_items_count;
    end_measure_type_t end_measure_type;
    uint16_t measure_idle_threshold_us;
    uint64_t break_measure_time_us;
    void (*begin_measure_callback)(void);
    void (*pending_measure_callback)(rmt_item32_t *items, size_t qty);
    void (*finish_measure_callback)();
    void (*timeout_measure_callback)(void);
    void (*error_measure_callback)(void);
} demodulator_configuration_t;

void start_measure(void);
void stop_measure(void);
void rmt_rx_task(void *pvParameter);
void print_rx_data(rmt_item32_t *item, size_t elements);
#endif