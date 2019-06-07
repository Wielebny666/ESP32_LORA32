#ifndef _DEMODULATOR_H_
#define _DEMODULATOR_H_

typedef struct
{
    uint8_t rmt_gpio_num;
    uint8_t ctrl_gpio_num;
    uint8_t rmt_channel;
    uint8_t rmt_clk_div;
    size_t budrate_filter;
    size_t rx_buff_items;
    size_t min_items_count;
    size_t measure_idle_threshold_us;
    void (*begin_measure_callback)(void);
    void (*pending_measure_callback)(rmt_item32_t *items, size_t qty);
    void (*finish_measure_callback)();
    void (*timeout_measure_callback)(void);
    void (*error_measure_callback)(void);
} demodulator_configuration_t;

void rmt_rx_task(void *pvParameter);
void print_rx_data(void *item, size_t elements);
#endif