#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "modulator.h"

static const char *RMT_TX = "RMT Tx";

static uint32_t rmt_bit_duration = 32767;

//Convert uint8_t type of data to rmt format data.
static void IRAM_ATTR u8_to_rmt(const void *src, rmt_item32_t *dest, size_t src_size,
                                size_t wanted_num, size_t *translated_size, size_t *item_num);

void printTxData(rmt_item32_t *item, size_t size);

static void rmt_tx_int(uint8_t tx_gpio, rmt_channel_t channel, uint8_t clk_div)
{
    rmt_config_t rmt_tx = {
        .rmt_mode = RMT_MODE_TX,
        .channel = channel,
        .gpio_num = tx_gpio,
        .clk_div = clk_div,
        .mem_block_num = 2,
        .tx_config.carrier_en = false,
        .tx_config.carrier_freq_hz = 50000,
        .tx_config.carrier_duty_percent = 50,
        .tx_config.carrier_level = 1,
        .tx_config.loop_en = 0,
        .tx_config.idle_output_en = 0,
    };

    ESP_ERROR_CHECK(rmt_config(&rmt_tx));
    ESP_ERROR_CHECK(rmt_driver_install(rmt_tx.channel, 0, 0));
    ESP_ERROR_CHECK(rmt_translator_init(rmt_tx.channel, u8_to_rmt));
}

const static uint8_t sample[] = {0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55};

void rmt_tx_task(void *pvParameter)
{
    modulator_configuration_t configuration;
    assert(pvParameter);
    configuration = *(modulator_configuration_t *)pvParameter;
    modulator_configuration_t *task_inputs = &configuration;

    ESP_LOGD(RMT_TX, "Channel: %d|Div: %d|Gpio: %d|Baudrate: %d",
             task_inputs->rmt_channel,
             task_inputs->rmt_clk_div,
             task_inputs->rmt_gpio,
             task_inputs->baudrate);

    rmt_tx_int(task_inputs->rmt_gpio, task_inputs->rmt_channel, task_inputs->rmt_clk_div);

    uint32_t rmt_tick_10_us = 80000000 / task_inputs->rmt_clk_div / 100000;
    float rmt_bit_us = 1000000 / task_inputs->baudrate;
    rmt_bit_duration = (rmt_bit_us * rmt_tick_10_us) / 10;

    while (1)
    {
        ESP_LOGD(RMT_TX, "Transmission begin");
        ESP_ERROR_CHECK(rmt_write_sample(task_inputs->rmt_channel, sample, sizeof(sample) / sizeof(sample[0]), false));
        ESP_LOGD(RMT_TX, "Transmission complete");
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

static void IRAM_ATTR u8_to_rmt(const void *src, rmt_item32_t *dest, size_t src_size,
                                size_t wanted_num, size_t *translated_size, size_t *item_num)
{
    if (src == NULL || dest == NULL)
    {
        *translated_size = 0;
        *item_num = 0;
        return;
    }

    size_t size = 0;
    size_t num = 0;
    uint8_t *psrc = (uint8_t *)src;
    uint16_t *pdest = (uint16_t *)dest;

    while (size < src_size && num < wanted_num)
    {
        for (int i = 7; i > -1; i--)
        {
            uint32_t bit_duration = rmt_bit_duration;
            while (bit_duration > 0)
            {
                uint32_t duration = bit_duration > 32767 ? 32767 : bit_duration;
                pdest[num] = (((*psrc >> i) & 1) << 15) | duration;
                bit_duration -= duration;
                num++;
            }
        }
        psrc++;
        size++;
    }
    *translated_size = size;
    *item_num = num;
    //printData(dest, *item_num);
    ESP_LOGD(RMT_TX, "Data prepare complete");
}

void printTxData(rmt_item32_t *item, size_t elements)
{
    for (int i = 0; i < elements / 2; i++)
    {
        ESP_LOGD(RMT_TX, "[%d] level0: %d duration0: %d", i, (item + i)->level0, (item + i)->duration0);
        ESP_LOGD(RMT_TX, "[%d] level1: %d duration1: %d", i, (item + i)->level1, (item + i)->duration1);
    }
}