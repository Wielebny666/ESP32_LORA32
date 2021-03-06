/*
 * MIT License
 *
 * Copyright (c) 2018 David Antliff
 * Copyright (c) 2018 Chris Morgan <chmorgan@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "driver/pcnt.h"
#include "driver/rmt.h"

#include "esp_system.h"
#include "esp_log.h"
#include "driver/pcnt.h"
#include "driver/gpio.h"

#include "frequency_count.h"

static const char *TAG = "frequency_counter";

static void init_rmt(uint8_t tx_gpio, rmt_channel_t channel, uint8_t clk_div);
static void init_pcnt(uint8_t pulse_gpio, uint8_t ctrl_gpio, pcnt_unit_t unit, pcnt_channel_t channel, uint16_t pcnt_filter_length);
static uint32_t create_rmt_window(rmt_item32_t *items, double sampling_window_seconds, double rmt_period);

void frequency_count_task_function(void *pvParameter)
{
    ESP_LOGD(TAG, "Core ID %d", xPortGetCoreID());
    assert(pvParameter);
    FrequencyCountConfig configuration = pvParameter;

    ESP_LOGD(TAG, "pcnt_gpio %d|pcnt_unit %d|pcnt_channel %d|rmt_gpio %d|rmt_clk_div %d|sampling_period_seconds %f|sampling_window_seconds %f|pcnt_filter_length %d",
             configuration->pcnt_gpio,
             configuration->pcnt_unit,
             configuration->pcnt_channel,
             configuration->rmt_gpio,
             configuration->rmt_clk_div,
             configuration->sampling_period_seconds,
             configuration->sampling_window_seconds,
             configuration->pcnt_filter_length);

    init_rmt(configuration->rmt_gpio, configuration->rmt_channel, configuration->rmt_clk_div);
    init_pcnt(configuration->pcnt_gpio, configuration->rmt_gpio, configuration->pcnt_unit, configuration->pcnt_channel, configuration->pcnt_filter_length);

    // assuming 80MHz APB clock
    const double rmt_period = (double)(configuration->rmt_clk_div) / 80000000.0;

    uint8_t req_blocks = (uint8_t)ceil((configuration->sampling_window_seconds / ((2.0 * 32767.0) / (80000000.0 / configuration->rmt_clk_div))) / RMT_MEM_ITEM_NUM);
    ESP_LOGD(TAG, "Req blocks %d", req_blocks);
    assert(req_blocks < 8);
    const size_t items_size = RMT_MEM_BLOCK_BYTE_NUM * req_blocks;
    rmt_item32_t *rmt_items = malloc(items_size);
    assert(rmt_items);
    memset(rmt_items, 0, items_size);
    uint32_t num_rmt_items = create_rmt_window(rmt_items, configuration->sampling_window_seconds, rmt_period);
    assert(num_rmt_items <= req_blocks * RMT_MEM_ITEM_NUM);

    TickType_t last_wake_time = xTaskGetTickCount();
    double frequency_hz;

    while (xTaskNotifyWait(0, 0, NULL, portMAX_DELAY) == pdTRUE)
    {
        last_wake_time = xTaskGetTickCount();

        // clear counter
        ESP_ERROR_CHECK(pcnt_counter_clear(configuration->pcnt_unit));

        // start sampling window
        ESP_ERROR_CHECK(rmt_write_items(configuration->rmt_channel, rmt_items, num_rmt_items, false));

        // call wndow-start callback if set
        if (configuration->window_start_callback)
        {
            (configuration->window_start_callback)();
        }

        // wait for window to finish
        ESP_ERROR_CHECK(rmt_wait_tx_done(configuration->rmt_channel, portMAX_DELAY));

        // read counter
        int16_t count = 0;
        ESP_ERROR_CHECK(pcnt_get_counter_value(configuration->pcnt_unit, &count));

        // TODO: check for overflow?

        frequency_hz = abs(count / 2.0 / configuration->sampling_window_seconds);

        // call the frequency update callback
        if (configuration->frequency_update_callback)
        {
            (configuration->frequency_update_callback)(frequency_hz);
        }

        int delay_time = configuration->sampling_period_seconds * 1000 / portTICK_PERIOD_MS;
        if (delay_time > 0)
        {
            vTaskDelayUntil(&last_wake_time, delay_time);
        }
    }

    free(rmt_items);
    FrequencyCountConfig_Destroy(pvParameter);
    //free(task_inputs); // TODO: avoid this if not dynamically allocated
    vTaskDelete(NULL);
}

FrequencyCountConfig FrequencyCountConfig_Create(void)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    FrequencyCountConfig me = malloc(sizeof(struct FrequencyCountConfig));
    assert(me);
    return me;
}

void FrequencyCountConfig_Destroy(FrequencyCountConfig const me)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    if (me == NULL)
    {
        ESP_LOGI(TAG, "Pointer not initialize");
    }
    free(me);
}

static void init_rmt(uint8_t tx_gpio, rmt_channel_t channel, uint8_t clk_div)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    rmt_config_t rmt_tx = {
        .rmt_mode = RMT_MODE_TX,
        .channel = channel,
        .gpio_num = tx_gpio,
        .mem_block_num = 1, // single block
        .clk_div = clk_div,
        .tx_config.loop_en = false,
        .tx_config.carrier_en = false,
        .tx_config.idle_level = RMT_IDLE_LEVEL_LOW,
        .tx_config.idle_output_en = true,
    };
    ESP_ERROR_CHECK(rmt_config(&rmt_tx));
    ESP_ERROR_CHECK(rmt_driver_install(rmt_tx.channel, 0, 0));
}

static void init_pcnt(uint8_t pulse_gpio, uint8_t ctrl_gpio, pcnt_unit_t unit, pcnt_channel_t channel, uint16_t pcnt_filter_length)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    // set up counter
    pcnt_config_t pcnt_config = {
        .pulse_gpio_num = pulse_gpio,
        .ctrl_gpio_num = ctrl_gpio,
        .lctrl_mode = PCNT_MODE_DISABLE,
        .hctrl_mode = PCNT_MODE_KEEP,
        .pos_mode = PCNT_COUNT_INC, // count both rising and falling edges
        .neg_mode = PCNT_COUNT_INC,
        .counter_h_lim = 0,
        .counter_l_lim = 0,
        .unit = unit,
        .channel = channel,
    };

    ESP_ERROR_CHECK(pcnt_unit_config(&pcnt_config));

    // set the GPIO back to high-impedance, as pcnt_unit_config sets it as pull-up
    ESP_ERROR_CHECK(gpio_set_pull_mode(pulse_gpio, GPIO_FLOATING));

    // enable counter filter - at 80MHz APB CLK, 1000 pulses is max 80,000 Hz, so ignore pulses less than 12.5 us.
    ESP_ERROR_CHECK(pcnt_set_filter_value(unit, pcnt_filter_length));
    ESP_ERROR_CHECK(pcnt_filter_enable(unit));
}

static uint32_t create_rmt_window(rmt_item32_t *items, double sampling_window_seconds, double rmt_period)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    uint32_t num_items = 0;

    // enable counter for exactly x seconds:
    uint32_t total_duration = (uint32_t)(sampling_window_seconds / rmt_period);
    ESP_LOGD(TAG, "total_duration %f seconds = %d * %g seconds", sampling_window_seconds, total_duration, rmt_period);

    uint16_t *p_items = (uint16_t *)items;
    // max duration per item is 2^15-1 = 32767
    while (total_duration > 0)
    {
        uint32_t duration = total_duration > 32767 ? 32767 : total_duration;
        p_items[num_items] = (1 << 15) | duration;
        total_duration -= duration;
        num_items++;
    }

    return (num_items + 1) / 2;
}