/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/rmt.h"

#include "esp_log.h"
#include "esp_err.h"

//#include "mod_demod.h"
#include "display.h"
#include "radio.h"
#include "as3933.h"

#include "frequency_count.h"

#include "hardware.h"

static const char *TAG = "app_main";

#define GPIO_SIGNAL_INPUT GPIO_NUM_34
#define GPIO_RMT_GATE GPIO_NUM_4
#define GPIO_LED GPIO_NUM_25

// internal signals for GPIO constant levels
#define GPIO_CONSTANT_LOW 0x30
#define GPIO_CONSTANT_HIGH 0x38

#define PCNT_UNIT (0)
#define PCNT_CHANNEL (PCNT_CHANNEL_0)
#define RMT_CHANNEL (RMT_CHANNEL_0)
//#define RMT_MAX_BLOCKS (2) // allow up to 2 * 64 * (2 * 32767) RMT periods in window
//#define RMT_CLK_DIV       160   // results in 2us steps (80MHz / 160 = 0.5 MHz
#define RMT_CLK_DIV 20 // results in 0.25us steps (80MHz / 20 = 4 MHz
//#define RMT_CLK_DIV       1     // results in 25ns steps (80MHz / 2 / 1 = 40 MHz)

#define SAMPLE_PERIOD 1.0 // seconds

// The counter is signed 16-bit, so maximum positive value is 32767
// The filter is unsigned 10-bit, maximum value is 1023. Use full period of maximum frequency.
// For higher expected frequencies, the sample period and filter must be reduced.

// suitable up to 16,383.5 Hz
// #define WINDOW_DURATION 10.0  // seconds
// #define FILTER_LENGTH 1023  // APB @ 80MHz, limits to < 39,100 Hz

// suitable up to 163,835 Hz
#define WINDOW_DURATION 0.1 // seconds
#define FILTER_LENGTH 1     // APB @ 80MHz, limits to < 655,738 Hz

// suitable up to 1,638,350 Hz
//#define WINDOW_DURATION  0.01  // seconds
//#define FILTER_LENGTH 12  // APB @ 80MHz, limits to < 3,333,333 Hz

// suitable up to 16,383,500 Hz - no filter
//#define WINDOW_DURATION  0.001  // seconds
//#define FILTER_LENGTH 0  // APB @ 80MHz, limits to < 40 MHz

#if (FILTER_LENGTH > 1023)
#error "The filter is unsigned 10-bit, maximum value is 1023."
#endif

static void window_start_callback(void)
{
    ESP_LOGI(TAG, "Begin sampling");
    gpio_matrix_in(GPIO_SIGNAL_INPUT, SIG_IN_FUNC228_IDX, false);
}

static void frequency_callback(double hz)
{
    gpio_matrix_in(GPIO_CONSTANT_LOW, SIG_IN_FUNC228_IDX, false);
    ESP_LOGI(TAG, "Frequency %f Hz", hz);
}

static void config_led(void)
{
    gpio_pad_select_gpio(GPIO_LED);
    gpio_set_direction(GPIO_LED, GPIO_MODE_OUTPUT);

    // route incoming frequency signal to onboard LED when sampling
    gpio_matrix_out(GPIO_LED, SIG_IN_FUNC228_IDX, false, false);
}

void app_main()
{
    // ESP_LOGD(TAG, "val %f", (float)(WINDOW_DURATION));
    // ESP_LOGD(TAG, "val %f", (float)(2.0 * 32767.0));
    // ESP_LOGD(TAG, "val %f", (float)((2.0 * 32767.0) / (80000000.0 / RMT_CLK_DIV)));

    // ESP_LOGD(TAG, "val %f", (float)(WINDOW_DURATION / ((2.0 * 32767.0) / (80000000.0 / RMT_CLK_DIV))));
    // ESP_LOGD(TAG, "val %d", (uint8_t)ceil((WINDOW_DURATION / ((2.0 * 32767.0) / (80000000.0 / RMT_CLK_DIV))) / RMT_MEM_ITEM_NUM));

    // assert(((WINDOW_DURATION / (float)((2 * 32767) / 80000000 / RMT_CLK_DIV)) > (RMT_MAX_BLOCKS * 64 * 2 * 32767)));

    // while (1)
    // {
    // }

    config_led();
    window_start_callback();

    frequency_count_configuration_t *config = malloc(sizeof(*config));
    config->pcnt_gpio = GPIO_SIGNAL_INPUT;
    config->pcnt_unit = PCNT_UNIT;
    config->pcnt_channel = PCNT_CHANNEL;
    config->pcnt_filter_length = FILTER_LENGTH;
    config->rmt_gpio = GPIO_RMT_GATE;
    config->rmt_channel = RMT_CHANNEL;
    config->rmt_clk_div = RMT_CLK_DIV;
    config->sampling_period_seconds = SAMPLE_PERIOD;
    config->sampling_window_seconds = WINDOW_DURATION;
    config->window_start_callback = NULL;     // &window_start_callback;
    config->frequency_update_callback = NULL; // &frequency_callback;

    static TaskHandle_t task;
    xTaskCreate(&frequency_count_task_function, "frequency_count_task", 4096, config, 5, &task);

    as3933_spi_init();
    as3933_reset();
    uint8_t test5 = as3933_read(5);
    uint8_t test6 = as3933_read(6);
    ESP_LOGD(TAG, "%x %x", test5, test6);
    as3933_band_select(125000);
    as3933_set_channel(1, true);
    as3933_set_channel(2, true);
    as3933_set_channel(3, true);
    as3933_set_listening_mode(LM_SCANNING);
    as3933_set_patern_correlation(WK_FREQ_DET_ONLY);
    as3933_set_manchaster_decode(false);
    as3933_set_patern_correlation(false);
    as3933_set_freq_tolerance(TIGHT);

    as3933_set_comparator_hysteresis(BOTH_EDGE_40MV);
    as3933_set_gain_reduction(GAIN_REDUCTION_24DB);
    as3933_set_antenna_damper(RESISTOR_27KOM);
    as3933_enable_antenna_damper(true);

    as3933_set_capacity(1, 17);
    as3933_set_capacity(2, 31);
    as3933_set_capacity(3, 31);
    //ESP_LOGD(TAG, "cal status %d", as3933_rc_osc_self_calibrate());

    //as3933_route_clock_on_dat(true);
    //as3933_route_res_freq_on_dat(1, true);

    while (1)
    {
        for (uint8_t i = 1; i < 4; i++)
        {
            //as3933_set_channel(i, true);
            //vTaskDelay(1000 / portTICK_PERIOD_MS);
            uint8_t c1 = as3933_get_rssi(1);
            uint8_t c2 = as3933_get_rssi(2);
            uint8_t c3 = as3933_get_rssi(3);
            ESP_LOGD(TAG, "Channel 1 %d, Channel 2 %d, Channel 3 %d", c1, c2, c3);
            as3933_reset_rssi();
            as3933_clear_wake_up();
            //ESP_LOGD(TAG, "Channel %d", i);
            //as3933_route_res_freq_on_dat(0, false);
            //as3933_route_res_freq_on_dat(1, true);
            //vTaskDelay(100 / portTICK_PERIOD_MS);
            //xTaskNotify(task, 0, eNoAction);
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
    }

    QueueHandle_t display_queue = xQueueCreate(10, sizeof(display_t));
#ifdef TTGO
    xTaskCreatePinnedToCore(task_display, "SSD1306", 2048, (void *)display_queue, 5, NULL, 1);
    //xTaskCreate(task_display, "SSD1306", (2048), (void *)display_queue, 5, NULL);
#endif
    xTaskCreate(task_radio, "SX1276", (4096), (void *)display_queue, 10, NULL);

    //xTaskCreate(rmt_tx_task, "rmt_tx_task", 4096, NULL, 10, NULL);

    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    for (int i = 15; i >= 0; i--)
    {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
