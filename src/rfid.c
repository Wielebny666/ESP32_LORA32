#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "driver/rmt.h"
#include "esp_log.h"

#include "as3933.h"
#include "demodulator.h"
#include "rfid.h"

#define GPIO_SIGNAL_INPUT GPIO_NUM_34

static const char *TAG = "rfid";

typedef enum
{
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    CALLIBRATE,
    IDDLE
} rfid_states_t;

volatile rfid_states_t rfid_state = IDDLE;

static void begin_sampling();
static void finish_sampling(rmt_item32_t *items, size_t qty);

void rfid_task(void *pvParameter __attribute__((unused)))
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    //assert(pvParameter);

    demodulator_configuration_t *demod_config = malloc(sizeof(*demod_config));
    demod_config->rmt_gpio = GPIO_SIGNAL_INPUT;
    demod_config->rmt_clk_div = 2;
    demod_config->rmt_channel = RMT_CHANNEL_0;
    demod_config->rx_buff_size = 8 * 64 * 4;
    demod_config->budrate_filter = 120000;
    demod_config->break_measure_time_us = 200000;
    demod_config->measure_idle_threshold_us = 80;
    demod_config->begin_measure_callback = &begin_sampling;
    demod_config->finish_measure_callback = &finish_sampling;

    as3933_spi_init();
    as3933_reset();
    uint8_t test5 = as3933_read(5);
    uint8_t test6 = as3933_read(6);
    ESP_LOGD(TAG, "%x %x", test5, test6);
    as3933_band_select(125000);
    as3933_set_channel(1, true);
    as3933_set_channel(2, true);
    as3933_set_channel(3, true);
    as3933_set_listening_mode(LM_STANDARD);
    as3933_set_patern_correlation(WK_FREQ_DET_ONLY);
    as3933_set_manchaster_decode(false);
    as3933_set_patern_correlation(false);
    as3933_set_freq_tolerance(TIGHT);

    as3933_set_comparator_hysteresis(BOTH_EDGE_40MV);
    as3933_set_gain_reduction(GAIN_REDUCTION_20DB);
    as3933_set_antenna_damper(RESISTOR_27KOM);
    as3933_enable_antenna_damper(false);

    as3933_set_capacity(1, 5);
    as3933_set_capacity(2, 15);
    as3933_set_capacity(3, 17);

    as3933_route_res_freq_on_dat(0, false);
    as3933_route_res_freq_on_dat(1, true);
    vTaskDelay(200 / portTICK_PERIOD_MS);

    static TaskHandle_t rmt_rx_task_handler;
    xTaskCreate(rmt_rx_task, "rmt_rx_task", 1024 * 5, demod_config, 20, &rmt_rx_task_handler);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    xTaskNotify(rmt_rx_task_handler, 0, eNoAction);

    while (1)
    {
        switch (rfid_state)
        {
        case RX:
            ESP_LOGD(TAG, "RX");
            xTaskNotify(rmt_rx_task_handler, 0, eNoAction);
            rfid_state = IDDLE;
            break;
        case RX_TIMEOUT:
            break;
        case RX_ERROR:
            break;
        case TX:
            break;
        case CALLIBRATE:
            break;
        case IDDLE:
        default:
            ESP_LOGD(TAG, "IDDLE");
            vTaskDelay(100 / portTICK_PERIOD_MS);
            break;
        }
    }
}

static void begin_sampling()
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    //gpio_matrix_in(GPIO_SIGNAL_INPUT, SIG_IN_FUNC224_IDX, false);
}

static void finish_sampling(rmt_item32_t *items, size_t qty)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    ESP_LOGI(TAG, "Items qty %d", qty);
    //gpio_matrix_in(GPIO_FUNC_IN_LOW, RMT_SIG_IN0_IDX, false);
    print_rx_data(items, qty);
    rfid_state = RX;
}