
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include "driver/rmt.h"
#include "esp_log.h"

#include "as3933.h"
#include "demodulator.h"
#include "rfid.h"

#include "hardware.h"

static const char *TAG = "rfid";

typedef enum
{
    RX,
    RX_PENDING,
    RX_FINISH,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    CALLIBRATE,
    IDDLE
} rfid_states_t;

static TaskHandle_t rmt_rx_task_handler;
rfid_configuration_t *task_inputs;

volatile rfid_states_t rfid_state = IDDLE;

static void on_begin();
static void on_finish();
static void on_pending(rmt_item32_t *items, size_t qty);
static void on_timeout();
static void on_error();
static void IRAM_ATTR wake_up_isr_handler(void *arg);

void rfid_init(uint8_t freq, uint8_t bitrate, bool manchaster)
{
    as3933_spi_init();
    as3933_reset();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    as3933_clear_wake_up();
    vTaskDelay(100 / portTICK_PERIOD_MS);
    uint8_t test5 = as3933_read(5);
    uint8_t test6 = as3933_read(6);
    ESP_LOGD(TAG, "%x %x", test5, test6);

    as3933_set_listening_mode(LM_STANDARD);
    as3933_band_select(freq);
    as3933_set_freq_tolerance(TIGHT);
    as3933_set_gain_reduction(GAIN_REDUCTION_12DB);
    as3933_set_antenna_damper(RESISTOR_3KOM);
    as3933_set_off_timer(OFF_1MS);
    as3933_enable_antenna_damper(false);

    as3933_set_manchaster_decode(manchaster);
    as3933_set_patern_correlation(WK_FREQ_DET_ONLY);

    as3933_set_capacity(1, 5);
    as3933_set_capacity(2, 15);
    as3933_set_capacity(3, 17);

    as3933_set_channel(1, true);
    as3933_set_channel(2, true);
    as3933_set_channel(3, true);
}

void rfid_io_init(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .pin_bit_mask = (1ULL << RFID_WAKE_UP),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = 1};
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    //install gpio isr service
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT));
    //hook isr handler for specific gpio pin
    ESP_ERROR_CHECK(gpio_isr_handler_add(RFID_WAKE_UP, wake_up_isr_handler, (void *)RFID_WAKE_UP));
}

static void IRAM_ATTR wake_up_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t)arg;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (rmt_rx_task_handler != NULL)
        xTaskNotifyFromISR(rmt_rx_task_handler, 0, eNoAction, &xHigherPriorityTaskWoken);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    if (task_inputs->wake_up_callback != NULL)
    {
        task_inputs->wake_up_callback(as3933_get_rssi(1), as3933_get_rssi(2), as3933_get_rssi(3));
    }
    as3933_clear_wake_up();
}

void rfid_task(void *pvParameter)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    assert(pvParameter);
    rfid_configuration_t configuration = *(rfid_configuration_t *)pvParameter;
    task_inputs = &configuration;

    rfid_init(task_inputs->frequency, task_inputs->bitrate, task_inputs->manchester);
    rfid_io_init();

    demodulator_configuration_t *demod_config = malloc(sizeof(*demod_config));
    demod_config->rmt_gpio = RFID_DATA;
    demod_config->ctrl_gpio_num = RFID_WAKE_UP;
    demod_config->rmt_clk_div = 1;
    demod_config->rmt_channel = RMT_CHANNEL_0;
    demod_config->rx_buff_items = 8 * 64 * 20;
    demod_config->budrate_filter = 120000;
    demod_config->min_items_count = 10;
    demod_config->end_measure_type = TIME_WINDOW;
    demod_config->break_measure_time_us = 5000;
    demod_config->measure_idle_threshold_us = 80;
    demod_config->begin_measure_callback = &on_begin;
    demod_config->pending_measure_callback = &on_pending;
    demod_config->finish_measure_callback = &on_finish;
    demod_config->timeout_measure_callback = &on_timeout;
    demod_config->error_measure_callback = &on_error;

    xTaskCreate(rmt_rx_task, "rmt_rx_task", 1024 * 5, demod_config, 40, &rmt_rx_task_handler);
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    //start
    start_measure();

    while (1)
    {
        switch (rfid_state)
        {
        case RX:
            ESP_LOGD(TAG, "RX");
            rfid_state = IDDLE;
            break;
        case RX_PENDING:
            ESP_LOGD(TAG, "RX_PENDING");
            start_measure();
            rfid_state = IDDLE;
            break;
        case RX_FINISH:
            ESP_LOGD(TAG, "RX_FINISH");
            start_measure();
            xTaskNotify(rmt_rx_task_handler, 0, eNoAction);
            rfid_state = IDDLE;
            //xTaskNotify(rmt_rx_task_handler, 0, eNoAction);
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            ESP_LOGD(TAG, "RX_ERROR");
            start_measure();
            rfid_state = IDDLE;
            //xTaskNotify(rmt_rx_task_handler, 0, eNoAction);
            break;
        case TX:
            break;
        case CALLIBRATE:
            break;
        case IDDLE:
        default:
            ESP_LOGD(TAG, "IDDLE");
            vTaskDelay(10 / portTICK_PERIOD_MS);
            break;
        }
    }
}

static void on_begin()
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    as3933_route_res_freq_on_dat(1, true);
    rfid_state = RX;
}

static void on_pending(rmt_item32_t *items, size_t qty)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    ESP_LOGI(TAG, "Items qty %d", qty);
    //print_rx_data(items, qty);
    rfid_state = RX_PENDING;
}

static void on_finish()
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    as3933_route_res_freq_on_dat(1, false);
    rfid_state = RX_FINISH;
}

static void on_timeout()
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    stop_measure();
    rfid_state = RX_TIMEOUT;
}

static void on_error()
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    stop_measure();
    rfid_state = RX_ERROR;
}
