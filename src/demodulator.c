#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/ringbuf.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_clk.h"

#include "driver/rmt.h"
#include "driver/gpio.h"

#include "demodulator.h"

static const char *TAG = "rmt_rx";

demodulator_configuration_t *task_inputs;
esp_timer_handle_t measure_break_timer;

//static void IRAM_ATTR break_measure_callback(void *arg);

static void rmt_rx_init(uint8_t tx_gpio, rmt_channel_t channel, uint8_t clk_div, size_t buff_items, uint8_t filter_ticks_thresh, uint16_t measure_iddle_ticks)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    rmt_config_t rmt_rx = {
        .rmt_mode = RMT_MODE_RX,
        .gpio_num = tx_gpio,
        .channel = channel,
        .clk_div = clk_div,
        .mem_block_num = 8,
        .rx_config.filter_en = false,
        .rx_config.filter_ticks_thresh = filter_ticks_thresh,
        .rx_config.idle_threshold = UINT16_MAX}; //measure_iddle_ticks};

    ESP_ERROR_CHECK(rmt_config(&rmt_rx));
    ESP_ERROR_CHECK(rmt_driver_install(rmt_rx.channel, (sizeof(rmt_item32_t) * buff_items), 0));
    ESP_ERROR_CHECK(gpio_set_pull_mode(tx_gpio, GPIO_PULLDOWN_ONLY));
}

void rmt_rx_task(void *pvParameter)
{
    assert(pvParameter);
    demodulator_configuration_t configuration = *(demodulator_configuration_t *)pvParameter;
    task_inputs = &configuration;

    ESP_LOGD(TAG, "Channel %d|Div %d|Gpio %d|BuffSize %d|Baudrate_filter %d|Min items count %d|Measure iddle time %dus",
             task_inputs->rmt_channel,
             task_inputs->rmt_clk_div,
             task_inputs->rmt_gpio_num,
             task_inputs->rx_buff_items,
             task_inputs->budrate_filter,
             task_inputs->min_items_count,
             task_inputs->measure_idle_threshold_us);

    size_t measure_iddle_ticks = (esp_clk_apb_freq() / task_inputs->rmt_clk_div / 1000000) * task_inputs->measure_idle_threshold_us;
    size_t filter_ticks_thresh = esp_clk_apb_freq() / task_inputs->rmt_clk_div / task_inputs->budrate_filter;
    ESP_LOGD(TAG, "measure_iddle_ticks: %d", measure_iddle_ticks);
    ESP_LOGD(TAG, "filter_ticks_thresh: %d", filter_ticks_thresh);
    assert(measure_iddle_ticks < UINT16_MAX);
    //assert(filter_ticks_thresh < UINT8_MAX);

    rmt_rx_init(task_inputs->rmt_gpio_num, task_inputs->rmt_channel, task_inputs->rmt_clk_div, task_inputs->rx_buff_items, filter_ticks_thresh, measure_iddle_ticks);

    RingbufHandle_t rb;
    // //get RMT RX ringbuffer
    ESP_ERROR_CHECK(rmt_get_ringbuf_handle(task_inputs->rmt_channel, &rb));

    // const esp_timer_create_args_t measure_break_timer_args = {
    //     .callback = break_measure_callback,
    //     .dispatch_method = ESP_TIMER_TASK,
    //     /* argument specified here will be passed to timer callback function */
    //     .arg = (void *)&task_inputs->rmt_channel,
    //     .name = "measure_timeout"};
    // ESP_ERROR_CHECK(esp_timer_create(&measure_break_timer_args, &measure_break_timer));
    gpio_matrix_in(GPIO_FUNC_IN_LOW, RMT_SIG_IN0_IDX, false);

    SemaphoreHandle_t start_measure_sem;
    start_measure_sem = xSemaphoreCreateMutex();

    while (xTaskNotifyWait(0, 0, NULL, portMAX_DELAY) == pdTRUE)
    {
        xSemaphoreTake(start_measure_sem, portMAX_DELAY);
        //ESP_ERROR_CHECK(esp_timer_start_once(measure_break_timer, task_inputs->break_measure_time_us));
        ESP_ERROR_CHECK(rmt_rx_start(task_inputs->rmt_channel, true));
        gpio_matrix_in(task_inputs->rmt_gpio_num, RMT_SIG_IN0_IDX, false);

        if (task_inputs->begin_measure_callback != NULL)
        {
            task_inputs->begin_measure_callback();
        }

        size_t rx_size = 0;
        //try to receive data from ringbuffer.
        //RMT driver will push all the data it receives to its ringbuffer.
        //We just need to parse the value and return the spaces of ringbuffer.
        //rmt_item32_t* item = (rmt_item32_t*) (RMT_CHANNEL_MEM(rmt_rx.channel));
        rmt_item32_t *item = (rmt_item32_t *)xRingbufferReceive(rb, (size_t *)&rx_size, portMAX_DELAY); // 500 / portTICK_PERIOD_MS); //

        if (item == NULL)
        {
            ESP_LOGD(TAG, "Timeout on recv!");
            gpio_matrix_in(GPIO_FUNC_IN_LOW, RMT_SIG_IN0_IDX, false);
            if (task_inputs->timeout_measure_callback != NULL)
            {
                task_inputs->timeout_measure_callback();
            }
        }
        else if (rx_size == 0)
        {
            ESP_LOGD(TAG, "End packet received.");
            gpio_matrix_in(GPIO_FUNC_IN_LOW, RMT_SIG_IN0_IDX, false);
            if (task_inputs->finish_measure_callback != NULL)
            {
                task_inputs->finish_measure_callback();
            }
            vRingbufferReturnItem(rb, (void *)item);
        }
        else
        {
            ESP_LOGD(TAG, "Received data in ringbuffer");
            gpio_matrix_in(GPIO_FUNC_IN_LOW, RMT_SIG_IN0_IDX, false);
            if ((rx_size / sizeof(rmt_item32_t)) < task_inputs->min_items_count)
            {
                if (task_inputs->error_measure_callback != NULL)
                {
                    task_inputs->error_measure_callback();
                }
            }
            else if (task_inputs->pending_measure_callback != NULL)
            {
                task_inputs->pending_measure_callback(item, rx_size / sizeof(rmt_item32_t));
            }
            vRingbufferReturnItem(rb, (void *)item);
        }
        xSemaphoreGive(start_measure_sem);
    }
    ESP_LOGD(TAG, "Delete task");
    vSemaphoreDelete(start_measure_sem);
    vTaskDelete(NULL);
}

// static void IRAM_ATTR break_measure_callback(void *arg)
// {
//     assert(arg);
//     rmt_channel_t channel = *(rmt_channel_t *)arg;
//     //ESP_ERROR_CHECK(rmt_rx_stop(channel));
//     gpio_matrix_in(GPIO_FUNC_IN_LOW, RMT_SIG_IN0_IDX, false);
//     ESP_LOGD(TAG, "Break measure on channel: %d", (uint8_t)channel);
//     //gpio_matrix_in(0x30, SIG_IN_FUNC224_IDX, false);
// }

void print_rx_data(void *item, size_t elements)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    rmt_item32_t *items = item;
    for (int i = 0; i < elements; i++)
    {
        ESP_LOGD(TAG, "[%d] level0: %d duration0: %d", i, items[i].level0, items[i].duration0);
        ESP_LOGD(TAG, "[%d] level1: %d duration1: %d", i, items[i].level1, items[i].duration1);
    }
}
