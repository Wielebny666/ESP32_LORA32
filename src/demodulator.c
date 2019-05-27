#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "esp_log.h"
#include "esp_err.h"

#include "driver/rmt.h"
#include "driver/gpio.h"

#include "demodulator.h"

static const char *TAG = "RMT Rx";

#define RMT_TX_CHANNEL RMT_CHANNEL_1
#define RMT_RX_CHANNEL RMT_CHANNEL_2
#define RMT_ITEM32_TIMEOUT_US 5000

#define RMT_CLK_DIV 8                                    /*!< RMT counter clock divider */
#define RMT_TICK_10_US (80000000 / RMT_CLK_DIV / 100000) /*!< RMT counter value for 10 us.(Source clock is APB clock) */
#define RMT_BIT_US 4
#define RMT_BIT_DURATION ((RMT_BIT_US * RMT_TICK_10_US) / 10)
//#define RMT_BIT_DURATION (RMT_BIT_US / 10 * RMT_TICK_10_US)

#define GPIO_OUTPUT_IO_0 (RMT_TX_GPIO_NUM)
#define GPIO_OUTPUT_PIN_GROUP (1ULL << GPIO_OUTPUT_IO_0)
#define GPIO_INPUT_IO_0 (RMT_RX_GPIO_NUM)
#define GPIO_INPUT_PIN_GROUP (1ULL << GPIO_INPUT_IO_0)

#if (RMT_BIT_DURATION == 0)
#error "RMT_BIT_DURATION value must be positive"
#endif

static void rmt_rx_init(uint8_t tx_gpio, rmt_channel_t channel, uint8_t clk_div, size_t buff_size, size_t filter_ticks_thresh)
{
    rmt_config_t rmt_rx = {
        .rmt_mode = RMT_MODE_RX,
        .gpio_num = tx_gpio,
        .channel = channel,
        .clk_div = clk_div,
        .mem_block_num = 4,
        .rx_config.filter_en = true,
        .rx_config.filter_ticks_thresh = 160, //filter_ticks_thresh,
        .rx_config.idle_threshold = 4096};    //RMT_ITEM32_TIMEOUT_US / 10 * (RMT_TICK_10_US)};

    ESP_ERROR_CHECK(rmt_config(&rmt_rx));
    ESP_ERROR_CHECK(rmt_driver_install(rmt_rx.channel, buff_size, 0));
}

//static void IRAM_ATTR finish_measure_callback(TimerHandle_t xTimer)
static void IRAM_ATTR finish_measure_callback(void *arg)
{
    assert(arg);
    rmt_channel_t channel = *(rmt_channel_t *)arg;
    ESP_LOGD(TAG, "Measure channel: %d - time out", (uint8_t)channel);
    //ESP_LOGD(TAG, "Measure timer: %s - time out ", pcTimerGetTimerName(xTimer));
    ESP_ERROR_CHECK(rmt_rx_stop(channel));
    gpio_matrix_in(GPIO_FUNC_IN_LOW, RMT_SIG_IN1_IDX, false);
    //gpio_matrix_in(0x30, SIG_IN_FUNC224_IDX, false);
}

void rmt_rx_task(void *pvParameter)
{
    assert(pvParameter);
    demodulator_configuration_t configuration = *(demodulator_configuration_t *)pvParameter;
    demodulator_configuration_t *task_inputs = &configuration;

    ESP_LOGD(TAG, "Channel: %d|Div: %d|Gpio: %d|BuffSize: %d|Baudrate_filter: %d",
             task_inputs->rmt_channel,
             task_inputs->rmt_clk_div,
             task_inputs->rmt_gpio,
             task_inputs->rx_buff_size,
             task_inputs->budrate_filter);

    size_t filter_ticks = 80000000 / task_inputs->rmt_clk_div / task_inputs->budrate_filter;
    ESP_LOGD(TAG, "Filter value: %d", filter_ticks);
    //assert(filter_ticks < 255);

    rmt_rx_init(task_inputs->rmt_gpio, task_inputs->rmt_channel, task_inputs->rmt_clk_div, task_inputs->rx_buff_size, filter_ticks);

    RingbufHandle_t rb;
    //get RMT RX ringbuffer
    ESP_ERROR_CHECK(rmt_get_ringbuf_handle(task_inputs->rmt_channel, &rb));

    const esp_timer_create_args_t measure_timeout_args = {
        .callback = finish_measure_callback,
        .dispatch_method = ESP_TIMER_TASK,
        /* argument specified here will be passed to timer callback function */
        .arg = (void *)&task_inputs->rmt_channel,
        .name = "measure_timeout"};
    esp_timer_handle_t measure_timeout_timer;
    ESP_ERROR_CHECK(esp_timer_create(&measure_timeout_args, &measure_timeout_timer));

    ESP_ERROR_CHECK(rmt_set_rx_filter(task_inputs->rmt_channel, true, 150));
    // TimerHandle_t RxTimeoutTimer = xTimerCreate("timer_name",                     // Just a text name, not used by the kernel.
    //                                             1000 / portTICK_PERIOD_MS,         // The timer period in ticks.
    //                                             pdFALSE,                          // The timers will auto-reload themselves when they expire.
    //                                             (void *)task_inputs->rmt_channel, // Assign each timer a unique id equal to its array index.
    //                                             finish_measure_callback                    // Each timer calls the same callback when it expires.
    // );

    // assert(RxTimeoutTimer);

    //ESP_LOGD(TAG, "Set Timeout Timer");
    //xTimerChangePeriod(RxTimeoutTimer, pdMS_TO_TICKS(10), 0 / portTICK_PERIOD_MS);
    // gpio_pad_pulldown(task_inputs->rmt_gpio);
    // gpio_pad_pullup(task_inputs->rmt_gpio);

    gpio_matrix_in(GPIO_FUNC_IN_LOW, RMT_SIG_IN1_IDX, false);

    while (xTaskNotifyWait(0, 0, NULL, portMAX_DELAY) == pdTRUE)
    {
        //xTimerStart(RxTimeoutTimer, 0 / portTICK_PERIOD_MS);
        ESP_ERROR_CHECK(esp_timer_start_once(measure_timeout_timer, 1000));
        ESP_ERROR_CHECK(rmt_rx_start(task_inputs->rmt_channel, true));
        if (task_inputs->begin_measure_callback != NULL)
        {
            task_inputs->begin_measure_callback();
        }

        size_t rx_size = 0;
        //try to receive data from ringbuffer.
        //RMT driver will push all the data it receives to its ringbuffer.
        //We just need to parse the value and return the spaces of ringbuffer.
        //rmt_item32_t* item = (rmt_item32_t*) (RMT_CHANNEL_MEM(rmt_rx.channel));
        rmt_item32_t *item = (rmt_item32_t *)xRingbufferReceive(rb, &rx_size, 1000 / portTICK_PERIOD_MS);

        if (item == NULL)
        {
            ESP_LOGD(TAG, "Timeout on recv!");
        }
        else if (rx_size == 0)
        {
            ESP_LOGD(TAG, "End packet received.");
            vRingbufferReturnItem(rb, (void *)item);
            //break;
        }
        else
        {
            ESP_LOGD(TAG, "Received data in ringbuffer");
            //ESP_ERROR_CHECK(esp_timer_stop(measure_timeout_timer));
            //xTimerStop(RxTimeoutTimer, 0 / portTICK_PERIOD_MS);

            if (task_inputs->finish_measure_callback != NULL)
            {
                task_inputs->finish_measure_callback(item, rx_size / sizeof(rmt_item32_t));
            }
            //print_rx_data(item, rx_size / sizeof(rmt_item32_t));
            vRingbufferReturnItem(rb, (void *)item);
        }

        ESP_ERROR_CHECK(rmt_rx_stop(task_inputs->rmt_channel));
    }
    vTaskDelete(NULL);
}

void print_rx_data(rmt_item32_t *item, size_t elements)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    for (int i = 0; i < elements; i++)
    {
        ESP_LOGD(TAG, "[%d] level0: %d duration0: %d", i, item[i].level0, item[i].duration0);
        ESP_LOGD(TAG, "[%d] level1: %d duration1: %d", i, item[i].level1, item[i].duration1);
    }
}
