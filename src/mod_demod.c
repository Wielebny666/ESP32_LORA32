#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"

#include "driver/rmt.h"
#include "driver/gpio.h"

#include "hardware.h"

static const char *RMT_TX = "RMT Tx";
static const char *RMT_RX = "RMT Rx";

#define RMT_TX_CHANNEL RMT_CHANNEL_0
#define RMT_RX_CHANNEL RMT_CHANNEL_1
#define RMT_ITEM32_TIMEOUT_US 3000

#define RMT_CLK_DIV 80                                   /*!< RMT counter clock divider */
#define RMT_TICK_10_US (80000000 / RMT_CLK_DIV / 100000) /*!< RMT counter value for 10 us.(Source clock is APB clock) */
#define RMT_BIT_US 500
#define RMT_BIT_DURATION (RMT_BIT_US / 10 * RMT_TICK_10_US)

#define GPIO_OUTPUT_IO_0 (RMT_TX_GPIO_NUM)
#define GPIO_OUTPUT_PIN_SEL (1ULL << GPIO_OUTPUT_IO_0)
#define GPIO_INPUT_IO_0 (RMT_RX_GPIO_NUM)
#define GPIO_INPUT_PIN_SEL (1ULL << GPIO_INPUT_IO_0)

//Convert uint8_t type of data to rmt format data.
static void IRAM_ATTR u8_to_rmt(const void *src, rmt_item32_t *dest, size_t src_size,
                                size_t wanted_num, size_t *translated_size, size_t *item_num);
static void rmt_tx_int()
{
    // gpio_config_t tx_out = {
    //     .mode = GPIO_MODE_OUTPUT,
    //     .intr_type = GPIO_PIN_INTR_DISABLE,
    //     .pull_up_en = GPIO_PULLUP_DISABLE,
    //     .pull_down_en = GPIO_PULLDOWN_ENABLE,
    //     .pin_bit_mask = GPIO_OUTPUT_PIN_SEL};
    // ESP_ERROR_CHECK(gpio_config(&tx_out));

    rmt_config_t rmt_tx = {
        .rmt_mode = RMT_MODE_TX,
        .channel = RMT_TX_CHANNEL,
        .gpio_num = RMT_TX_GPIO_NUM,
        .clk_div = RMT_CLK_DIV,
        .mem_block_num = 7,
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

static void rmt_rx_init()
{
    gpio_config_t rx_in = {
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_PIN_INTR_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pin_bit_mask = GPIO_INPUT_PIN_SEL};
    ESP_ERROR_CHECK(gpio_config(&rx_in));

    rmt_config_t rmt_rx = {
        .rmt_mode = RMT_MODE_RX,
        .channel = RMT_RX_CHANNEL,
        .gpio_num = RMT_RX_GPIO_NUM,
        .clk_div = RMT_CLK_DIV,
        .mem_block_num = 1,
        .rx_config.filter_en = true,
        .rx_config.filter_ticks_thresh = 200,
        .rx_config.idle_threshold = RMT_ITEM32_TIMEOUT_US / 10 * (RMT_TICK_10_US)};

    ESP_ERROR_CHECK(rmt_config(&rmt_rx));
    ESP_ERROR_CHECK(rmt_driver_install(rmt_rx.channel, 5000, 0));
}

void printData(rmt_item32_t *item, size_t size)
{
    for (int i = 0; i < size / sizeof(rmt_item32_t); i++)
    {
        ESP_LOGD(RMT_RX, "[%d] level0: %d duration0: %d", i, (item + i)->level0, (item + i)->duration0);
        ESP_LOGD(RMT_RX, "[%d] level1: %d duration1: %d", i, (item + i)->level1, (item + i)->duration1);
    }
}

void rmt_rx_task(void *pvParameter)
{
    rmt_rx_init();
    RingbufHandle_t rb = NULL;
    //get RMT RX ringbuffer
    ESP_ERROR_CHECK(rmt_get_ringbuf_handle(RMT_RX_CHANNEL, &rb));

    assert(rb);
    ESP_ERROR_CHECK(rmt_rx_start(RMT_RX_CHANNEL, true));

    rmt_set_rx_filter(RMT_RX_CHANNEL, true, 100);

    while (rb)
    {
        size_t rx_size = 0;
        //try to receive data from ringbuffer.
        //RMT driver will push all the data it receives to its ringbuffer.
        //We just need to parse the value and return the spaces of ringbuffer.
        //rmt_item32_t* item = (rmt_item32_t*) (RMT_CHANNEL_MEM(rmt_rx.channel));
        rmt_item32_t *item = (rmt_item32_t *)xRingbufferReceive(rb, &rx_size, 500 / portTICK_PERIOD_MS);

        if (item == NULL)
        {
            ESP_LOGD(RMT_RX, "Timeout on recv!");
        }
        else if (rx_size == 0)
        {
            ESP_LOGD(RMT_RX, "End packet received.");
            vRingbufferReturnItem(rb, (void *)item);
            break;
        }
        else
        {
            ESP_LOGD(RMT_RX, "Received data in ringbuffer");
            printData(item, rx_size);
            vRingbufferReturnItem(rb, (void *)item);
        }
    }
    vTaskDelete(NULL);
}

void rmt_tx_task(void *pvParameter)
{
    rmt_tx_int();

    const static uint8_t sample[] = {0xcb, 0x55, 0x53, 0x55, 0x33, 0x55, 0x81, 0xAA, 0x81, 0xAA};

    while (1)
    {
        ESP_LOGD(RMT_TX, "Size: %d", sizeof(sample) / sizeof(sample[0]));

        ESP_ERROR_CHECK(rmt_write_sample(RMT_TX_CHANNEL, sample, sizeof(sample) / sizeof(sample[0]), true));
        ESP_LOGD(RMT_TX, "Sample transmission complete");
        vTaskDelay(500 / portTICK_PERIOD_MS);
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
    const rmt_item32_t bit0 = {{{RMT_BIT_DURATION / 2, 0, RMT_BIT_DURATION / 2, 0}}}; //Logical 0
    const rmt_item32_t bit1 = {{{RMT_BIT_DURATION / 2, 1, RMT_BIT_DURATION / 2, 1}}}; //Logical 1
    size_t size = 0;
    size_t num = 0;
    uint8_t *psrc = (uint8_t *)src;

    rmt_item32_t *pdest = dest;
    while (size < src_size && num < wanted_num)
    {
        for (int i = 7; i > -1; i--)
        {
            if (*psrc & (0x1 << i))
            {
                pdest->val = bit1.val;
            }
            else
            {
                pdest->val = bit0.val;
            }
            num++;
            pdest++;
        }
        size++;
        psrc++;
    }
    *translated_size = size;
    *item_num = num;

    //rmt_item32_t *pdest_ = dest;

    ESP_LOGD(RMT_TX, "Sample prepare complete");
    // for (int i = 0; i < num; i++)
    // {
    //     ESP_LOGD(RMT_TX, "[%d] level0: %d duration0: %d", i, (pdest_ + i)->level0, (pdest_ + i)->duration0);
    //     ESP_LOGD(RMT_TX, "[%d] level1: %d duration1: %d", i, (pdest_ + i)->level1, (pdest_ + i)->duration1);
    // }
}
