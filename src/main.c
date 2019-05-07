/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// #include "freertos/timers.h"
// #include "timeout_timer.h"

#include "esp_log.h"
#include "esp_err.h"

#include "esp_system.h"
#include "esp_spi_flash.h"

#include "driver/uart.h"
#include "soc/uart_struct.h"

//#include "sdkconfig.h"
//#include "cc1101_async.h"
#include "mod_demod.h"
#include "display.h"
#include "radio.h"
#include "led.h"

#define PIN_NUM_TXD UART_NUM_2_TXD_DIRECT_GPIO_NUM
#define PIN_NUM_RXD UART_NUM_2_RXD_DIRECT_GPIO_NUM

#define GPIO_INPUT_IO_0 (PIN_NUM_RXD)
#define GPIO_INPUT_PIN_SEL (1ULL << GPIO_INPUT_IO_0)
#define GPIO_OUTPUT_IO_0 (PIN_NUM_TXD)
#define GPIO_OUTPUT_PIN_SEL (1ULL << GPIO_OUTPUT_IO_0)

static const char *TAG = "app_main";

display_t send_message;

void uart_init();

// static void SX1276OnTimeoutIrq(TimerHandle_t timer)
// {
//     ESP_LOGD(TAG, "%s", __FUNCTION__);
// }

// TimerHandle_t TxTimeoutTimerr = NULL;

// void task_timer(void *pvParameter){

//     TimerInit(&TxTimeoutTimerr, 0, "TxTimeoutTimer", SX1276OnTimeoutIrq);
//     TimerSetValue(&TxTimeoutTimerr, 1000);

//     TimerStart(&TxTimeoutTimerr);

//     while(1){

//     }
// }

// void app_main()
// {
//     xTaskCreate(task_timer, "test", (2048), NULL, 10, NULL);


//     while (1)
//     {
//     }
// }
// static void timer_callback(TimerHandle_t timer)
// {
//     volatile int *count;
//     count = (volatile int *)pvTimerGetTimerID(timer);
//     (*count)++;
//     printf("Callback timer %p count %p = %d\n", timer, count, *count);
// }

// void app_main()
// {
//     volatile int count = 0;
//     TimerHandle_t oneshot = xTimerCreate("oneshot", 100 / portTICK_PERIOD_MS, pdFALSE,
//                                          (void *)&count, timer_callback);
//     assert(oneshot);
//     xTimerIsTimerActive(oneshot);

//     xTimerStart(oneshot, 1);
//     vTaskDelay(2); /* give the timer task a chance to process the message */

//     xTimerIsTimerActive(oneshot);

//     vTaskDelay(250 / portTICK_PERIOD_MS); // 2.5 timer periods

//     xTimerIsTimerActive(oneshot);
//     while (1)
//     {
//     }
// }

void app_main()
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external\n");

    esp_log_level_set("u8g2_hal", ESP_LOG_NONE);

    display_queue = xQueueCreate(5, sizeof(display_t));

    xTaskCreate(task_radio, "SX1276", (4096), NULL, 10, NULL);
    xTaskCreate(task_display, "SSD1306", (2048), (void *)display_queue, 10, NULL);
    xTaskCreate(task_led, "LED", (2048), NULL, 10, NULL);
    //xTaskCreate(rmt_tx_task, "rmt_tx_task", 4096, NULL, 10, NULL);

    while (1)
    {
        if (display_queue == NULL)
        {
            ESP_LOGI(TAG, "Display queue not exist!");
            continue;
        }

        for (int8_t i = 20; i > -100; i--)
        {
            send_message.rssi_value = i;
            send_message.freq_value = 433.56;
            xQueueSend(display_queue, &send_message, (TickType_t)0);
            vTaskDelay(200 / portTICK_PERIOD_MS);
        }
    }

    //led_init();
    //uart_init();

    //xTaskCreate(cc1101_tx_async, "cc1101_tx_task", 2048, NULL, 10, NULL);
    //xTaskCreate(cc1101_rx_async, "cc1101_rx_task", 2048, NULL, 10, NULL);
    //vTaskDelay(2000 / portTICK_PERIOD_MS);
    //xTaskCreate(rmt_rx_task, "rmt_rx_task", 2048, NULL, 10, NULL);

    //const static uint8_t data[] = {0x00, 0x55, 0x55, 0xAA, 0xAA};

    while (1)
    {
        //const int txBytes = uart_write_bytes(UART_NUM_2, (const char *)data, 5);
        //const int txBytes = uart_write_bytes(UART_NUM_2, (const char *)test_str, strlen(test_str));
        //                                                         printf("Tx data %d\n", txBytes);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        //send_data(spi);
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

void uart_init()
{
    const uart_config_t uart_config = {
        .baud_rate = 18200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    uart_param_config(UART_NUM_2, &uart_config);
    uart_set_pin(UART_NUM_2, PIN_NUM_TXD, PIN_NUM_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // We won't use a buffer for sending data.
    uart_driver_install(UART_NUM_2, 256, 256, 0, NULL, 0);
    uart_set_line_inverse(UART_NUM_2, UART_LINE_INV_MASK);
    uart_set_mode(UART_NUM_2, UART_MODE_UART);
}