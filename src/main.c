/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


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

#include "hardware.h"

static const char *TAG = "app_main";

QueueHandle_t display_queue;
display_t send_message;

void uart_init();

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

    display_queue = xQueueCreate(10, sizeof(display_t));
#ifdef TTGO
    xTaskCreate(task_display, "SSD1306", (2048), (void *)display_queue, 5, NULL);
#endif
    xTaskCreate(task_radio, "SX1276", (4096), (void *)display_queue, 10, NULL);

    //xTaskCreate(task_led, "LED", (2048), NULL, 1, NULL);
    //xTaskCreate(rmt_tx_task, "rmt_tx_task", 4096, NULL, 10, NULL);

    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
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
    // const uart_config_t uart_config = {
    //     .baud_rate = 18200,
    //     .data_bits = UART_DATA_8_BITS,
    //     .parity = UART_PARITY_DISABLE,
    //     .stop_bits = UART_STOP_BITS_1,
    //     .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};
    // uart_param_config(UART_NUM_2, &uart_config);
    // uart_set_pin(UART_NUM_2, PIN_NUM_TXD, PIN_NUM_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // // We won't use a buffer for sending data.
    // uart_driver_install(UART_NUM_2, 256, 256, 0, NULL, 0);
    // uart_set_line_inverse(UART_NUM_2, UART_LINE_INV_MASK);
    // uart_set_mode(UART_NUM_2, UART_MODE_UART);
}