/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_err.h"

//#include "mod_demod.h"
#include "display.h"
#include "radio.h"
#include "as3933.h"

#include "hardware.h"

static const char *TAG = "app_main";

void app_main()
{

    as3933_spi_init();
    as3933_reset();
    as3933_band_select(125000);
    as3933_band_select(50000);
    while (1)
    {
        // uint8_t test5 = as3933_read(5);
        // uint8_t test6 = as3933_read(6);
        // ESP_LOGD(TAG, "%x %x", test5, test6);
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
