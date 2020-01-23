#include <stdio.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_err.h"

#ifdef SSD1306
#include "display.h"
#endif

#include "led.h"
#include "frequency_count.h"

static const char *TAG = "app_main";
QueueHandle_t displayQueue;

void app_main()
{
    DisplayStatus messageStatus;
    FrequencyCountConfig frequencyCountConfig;
    ESP_LOGI(TAG, "DisplayStatus %d", sizeof(struct DisplayStatus));
    ESP_LOGI(TAG, "FrequencyCountConfig %d", sizeof(struct FrequencyCountConfig));

    messageStatus = DisplayStatus_Create();
    frequencyCountConfig = FrequencyCountConfig_Create();

#ifdef TTGO
    if ((displayQueue = xQueueCreate(5, sizeof(struct DisplayStatus))) == NULL)
    {
        ESP_LOGI(TAG, "Display queue not exist!");
        return;
    }

    xTaskCreatePinnedToCore(task_display, "SSD1306", 1024 * 4, (void *)displayQueue, 5, NULL, 1);
#endif

    while (1)
    {
        if (xQueueSend(displayQueue, messageStatus, (portTickType)0) == errQUEUE_FULL)
        {
            ESP_LOGI("Queue", "Queue is full..");
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
        messageStatus->freq_value++;
        messageStatus->rfid.rssi_1++;
        messageStatus->rfid.rssi_2++;
        messageStatus->rfid.rssi_3++;
        ESP_LOGI(TAG, "f= %.1f MHz", messageStatus->freq_value);
    }
}