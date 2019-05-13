#include <stdio.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos/queue.h"

#include <u8g2.h>

#include "esp_log.h"
#include "esp_err.h"

#include "u8g2_esp32_hal.h"
#include "display.h"
#include "hardware.h"

static const char *TAG = "display";

void task_display(void *pvParameters)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    esp_log_level_set("u8g2_hal", ESP_LOG_NONE);

    QueueHandle_t display_queue = (QueueHandle_t *)pvParameters;

    display_t message;

    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
    u8g2_esp32_hal.sda = PIN_I2C_SDA;
    u8g2_esp32_hal.scl = PIN_I2C_SCL;
    u8g2_esp32_hal_init(u8g2_esp32_hal);

    u8g2_t u8g2; // a structure which will contain all the data for one display
    u8g2_Setup_ssd1306_i2c_128x32_univision_f(
        &u8g2,
        U8G2_R0,
        u8g2_esp32_i2c_byte_cb,
        u8g2_esp32_gpio_and_delay_cb); // init u8g2 structure
    u8x8_SetI2CAddress(&u8g2.u8x8, 0x78);

    u8g2_InitDisplay(&u8g2);     // send init sequence to the display, display is in sleep mode after this,
    u8g2_SetPowerSave(&u8g2, 0); // wake up display
    u8g2_ClearBuffer(&u8g2);

    while (1)
    {
        if (display_queue == NULL)
        {
            ESP_LOGD(TAG, "Display queue not exist!");
            continue;
        }

        while (xQueueReceive(display_queue, &message, 500 / portTICK_PERIOD_MS) == pdTRUE)
        {
            char rssi_string[12];
            sprintf(rssi_string, "RSSI= %4d dB", message.rssi_value);
            char snr_string[10];
            sprintf(snr_string, "Snr= %4d", message.snr_value);
            char status_string[10];
            sprintf(status_string, "%s", message.status);
            // char freq_string[10];
            // sprintf(freq_string, "Freq= %.1f MHz", message.freq_value);

            u8g2_ClearBuffer(&u8g2);

            u8g2_SetFont(&u8g2, u8g2_font_courR08_tr);
            u8g2_DrawStr(&u8g2, 0, 8, rssi_string);
            //u8g2_DrawStr(&u8g2, 0, 20, freq_string);
            u8g2_DrawStr(&u8g2, 0, 16+1, snr_string);
            u8g2_DrawStr(&u8g2, 0, 26, status_string);
            u8g2_SendBuffer(&u8g2);
        }
        vTaskDelay(500 / portTICK_PERIOD_MS); //wait for 500 ms
    }

    ESP_LOGD(TAG, "All done!");

    vTaskDelete(NULL);
}
