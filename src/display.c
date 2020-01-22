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
static char freq_string[14];

void task_display(void *pvParameters)
{
    ESP_LOGI(TAG, "%s", __FUNCTION__);
    esp_log_level_set("u8g2_hal", ESP_LOG_NONE);

    assert(pvParameters);
    QueueHandle_t display_queue = (QueueHandle_t *)pvParameters;

    DisplayStatus message = malloc(sizeof(struct DisplayStatus));
    if (message == NULL)
    {
        ESP_LOGI(TAG, "[message] pointer not initialize");
        vTaskDelete(NULL);
        return;
    }

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

    u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,
    vTaskDelay(50 / portTICK_PERIOD_MS);
    u8g2_SetPowerSave(&u8g2, 0); // wake up display
    vTaskDelay(50 / portTICK_PERIOD_MS);
    u8g2_ClearBuffer(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_courR08_tr);
    u8g2_DrawStr(&u8g2, 0, 8, "LCD init...");
    u8g2_SendBuffer(&u8g2);

    while (1)
    {
        portBASE_TYPE ret = xQueueReceive(display_queue, message, 1000 / portTICK_PERIOD_MS);
        if (ret == pdTRUE)
        {
            ESP_LOGI(TAG, "xQueueReceive f= %.1f MHz", message->freq_value);
            //char rssi_string[12];
            // sprintf(rssi_string, "RSSI= %4d dB", message->rssi_value);
            // char snr_string[10];
            // sprintf(snr_string, "Snr= %4d", message->snr_value);
            // char status_string[10];
            // sprintf(status_string, "%s", message->status);
            // char rfid_string[20];
            // sprintf(rfid_string, "R1= %2d R2= %2d R3= %2d", message->rfid.rssi_1, message->rfid.rssi_2, message->rfid.rssi_3);

            sprintf(freq_string, "f= %.1f MHz", message->freq_value);

            u8g2_ClearBuffer(&u8g2);

            u8g2_SetFont(&u8g2, u8g2_font_courR08_tr);
            // u8g2_DrawStr(&u8g2, 0, 8, rssi_string);
            u8g2_DrawStr(&u8g2, 0, 20, freq_string);
            // u8g2_DrawStr(&u8g2, 0, 16 + 1, snr_string);
            // //u8g2_DrawStr(&u8g2, 0, 26, status_string);
            // u8g2_DrawStr(&u8g2, 0, 26, rfid_string);
            u8g2_SendBuffer(&u8g2);
        }
        else
        {
            ESP_LOGI(TAG, "xQueueReceive Timeout");
        }
    }
    ESP_LOGD(TAG, "All done!");
    vTaskDelay(5000 / portTICK_PERIOD_MS); //wait for 500 ms
    vTaskDelete(NULL);
}

DisplayStatus DisplayStatus_Create(void)
{
    DisplayStatus me = malloc(sizeof(struct DisplayStatus));
    if (me == NULL)
    {
        ESP_LOGI(TAG, "Pointer not initialize");
        abort();
        return NULL;
    }
    return me;
}

void DisplayStatus_Destroy(DisplayStatus const me)
{
    if (me == NULL)
    {
        ESP_LOGI(TAG, "Pointer not initialize");
    }
    free(me);
}