#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/ledc.h"
#include "esp_log.h"
#include "hardware.h"

#define LEDC_LS_TIMER LEDC_TIMER_1
#define LEDC_LS_MODE LEDC_LOW_SPEED_MODE

//#define LEDC_LS_CH0_GPIO GPIO_NUM_2
#define LEDC_LS_CH0_CHANNEL LEDC_CHANNEL_0

static const char *TAG = "led";

void led_init()
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
        .freq_hz = 10,                        // frequency of PWM signal
        .speed_mode = LEDC_LS_MODE,           // timer mode
        .timer_num = LEDC_LS_TIMER            // timer index
    };
    // Set configuration of timer1 for lo speed channels
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_LS_CH0_CHANNEL,
        .duty = 4000,
        .gpio_num = LEDC_LS_CH0_GPIO,
        .speed_mode = LEDC_LS_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_LS_TIMER};

    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void task_led(void *pvParameter)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    led_init();

    while (1)
    {
        vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}