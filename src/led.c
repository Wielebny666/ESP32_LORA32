#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/ledc.h"
#include "esp_log.h"
#include "hardware.h"

#define LEDC_LS_TIMER LEDC_TIMER_0
#define LEDC_LS_MODE LEDC_LOW_SPEED_MODE

//#define LEDC_LS_CH0_GPIO GPIO_NUM_2
#define LEDC_LS_CH0_CHANNEL LEDC_CHANNEL_0

#define LEDC_SLOW_FADE_TIME (50)
#define LEDC_DUTY (10)

static const char *TAG = "led";

void led_init()
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LS_MODE,          // timer mode
        .duty_resolution = LEDC_TIMER_4_BIT, // resolution of PWM duty
        .timer_num = LEDC_LS_TIMER,          // timer index
        .freq_hz = 50                        // frequency of PWM signal
    };
    // Set configuration of timer1 for lo speed channels
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_LS_CH0_CHANNEL,
        .duty = 0,
        .gpio_num = LEDC_LS_CH0_GPIO,
        .speed_mode = LEDC_LS_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_LS_TIMER,
        .intr_type = LEDC_INTR_DISABLE};

    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

void task_led(void *pvParameter __attribute__((unused)))
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LS_MODE,           // timer mode
        .duty_resolution = LEDC_TIMER_12_BIT, // resolution of PWM duty
        .timer_num = LEDC_LS_TIMER,           // timer index
        .freq_hz = 50                         // frequency of PWM signal
    };
    // Set configuration of timer1 for lo speed channels
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_LS_CH0_CHANNEL,
        .duty = 0,
        .gpio_num = LEDC_LS_CH0_GPIO,
        .speed_mode = LEDC_LS_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_LS_TIMER,
        .intr_type = LEDC_INTR_DISABLE};

    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    ledc_fade_func_install(0);

    while (xTaskNotifyWait(0, 0, NULL, portMAX_DELAY) == pdTRUE)
    {
        ESP_ERROR_CHECK(ledc_set_fade_with_time(ledc_channel.speed_mode, ledc_channel.channel, LEDC_DUTY, LEDC_SLOW_FADE_TIME));
        ESP_ERROR_CHECK(ledc_fade_start(ledc_channel.speed_mode, ledc_channel.channel, LEDC_FADE_NO_WAIT));
        vTaskDelay(LEDC_SLOW_FADE_TIME / portTICK_PERIOD_MS);

        ESP_ERROR_CHECK(ledc_set_fade_with_time(ledc_channel.speed_mode, ledc_channel.channel, 0, LEDC_SLOW_FADE_TIME));
        ESP_ERROR_CHECK(ledc_fade_start(ledc_channel.speed_mode, ledc_channel.channel, LEDC_FADE_NO_WAIT));
        //vTaskDelay(LEDC_SLOW_FADE_TIME / portTICK_PERIOD_MS);

        // ESP_LOGD(TAG, "Up");
        // vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
    ESP_LOGD(TAG, "All done!");
    vTaskDelete(NULL);
}