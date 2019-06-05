#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "hardware.h"

//#define LEDC_LS_CH0_GPIO GPIO_NUM_2
#define RESOLUTION LEDC_TIMER_8_BIT
#define DUTY_CYCLE 1 << (RESOLUTION - 1)

#define LEDC_SLOW_FADE_TIME (50)
#define LEDC_DUTY (10)

static const char *TAG = "led";

void led_init()
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE, // timer mode
        .duty_resolution = RESOLUTION,      // resolution of PWM duty
        .timer_num = LEDC_TIMER_0,          // timer index
        .freq_hz = 10                       // frequency of PWM signal
    };
    // Set configuration of timer0 for lo speed channels
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_CHANNEL_0,
        .duty = DUTY_CYCLE,
        .gpio_num = GPIO_NUM_2,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE};

    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    // route incoming frequency signal to onboard LED
    //gpio_matrix_in(GPIO_FREQ_SIGNAL, SIG_IN_FUNC228_IDX, false);
    //gpio_matrix_out(LEDC_LS_CH0_GPIO, SIG_IN_FUNC228_IDX, false, false);

    gpio_matrix_in(GPIO_NUM_2, SIG_IN_FUNC224_IDX, false);
    //gpio_matrix_out(GPIO_NUM_25, LEDC_HS_SIG_OUT0_IDX, false, false);
}

void task_led(void *pvParameter __attribute__((unused)))
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_HIGH_SPEED_MODE, // timer mode
        .duty_resolution = RESOLUTION,      // resolution of PWM duty
        .timer_num = LEDC_TIMER_0,          // timer index
        .freq_hz = 125000                   // frequency of PWM signal
    };
    // Set configuration of timer0 for lo speed channels
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .channel = LEDC_CHANNEL_0,
        .duty = DUTY_CYCLE,
        .gpio_num = LEDC_LS_CH0_GPIO,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE};

    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    //ledc_fade_func_install(0);

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
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_LOGD(TAG, "All done!");
    vTaskDelete(NULL);
}