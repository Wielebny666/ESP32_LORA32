#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "timeout_timer.h"

#define TIMER_DIVIDER 16 //  Hardware timer clock divider

static const char *TAG = "timeout_timer";

void TimerInit(TimerHandle_t *timer, const uint8_t index, const char *timer_name, void (*fn)(void *))
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    ESP_LOGD(TAG, "Init Timer %s", timer_name);

    ESP_LOGD(TAG, "Init Timer %p", timer);
    ESP_LOGD(TAG, "Init Timer %p", *timer);

    *timer = xTimerCreate(timer_name,               // Just a text name, not used by the kernel.
                          100 / portTICK_PERIOD_MS, // The timer period in ticks.
                          pdFALSE,                  // The timers will auto-reload themselves when they expire.
                          (void *)&index,           // Assign each timer a unique id equal to its array index.
                          fn                        // Each timer calls the same callback when it expires.
    );

    assert(*timer);
    ESP_LOGD(TAG, "Init Timer %p", timer);
    ESP_LOGD(TAG, "Init Timer %p", *timer);
}

// void Init(timer_event_t *timer, timer_group_t timer_group, timer_idx_t timer_idx, void (*fn)(void *))
// {
//     ESP_LOGD(TAG, "%s", __FUNCTION__);

//     timer->timer_group = timer_group;
//     timer->timer_idx = timer_idx;

//     timer_config_t config = {
//         .divider = TIMER_DIVIDER,
//         .counter_dir = TIMER_COUNT_UP,
//         .counter_en = TIMER_PAUSE,
//         .alarm_en = TIMER_ALARM_EN,
//         .intr_type = TIMER_INTR_LEVEL,
//         .auto_reload = true};
//     ESP_ERROR_CHECK(timer_init(timer_group, timer_idx, &config));
//     ESP_ERROR_CHECK(timer_pause(timer_group, timer_idx));
//     ESP_ERROR_CHECK(timer_isr_register(timer_group, timer_idx, fn,
//                                        NULL, ESP_INTR_FLAG_IRAM, NULL));
// }

void TimerSetValue(TimerHandle_t *timer, uint32_t value)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    ESP_LOGD(TAG, "Set Timer %p Value %d", timer, value);

    xTimerChangePeriod(*timer, pdMS_TO_TICKS(value), 0); // /
    //ESP_ERROR_CHECK(timer_set_alarm_value(timer->timer_group, timer->timer_idx, value));
}

void TimerStart(TimerHandle_t *timer)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    ESP_LOGD(TAG, "Start Timer %p", timer);

    xTimerStart(*timer, 0);
}

void TimerStop(TimerHandle_t *timer)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    ESP_LOGD(TAG, "Stop Timer %p", timer);

    xTimerStop(*timer, 0);
}