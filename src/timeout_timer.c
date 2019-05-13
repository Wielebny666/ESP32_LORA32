#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_log.h"
#include "timeout_timer.h"

static const char *TAG = "timeout_timer";

void TimerInit(TimerHandle_t *timer, const uint8_t index, const char *timer_name, void (*fn)(void *))
{
    ESP_LOGD(TAG, "%s %s", __FUNCTION__, timer_name);

    *timer = xTimerCreate(timer_name,               // Just a text name, not used by the kernel.
                          100 / portTICK_PERIOD_MS, // The timer period in ticks.
                          pdFALSE,                  // The timers will auto-reload themselves when they expire.
                          (void *)&index,           // Assign each timer a unique id equal to its array index.
                          fn                        // Each timer calls the same callback when it expires.
    );

    assert(*timer);
}

void TimerSetValue(TimerHandle_t *timer, uint32_t value)
{
#ifdef DEBUG
    ets_printf("%s %s %s value %d\n", TAG, __FUNCTION__, pcTimerGetTimerName(*timer), value);
#endif
    xTimerChangePeriod(*timer, pdMS_TO_TICKS(value), 5 / portTICK_PERIOD_MS); //    xTimerChangePeriod(*timer, value, 0); // /
}

void TimerStart(TimerHandle_t *timer)
{
#ifdef DEBUG
    ets_printf("%s %s %s\n", TAG, __FUNCTION__, pcTimerGetTimerName(*timer));
#endif
    xTimerStart(*timer, 5 / portTICK_PERIOD_MS);
}

void TimerStop(TimerHandle_t *timer)
{
#ifdef DEBUG
    ets_printf("%s %s %s\n", TAG, __FUNCTION__, pcTimerGetTimerName(*timer));
#endif
    xTimerStopFromISR(*timer, 5 / portTICK_PERIOD_MS);
}

// void TimerStop(TimerHandle_t *timer)
// {
//     ESP_LOGD(TAG, "%s", __FUNCTION__);
//     ESP_LOGD(TAG, "Stop Timer %p", timer);

//     xTimerStopFromISR (*timer, 0);
// }
