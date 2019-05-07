#include "esp_err.h"
#include "esp_log.h"
#include "timeout_timer.h"

#define TIMER_DIVIDER 16 //  Hardware timer clock divider

static const char *TAG = "timeout_timer";

void Init (TimerHandle_t  *timer, const char* timer_name,  void (*fn)(void *){
            xTimers[ x ] = xTimerCreate(    "Timer",       // Just a text name, not used by the kernel.
                                        ( 100 * x ),   // The timer period in ticks.
                                        pdTRUE,        // The timers will auto-reload themselves when they expire.
                                        ( void * ) x,  // Assign each timer a unique id equal to its array index.
                                        vTimerCallback // Each timer calls the same callback when it expires.
                                    );
}

void TimerInit(timer_event_t *timer, timer_group_t timer_group, timer_idx_t timer_idx, void (*fn)(void *))
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    timer->timer_group = timer_group;
    timer->timer_idx = timer_idx;

    timer_config_t config = {
        .divider = TIMER_DIVIDER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en = TIMER_PAUSE,
        .alarm_en = TIMER_ALARM_EN,
        .intr_type = TIMER_INTR_LEVEL,
        .auto_reload = true};
    ESP_ERROR_CHECK(timer_init(timer_group, timer_idx, &config));
    ESP_ERROR_CHECK(timer_pause(timer_group, timer_idx));
    ESP_ERROR_CHECK(timer_isr_register(timer_group, timer_idx, fn,
                                       NULL, ESP_INTR_FLAG_IRAM, NULL));
}

void TimerSetValue(timer_event_t *timer, uint32_t value)
{
    ESP_ERROR_CHECK(timer_set_alarm_value(timer->timer_group, timer->timer_idx, value));
}

void TimerStart(timer_event_t *timer)
{
    ESP_ERROR_CHECK(timer_start(timer->timer_group, timer->timer_idx));
}

void TimerStop(timer_event_t *timer)
{
    ESP_ERROR_CHECK(timer_pause(timer->timer_group, timer->timer_idx));
}