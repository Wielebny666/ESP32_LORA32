#include "esp_err.h"
#include "timeout_timer.h"

void TimerInit(timer_event_t *timer, timer_group_t timer_group, timer_idx_t timer_idx, void (*fn)(void *))
{
    timer->timer_group = timer_group;
    timer->timer_idx = timer_idx;
    timer_isr_register(timer_group, timer_idx, fn,
                       NULL, ESP_INTR_FLAG_IRAM, NULL);
}

void TimerSetValue(timer_event_t *timer, uint32_t value)
{
    ESP_ERROR_CHECK(timer_set_alarm_value(timer->timer_group, timer->timer_idx, value));
}

void timTimerStarter_go(timer_event_t *timer)
{
    ESP_ERROR_CHECK(timer_start(timer->timer_group, timer->timer_idx));
}

void TimerStop(timer_event_t *timer)
{
    ESP_ERROR_CHECK(timer_pause(timer->timer_group, timer->timer_idx));
}