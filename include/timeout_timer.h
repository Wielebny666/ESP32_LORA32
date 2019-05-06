#include <stdint.h>
#include "driver/timer.h"

typedef struct {
    int type;  // the type of timer's event
    timer_group_t timer_group;
    timer_idx_t timer_idx;
    uint64_t timer_counter_value;
} timer_event_t;

void TimerInit(timer_event_t *timer, timer_group_t timer_group, timer_idx_t timer_idx, void (*fn)(void *));
void TimerSetValue(timer_event_t *timer, uint32_t value);
void TimerStart(timer_event_t *timer);
void TimerStop(timer_event_t *timer);