#include <stdint.h>
#include "freertos/timers.h"
//#include "driver/timer.h"

// typedef struct {
//     int type;  // the type of timer's event
//     timer_group_t timer_group;
//     timer_idx_t timer_idx;
//     uint64_t timer_counter_value;
// } timer_event_t;

void TimerInit(TimerHandle_t *timer, const uint8_t index, const char *timer_name, void (*fn)(void *));
void TimerSetValue(TimerHandle_t *timer, uint32_t value);
void TimerStart(TimerHandle_t *timer);
void TimerStop(TimerHandle_t *timer);