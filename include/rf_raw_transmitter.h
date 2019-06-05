#ifndef __RF_RAW_TRANSMITTER_H__
#define __RF_RAW_TRANSMITTER_H__

typedef struct
{
    struct
    {
        double frequencyInMHz;

    } radio_cfg;
void (*end_of_transmission_callback)(void);
} raw_transmitter_t;

void rf_raw_tx_task(void *pvParameter);

#endif