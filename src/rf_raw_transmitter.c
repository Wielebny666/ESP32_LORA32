#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "cc1101.h"

static void rf_raw_tx_init()
{
    cc1101_spi_init();
    cc1101_reset();
    cc1101_init_config();
    cc1101_read_config();
    cc1101_print_config();
}

void rf_raw_tx_task(void *pvParameter)
{

    rf_raw_tx_init();

    while (1)
    {
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}