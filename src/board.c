#include "esp_log.h"

#include "hardware.h"
#include "board.h"

static const char *TAG = "board";

void SpiInit()
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    spi_bus_config_t buscfg = {
        .miso_io_num = RADIO_MISO,
        .mosi_io_num = RADIO_MOSI,
        .sclk_io_num = RADIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1};

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_MASTER_FREQ_10M, //Clock out at 10 MHz
        .mode = 0,                         //SPI mode 0
        .spics_io_num = RADIO_NSS,         //CS pin
        .queue_size = 1,                   //We want to be able to queue 7 transactions at a time
        .command_bits = 8};

    //Initialize the SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &buscfg, 0));
    //Attach the Device to the SPI bus
    ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &devcfg, &SX1276.Spi));

    SX1276IoInit();
}