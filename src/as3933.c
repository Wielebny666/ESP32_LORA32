#include <string.h>

#include "esp_log.h"
#include "esp_err.h"

#include "driver/spi_master.h"
#include "hardware.h"
#include "as3933.h"

static const char *TAG = "as3933";

static spi_device_handle_t as3933;

void as3933_spi_init()
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    spi_bus_config_t buscfg = {
        .miso_io_num = RFID_MISO,
        .mosi_io_num = RFID_MOSI,
        .sclk_io_num = RFID_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1};

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_MASTER_FREQ_10M / 5, //Clock out at 10 MHz
        .mode = 1,                         //SPI mode 1
        .spics_io_num = RFID_CS,           //CS pin
        .flags = SPI_DEVICE_POSITIVE_CS,
        .queue_size = 1,                   //We want to be able to queue 7 transactions at a time
        .command_bits = 8};

    //Initialize the SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, 0));
    //Attach the Device to the SPI bus
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg, &as3933));
}


void as3933_cmd(cmd_t cmd)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 0;
    t.cmd = cmd | 0xC0;
    //t.flags = SPI_TRANS_USE_TXDATA;

    ESP_ERROR_CHECK(spi_device_polling_transmit(as3933, &t));
}

void as3933_write(uint8_t addr, uint8_t data)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.cmd = addr & 0x3F;
    t.flags = SPI_TRANS_USE_TXDATA;
    t.tx_data[0] = data;

    ESP_ERROR_CHECK(spi_device_polling_transmit(as3933, &t));
}

uint8_t as3933_read(uint8_t addr)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.cmd = addr | 0x40;
    t.flags = SPI_TRANS_USE_RXDATA;

    ESP_ERROR_CHECK(spi_device_polling_transmit(as3933, &t));
    return t.rx_data[0];
}

void as3933_write_buffer(uint16_t addr, uint8_t *buffer, uint8_t size)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = size * 8;
    t.cmd = addr | 0x80;
    t.tx_buffer = buffer;

    ESP_ERROR_CHECK(spi_device_polling_transmit(as3933, &t));
}

void as3933_read_buffer(uint16_t addr, uint8_t *buffer, uint8_t size)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = size * 8;
    t.cmd = addr & 0x7F;
    t.rx_buffer = buffer;

    ESP_ERROR_CHECK(spi_device_polling_transmit(as3933, &t));
}

void as3933_reset()
{
    as3933_cmd(PRESET_DEFAULT);
}