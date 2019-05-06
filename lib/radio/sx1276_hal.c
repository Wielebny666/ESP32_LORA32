#include <stdint.h>
#include <string.h>

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#include "sx1276_hal.h"
#include "../include/hardware.h"

#define GPIO_INPUT_PIN_SEL (1ULL << RADIO_DIO_0)
#define GPIO_OUTPUT_PIN_SEL (1ULL << RADIO_RESET)

static const char *TAG = "sx1276_hal";

spi_device_handle_t sx127x;

void sx1276_init_io(void)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    spi_bus_config_t buscfg = {
        .miso_io_num = RADIO_MISO,
        .mosi_io_num = RADIO_MOSI,
        .sclk_io_num = RADIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1};

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000, //Clock out at 10 MHz
        .mode = 0,                         //SPI mode 0
        .spics_io_num = RADIO_NSS,         //CS pin
        .queue_size = 1,                   //We want to be able to queue 7 transactions at a time
        .command_bits = 8};

    //Initialize the SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, 1));
    //Attach the Device to the SPI bus
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg, &sx127x));

    gpio_config_t sx1267_diox = {
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_PIN_INTR_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pin_bit_mask = GPIO_INPUT_PIN_SEL};
    ESP_ERROR_CHECK(gpio_config(&sx1267_diox));
}

void sx1276_set_reset(uint8_t state)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    gpio_pad_select_gpio(RADIO_RESET);
    ESP_ERROR_CHECK(gpio_set_direction(RADIO_RESET, GPIO_MODE_OUTPUT));

    if (state == RADIO_RESET_ON)
    {
        // Set RESET pin to 0
        ESP_ERROR_CHECK(gpio_set_level(RADIO_RESET, 0));
    }
    else
    {
        ESP_ERROR_CHECK(gpio_set_level(RADIO_RESET, 1));
    }
}

void sx1276_write(uint8_t addr, uint8_t data)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.cmd = addr | 0x80;
    t.flags = SPI_TRANS_USE_TXDATA;
    t.tx_data[0] = data;

    ESP_ERROR_CHECK(spi_device_polling_transmit(sx127x, &t));
}

void sx1276_read(uint8_t addr, uint8_t *data)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.cmd = addr & 0x7F;
    t.flags = SPI_TRANS_USE_RXDATA;

    ESP_ERROR_CHECK(spi_device_polling_transmit(sx127x, &t));
    *data = t.rx_data[0];
}

void sx1276_write_buffer(uint8_t addr, uint8_t *buffer, uint8_t size)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    ESP_LOGD(TAG, "Addr %x, size %x", addr, size);

    // for (uint8_t i = 0; i < size; i++)
    //     ESP_LOGD(TAG, "[%x], value %x", i + addr, *(buffer + i));

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = size * 8;
    t.cmd = addr | 0x80;
    t.tx_buffer = buffer;

    ESP_ERROR_CHECK(spi_device_polling_transmit(sx127x, &t));
}

void sx1276_read_buffer(uint8_t addr, uint8_t *buffer, uint8_t size)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    ESP_LOGD(TAG, "Addr %x, size %x", addr, size);

    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = size * 8;
    t.cmd = addr & 0x7F;
    t.rx_buffer = buffer;

    ESP_ERROR_CHECK(spi_device_polling_transmit(sx127x, &t));

    for (uint8_t i = 0; i < size; i++)
        ESP_LOGD(TAG, "[%x], value %x", i + addr, *(buffer + i));
}

void sx1276_write_fifo(uint8_t *buffer, uint8_t size)
{
    sx1276_write_buffer(0, buffer, size);
}

void sx1276_read_fifo(uint8_t *buffer, uint8_t size)
{
    sx1276_read_buffer(0, buffer, size);
}

uint8_t sx1276_read_dio0(void)
{
    return gpio_get_level(RADIO_DIO_0);
}

uint8_t sx1276_read_dio1(void)
{
    return 0;
}

uint8_t sx1276_read_dio2(void)
{
    return 0;
}

uint8_t sx1276_read_dio3(void)
{
    return 1;
}

uint8_t sx1276_read_dio4(void)
{
    return 0;
}
uint8_t sx1276_read_dio5(void)
{
    return 0;
}
