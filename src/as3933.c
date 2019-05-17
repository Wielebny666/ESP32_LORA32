#include <string.h>

#include "esp_log.h"
#include "esp_err.h"

#include "hardware.h"
#include "as3933.h"

as3933_init_cmd_t as3933_config_register[] = {
    // AS3933 default settings for approx. 9m LF range
    {R0, 0x6E},
    {R1, 0x2A}, // AGC_T-LIM = 0 Had improved stability (RSSI value constant), but was causing lost LF. Changed back
    {R2, 0x20},
    {R3, 0x3F},
    {R4, 0x30}, // reduce Off time.
    {R5, 0x69},
    {R6, 0x96},
    {R7, 0x3F},
    {R8, 0x00},
    {R9, 0x00},
    {R16, 0x00},
    {R17, 0x00},
    {R18, 0x00},
    {R19, 0x00}};

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
        .clock_speed_hz = SPI_MASTER_FREQ_10M / 2, //Clock out at 10 MHz
        .mode = 1,                                 //SPI mode 1
        .spics_io_num = RFID_CS,                   //CS pin
        .flags = SPI_DEVICE_POSITIVE_CS,
        .queue_size = 1, //We want to be able to queue 7 transactions at a time
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
    t.cmd = cmd | DIRECT_COMMAND;
    //t.flags = SPI_TRANS_USE_TXDATA;

    ESP_ERROR_CHECK(spi_device_polling_transmit(as3933, &t));
}

void as3933_write(uint8_t addr, uint8_t data)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.cmd = addr & WRITE;
    t.flags = SPI_TRANS_USE_TXDATA;
    t.tx_data[0] = data;

    ESP_ERROR_CHECK(spi_device_polling_transmit(as3933, &t));
}

uint8_t as3933_read(uint8_t addr)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.cmd = addr | READ;
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

void as3933_init()
{
}

void as3933_crystal_osc_select(bool select)
{
}

void as3933_set_channel(uint8_t channel, bool value)
{
}

void as3933_set_manchaster_decode(bool select)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    uint8_t resp = as3933_read(R1);
    r1_t *r1 = (r1_t *)resp;
    r1->en_manch = select;
    as3933_write(R1, resp);
}

void as3933_set_bitrate(uint8_t value)
{
}

void as3933_band_select(uint32_t freq)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    uint8_t resp = as3933_read(R8);
    r8_t *r8 = (r8_t *)resp;
    ESP_LOGD(TAG, "resp %x", resp);
    ESP_LOGD(TAG, "band_sel %x", r8->band_sel);
    if (freq >= 15000 && freq < 23000)
    {
        r8->band_sel = RANGE_15_23KHZ;
    }
    if (freq >= 23000 && freq < 40000)
    {
        r8->band_sel = RANGE_23_40KHZ;
    }
    if (freq >= 40000 && freq < 65000)
    {
        r8->band_sel = RANGE_40_65KHZ;
    }
    if (freq >= 65000 && freq < 95000)
    {
        r8->band_sel = RANGE_65_95KHZ;
    }
    if (freq >= 95000 && freq <= 150000)
    {
        r8->band_sel = RANGE_95_150KHZ;
    }
    as3933_write(R8, resp);
}

void as3933_set_route_res_freq_on_dat(uint8_t channel, bool value)
{
}

void as3933_set_capacity(uint8_t channel, uint8_t value)
{
}

void as3933_set_config()
{
}

static void IRAM_ATTR as3933_on_wake_up_irq(void *context)
{
}