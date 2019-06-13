#include <string.h>

#include "freertos/FreeRTOS.h"

#include "include/spi_iface_generic.h"
#include "include/rfid_controller.h"

#include "driver/spi_master.h"
#include "esp_err.h"

#include "hardware.h"

static spi_interface_t *spi_interface_ptr = NULL;
static spi_device_handle_t as3933;

esp_err_t as3933_setup(spi_device_t device, void *handler);
esp_err_t as3933_destroy(void);
esp_err_t as3933_cmd(uint8_t cmd);
esp_err_t as3933_read(uint8_t addr, uint8_t *byte);
esp_err_t as3933_write(uint8_t addr, uint8_t byte);
esp_err_t as3933_read_buffer(uint16_t addr, uint8_t *buffer, uint8_t size);
esp_err_t as3933_write_buffer(uint16_t addr, uint8_t *buffer, uint8_t size);

esp_err_t as3933_interface_create(spi_device_t device, void **handler)
{
    esp_err_t error = ESP_ERR_NOT_SUPPORTED;
    SPI_CHECK((device < DEVICE_MAX),
              ESP_ERR_INVALID_ARG,
              "incorrect device selected = %u.",
              (uint8_t)device);
    // Allocate space for master interface structure
    if (spi_interface_ptr == NULL)
    {
        spi_interface_ptr = malloc(sizeof(spi_interface_t));
    }
    SPI_ASSERT(spi_interface_ptr != NULL);
    // Initialize interface properties
    spi_communication_info_t *spi_comm = &spi_interface_ptr->opts[device].spi_comm;
    spi_comm->port = RFID_SPI;
    spi_comm->miso = RFID_MISO;
    spi_comm->mosi = RFID_MOSI;
    spi_comm->sclk = RFID_SCLK;
    spi_comm->cs = RFID_CS;
    spi_comm->spi_clock_speed_hz = 10;

    spi_interface_ptr->init = as3933_interface_create;
    spi_interface_ptr->setup = as3933_setup;
    spi_interface_ptr->destroy = NULL;
    spi_interface_ptr->cmd = as3933_cmd;
    spi_interface_ptr->read_byte = as3933_read;
    spi_interface_ptr->write_byte = as3933_write;
    spi_interface_ptr->read_buffer = as3933_read_buffer;
    spi_interface_ptr->write_buffer = as3933_write_buffer;

    return error;
}

esp_err_t as3933_setup(spi_device_t device, void *handler)
{
    esp_err_t error = ESP_OK;
    spi_communication_info_t *spi_comm = &spi_interface_ptr->opts[device].spi_comm;

    spi_bus_config_t buscfg = {
        .miso_io_num = spi_comm->miso,
        .mosi_io_num = spi_comm->mosi,
        .sclk_io_num = spi_comm->sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1};

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_MASTER_FREQ_10M / 2, //Clock out at 10 MHz
        .mode = 1,                                 //SPI mode 1
        .spics_io_num = spi_comm->cs,              //CS pin
        .flags = SPI_DEVICE_POSITIVE_CS,
        .queue_size = 1, //We want to be able to queue 7 transactions at a time
        .command_bits = 8};

    //Initialize the SPI bus
    error = spi_bus_initialize(RFID_SPI, &buscfg, 0);
    SPI_CHECK((error == ESP_OK),
              ESP_ERR_INVALID_STATE,
              "SPI device selected = %u. initialize fail",
              (uint8_t)device);
    //Attach the Device to the SPI bus
    error = spi_bus_add_device(RFID_SPI, &devcfg, &as3933);
    SPI_CHECK((error == ESP_OK),
              ESP_ERR_INVALID_STATE,
              "SPI device %u. add fail",
              (uint8_t)device);
    return error;
}

esp_err_t as3933_destroy(void)
{
    esp_err_t error = ESP_OK;
    SPI_CHECK((spi_interface_ptr != NULL),
              ESP_ERR_INVALID_STATE,
              "SPI interface uninitialized.");
    free(spi_interface_ptr);
    return error;
}

esp_err_t as3933_cmd(uint8_t cmd)
{
    esp_err_t error = ESP_OK;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 0;
    t.cmd = cmd | 0xC0;

    error = spi_device_polling_transmit(as3933, &t);
    return error;
}

esp_err_t as3933_read(uint8_t addr, uint8_t *byte)
{
    esp_err_t error = ESP_OK;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.cmd = addr | 0x30;
    t.flags = SPI_TRANS_USE_RXDATA;

    error = spi_device_polling_transmit(as3933, &t);
    *byte = t.rx_data[0];
    return error;
}

esp_err_t as3933_write(uint8_t addr, uint8_t byte)
{
    esp_err_t error = ESP_OK;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.cmd = addr & 0x3F;
    t.flags = SPI_TRANS_USE_TXDATA;
    t.tx_data[0] = byte;

    error = spi_device_polling_transmit(as3933, &t);
    return error;
}

esp_err_t as3933_read_buffer(uint16_t addr, uint8_t *buffer, uint8_t size)
{
    esp_err_t error = ESP_OK;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = size * 8;
    t.cmd = addr & 0x7F;
    t.rx_buffer = buffer;

    error = spi_device_polling_transmit(as3933, &t);
    return error;
}

esp_err_t as3933_write_buffer(uint16_t addr, uint8_t *buffer, uint8_t size)
{
    esp_err_t error = ESP_OK;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = size * 8;
    t.cmd = addr | 0x80;
    t.tx_buffer = buffer;

    error = spi_device_polling_transmit(as3933, &t);
    return error;
}
