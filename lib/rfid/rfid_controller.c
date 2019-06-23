#include <string.h>

#include "freertos/FreeRTOS.h"

#include "spi_iface_generic.h"
#include "rfid_controller.h"

#include "driver/spi_master.h"
#include "esp_err.h"

static const char* TAG = "rfid_controller";

#define R_CONTROLLER_CHECK(a, ret_val, str, ...) \
    if (!(a)) { \
        ESP_LOGE(TAG, "%s(%u): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
        return (ret_val); \
    }

#define R_CONTROLLER_ASSERT(con)                                                                     \
    do                                                                                      \
    {                                                                                       \
        if (!(con))                                                                         \
        {                                                                                   \
            ESP_LOGE(TAG, "assert errno:%d, errno_str: !(%s)", errno, strerror(errno)); \
            assert(0 && #con);                                                              \
        }                                                                                   \
    } while (0)

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
    R_CONTROLLER_CHECK((device < DEVICE_MAX),
              ESP_ERR_INVALID_ARG,
              "incorrect device selected = %u.",
              (uint8_t)device);
    // Allocate space for master interface structure
    if (spi_interface_ptr == NULL)
    {
        spi_interface_ptr = malloc(sizeof(spi_interface_t));
        memset(spi_interface_ptr, 0, sizeof(spi_interface_t));
    }
    R_CONTROLLER_ASSERT(spi_interface_ptr != NULL);
    // Initialize interface properties
    spi_communication_info_t *spi_comm = &spi_interface_ptr->opts[device].spi_comm;
    spi_comm->port = VSPI_HOST;
    spi_comm->miso = GPIO_NUM_19;
    spi_comm->mosi = GPIO_NUM_23;
    spi_comm->sclk = GPIO_NUM_18;
    spi_comm->cs = GPIO_NUM_5;
    spi_comm->spi_clock_speed_hz = SPI_MASTER_FREQ_10M;

    spi_interface_ptr->init = as3933_interface_create;
    spi_interface_ptr->setup = as3933_setup;
    spi_interface_ptr->destroy = NULL;
    spi_interface_ptr->cmd = as3933_cmd;
    spi_interface_ptr->read_byte = as3933_read;
    spi_interface_ptr->write_byte = as3933_write;
    spi_interface_ptr->read_buffer = as3933_read_buffer;
    spi_interface_ptr->write_buffer = as3933_write_buffer;

	*handler = spi_interface_ptr;
    return ESP_OK;
}

esp_err_t as3933_setup(spi_device_t device, void *comm_info)
{
    esp_err_t error = ESP_OK;
    spi_options_t *spi_comm = &spi_interface_ptr->opts[device];

    const spi_communication_info_t *comm_info_ptr = (spi_communication_info_t *)comm_info;

    spi_bus_config_t buscfg = {
        .miso_io_num = comm_info_ptr->miso,
        .mosi_io_num = comm_info_ptr->mosi,
        .sclk_io_num = comm_info_ptr->sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1};

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = comm_info_ptr->spi_clock_speed_hz,
        .mode = 1,                                           //SPI mode 1
        .spics_io_num = comm_info_ptr->cs,                   //CS pin
        .flags = SPI_DEVICE_POSITIVE_CS,
        .queue_size = 1, //We want to be able to queue 7 transactions at a time
        .command_bits = 8};

    //Initialize the SPI bus
    error = spi_bus_initialize(comm_info_ptr->port, &buscfg, 0);
    R_CONTROLLER_CHECK((error == ESP_OK),
              ESP_ERR_INVALID_STATE,
              "SPI device selected = %u. initialize fail",
              (uint8_t)device);
    //Attach the Device to the SPI bus
    error = spi_bus_add_device(comm_info_ptr->port, &devcfg, &as3933);
    R_CONTROLLER_CHECK((error == ESP_OK),
              ESP_ERR_INVALID_STATE,
              "SPI device %u. add fail",
              (uint8_t)device);
    spi_comm->spi_comm = *(spi_communication_info_t *)comm_info;
    return error;
}

esp_err_t as3933_destroy(void)
{
    esp_err_t error = ESP_OK;
    R_CONTROLLER_CHECK((spi_interface_ptr != NULL),
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
