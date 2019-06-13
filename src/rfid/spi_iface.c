#include "include/spi_iface_generic.h"
#include "include/rfid_controller.h"

static spi_interface_t *spi_interface_ptr = NULL;

/**
 * Initialization of RFID controller
 */
esp_err_t spi_init(spi_device_t device, void **handler)
{
    void *port_handler = NULL;
    esp_err_t error = ESP_ERR_NOT_SUPPORTED;
    switch (device)
    {
    case RFID1:
    case RFID2:
        error = as3933_interface_create(device, &port_handler);
        break;
    default:
        return ESP_ERR_NOT_SUPPORTED;
    }
    // Allocate space for master interface structure
    if (spi_interface_ptr == NULL)
    {
        spi_interface_ptr = malloc(sizeof(spi_interface_t));
    }
    SPI_CHECK((spi_interface_ptr != NULL),
              ESP_ERR_INVALID_STATE,
              "SPI interface is not correctly initialized.");
    return error;
}

esp_err_t spi_setup(spi_device_t device, void **handler)
{
    void *port_handler = NULL;
    esp_err_t error = ESP_ERR_NOT_SUPPORTED;
    switch (device)
    {
    case RFID1:
    case RFID2:
        error = spi_interface_ptr->setup(device, handler);
        break;
    default:
        return ESP_ERR_NOT_SUPPORTED;
    }

    return error;
}

esp_err_t spi_destroy()
{
    esp_err_t error = ESP_OK;
    SPI_CHECK((spi_interface_ptr != NULL),
              ESP_ERR_INVALID_STATE,
              "SPI interface is not correctly initialized.");
    SPI_CHECK((spi_interface_ptr->destroy != NULL),
              ESP_ERR_INVALID_STATE,
              "SPI interface is not correctly initialized.");
    error = spi_interface_ptr->destroy();
    SPI_CHECK((error == ESP_OK),
              ESP_ERR_INVALID_STATE,
              "SPI  destroy failure error=(0x%x).",
              (uint16_t)error);
    return error;
}

esp_err_t spi_cmd(uint8_t cmd)
{
    esp_err_t error = ESP_OK;
    SPI_CHECK((spi_interface_ptr != NULL),
              ESP_ERR_INVALID_STATE,
              "SPI interface is not correctly initialized.");
    SPI_CHECK((spi_interface_ptr->destroy != NULL),
              ESP_ERR_INVALID_STATE,
              "SPI interface is not correctly initialized.");
    error = spi_interface_ptr->cmd(cmd);
    SPI_CHECK((error == ESP_OK),
              ESP_ERR_INVALID_STATE,
              "SPI CMD failure error=(0x%x).",
              (uint16_t)error);
    return error;
}

esp_err_t spi_read_byte(uint8_t addr, uint8_t *byte)
{
    esp_err_t error = ESP_OK;
    SPI_CHECK((spi_interface_ptr != NULL),
              ESP_ERR_INVALID_STATE,
              "SPI interface is not correctly initialized.");
    SPI_CHECK((spi_interface_ptr->destroy != NULL),
              ESP_ERR_INVALID_STATE,
              "SPI interface is not correctly initialized.");
    error = spi_interface_ptr->read_byte(addr, byte);
    SPI_CHECK((error == ESP_OK),
              ESP_ERR_INVALID_STATE,
              "SPI Read byte failure error=(0x%x).",
              (uint16_t)error);
    return error;
}

esp_err_t spi_write_byte(uint8_t addr, uint8_t byte)
{
    esp_err_t error = ESP_OK;
    SPI_CHECK((spi_interface_ptr != NULL),
              ESP_ERR_INVALID_STATE,
              "SPI interface is not correctly initialized.");
    SPI_CHECK((spi_interface_ptr->destroy != NULL),
              ESP_ERR_INVALID_STATE,
              "SPI interface is not correctly initialized.");
    error = spi_interface_ptr->write_byte(addr, byte);
    SPI_CHECK((error == ESP_OK),
              ESP_ERR_INVALID_STATE,
              "SPI Write byte failure error=(0x%x).",
              (uint16_t)error);
    return error;
}

esp_err_t spi_read_buffer(uint16_t addr, uint8_t *buffer, uint8_t size)
{
    esp_err_t error = ESP_OK;
    SPI_CHECK((spi_interface_ptr != NULL),
              ESP_ERR_INVALID_STATE,
              "SPI interface is not correctly initialized.");
    SPI_CHECK((spi_interface_ptr->destroy != NULL),
              ESP_ERR_INVALID_STATE,
              "SPI interface is not correctly initialized.");
    error = spi_interface_ptr->read_buffer(addr, buffer, size);
    SPI_CHECK((error == ESP_OK),
              ESP_ERR_INVALID_STATE,
              "SPI Read buffer failure error=(0x%x).",
              (uint16_t)error);
    return error;
}

esp_err_t spi_write_buffer(uint16_t addr, uint8_t *buffer, uint8_t size)
{
    esp_err_t error = ESP_OK;
    SPI_CHECK((spi_interface_ptr != NULL),
              ESP_ERR_INVALID_STATE,
              "SPI interface is not correctly initialized.");
    SPI_CHECK((spi_interface_ptr->destroy != NULL),
              ESP_ERR_INVALID_STATE,
              "SPI interface is not correctly initialized.");
    error = spi_interface_ptr->write_buffer(addr, buffer, size);
    SPI_CHECK((error == ESP_OK),
              ESP_ERR_INVALID_STATE,
              "SPI Write buffer failure error=(0x%x).",
              (uint16_t)error);
    return error;
}
