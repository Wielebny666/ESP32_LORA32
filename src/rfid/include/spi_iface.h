#ifndef __SPI_IFACE_H__
#define __SPI_IFACE_H__

#include "spi_iface_common.h"

esp_err_t spi_init(spi_device_t device, void **handler);
esp_err_t spi_destroy();
esp_err_t spi_cmd(uint8_t cmd);
esp_err_t spi_read_byte(uint8_t addr, uint8_t *byte);
esp_err_t spi_write_byte(uint8_t addr, uint8_t byte);
esp_err_t spi_read_buffer(uint16_t addr, uint8_t *buffer, uint8_t size);
esp_err_t spi_write_buffer(uint16_t addr, uint8_t *buffer, uint8_t size);

#endif