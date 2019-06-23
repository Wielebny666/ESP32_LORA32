#ifndef __SPI_IFACE_GENERIC_H__
#define __SPI_IFACE_GENERIC_H__

#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "errno.h"

#include "spi_iface_common.h"

#define SPI_TAG "SPI_CONTROLLER"

#define SPI_CHECK(a, ret_val, str, ...)                                           \
    if (!(a))                                                                     \
    {                                                                             \
        ESP_LOGE(SPI_TAG, "%s(%u): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
        return (ret_val);                                                         \
    }

#define SPI_ASSERT(con)                                                                     \
    do                                                                                      \
    {                                                                                       \
        if (!(con))                                                                         \
        {                                                                                   \
            ESP_LOGE(SPI_TAG, "assert errno:%d, errno_str: !(%s)", errno, strerror(errno)); \
            assert(0 && #con);                                                              \
        }                                                                                   \
    } while (0)

typedef struct
{
    spi_communication_info_t spi_comm;
} spi_options_t;

typedef esp_err_t (*iface_cmd)(uint8_t cmd);
typedef esp_err_t (*iface_read_byte)(uint8_t addr, uint8_t *byte);
typedef esp_err_t (*iface_write_byte)(uint8_t addr, uint8_t byte);
typedef esp_err_t (*iface_read_buffer)(uint16_t addr, uint8_t *buffer, uint8_t size);
typedef esp_err_t (*iface_write_buffer)(uint16_t addr, uint8_t *buffer, uint8_t size);

typedef struct
{
    spi_options_t opts[DEVICE_MAX];
    iface_init init;
    iface_setup setup;
    iface_destroy destroy;
    iface_cmd cmd;
    iface_read_byte read_byte;
    iface_write_byte write_byte;
    iface_read_buffer read_buffer;
    iface_write_buffer write_buffer;
} spi_interface_t;

#endif