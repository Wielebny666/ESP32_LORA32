#ifndef __SPI_IFACE_COMMON_H__
#define __SPI_IFACE_COMMON_H__

#include "driver/spi_master.h"

typedef enum
{
	RFID1,
	RFID2,
	DEVICE_MAX
} spi_device_t;

/**
 * @brief Device communication structure to setup FRID controller
 */
typedef struct
{
	spi_host_device_t port;
	gpio_num_t mosi;
	gpio_num_t miso;
	gpio_num_t sclk;
	gpio_num_t cs;
	uint32_t spi_clock_speed_hz;
} spi_communication_info_t;

typedef esp_err_t(*iface_init)(spi_device_t, void **);
typedef esp_err_t(*iface_setup)(spi_device_t, void *);
typedef esp_err_t(*iface_destroy)(void);
#endif