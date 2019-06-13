#ifndef __RFID_CONTROLLER_H__
#define __RFID_CONTROLLER_H__

#include "esp_err.h"
#include "spi_iface_common.h"

esp_err_t as3933_interface_create(spi_device_t device, void **handler);

#endif