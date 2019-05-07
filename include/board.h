#ifndef __BOARD_H__
#define __BOARD_H__

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "timeout_timer.h"

#include "esp_err.h"

#include "radio.h"
#include "sx1276/sx1276.h"
#include "sx1276-board.h"

#define USE_MODEM_LORA (1)

void SpiInit();

#endif