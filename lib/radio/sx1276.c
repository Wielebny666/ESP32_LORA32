/*
 * THE FOLLOWING FIRMWARE IS PROVIDED: (1) "AS IS" WITH NO WARRANTY; AND 
 * (2)TO ENABLE ACCESS TO CODING INFORMATION TO GUIDE AND FACILITATE CUSTOMER.
 * CONSEQUENTLY, SEMTECH SHALL NOT BE HELD LIABLE FOR ANY DIRECT, INDIRECT OR
 * CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE CONTENT
 * OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING INFORMATION
 * CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 * 
 * Copyright (C) SEMTECH S.A.
 */
/*! 
 * \file       sx1276.c
 * \brief      SX1276 RF chip driver
 *
 * \version    2.0.0 
 * \date       May 6 2013
 * \author     Gregory Cristian
 *
 * Last modified by Miguel Luis on Jun 19 2013
 */
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"

#include "sx1276_fsk_regs.h"
#include "sx1276_fsk.h"
#include "sx1276_hal.h"
#include "sx1276.h"

/*!
 * SX1276 registers variable
 */
uint8_t sx1276_regs[0x70];

static const char *TAG = "sx1276";

void sx1276_init(void)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    // Initialize FSK registers structure
    sx1276 = (sx1276_reg_t *)sx1276_regs;
    assert(sx1276);

    sx1276_init_io();

    sx1276_reset();

    // Initialize FSK modem
    sx1276_fsk_init();
}

void sx1276_reset(void)
{
    sx1276_set_reset(RADIO_RESET_ON);

    // Wait 1ms
    vTaskDelay(10 / portTICK_PERIOD_MS);

    sx1276_set_reset(RADIO_RESET_OFF);

    // Wait 6ms
    vTaskDelay(60 / portTICK_PERIOD_MS);
}

void SX1276SetLoRaOn(bool enable)
{
    // sx1276_read_buffer(REG_OPMODE, (uint8_t *)sx1276 + 1, 0x70 - 1);
}

void sx1276_set_opmode(uint8_t opMode)
{

    sx1276_fsk_set_opmode(opMode);
}

// uint8_t SX1276GetOpMode(void)
// {

//     return SX1276FskGetOpMode();
// }

// double SX1276ReadRssi(void)
// {

//     return SX1276FskReadRssi();
// }

// uint8_t SX1276ReadRxGain(void)
// {

//     return SX1276FskReadRxGain();
// }

// uint8_t SX1276GetPacketRxGain(void)
// {

//     return SX1276FskGetPacketRxGain();
// }

// int8_t SX1276GetPacketSnr(void)
// {

//     while (1)
//     {
//         // Useless in FSK mode
//         // Block program here
//     }
// }

// double SX1276GetPacketRssi(void)
// {

//     return SX1276FskGetPacketRssi();
// }

// uint32_t SX1276GetPacketAfc(void)
// {

//     return SX1276FskGetPacketAfc();
// }

// void SX1276StartRx(void)
// {

//     SX1276FskSetRFState(RF_STATE_RX_INIT);
// }

// void SX1276GetRxPacket(void *buffer, uint16_t *size)
// {

//     SX1276FskGetRxPacket(buffer, size);
// }

void sx1276_set_tx_packet(const void *buffer, uint16_t size)
{
    sx1276_fsk_set_tx_packet(buffer, size);
}

// uint8_t SX1276GetRFState(void)
// {

//     return SX1276FskGetRFState();
// }

// void SX1276SetRFState(uint8_t state)
// {

//     SX1276FskSetRFState(state);
// }

uint32_t sx1276_process(void)
{

    return sx1276_fsk_process();
}
