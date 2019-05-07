/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: SX1276 driver specific target board functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/
#include "hardware.h"
#include "board.h"
#include "radio.h"
#include "sx1276/sx1276.h"
#include "sx1276-board.h"

static const char *TAG = "sx1276-board";

/*!
 * Radio driver structure initialization
 */
const struct Radio_s Radio =
    {
        SX1276Init,
        SX1276GetStatus,
        SX1276SetModem,
        SX1276SetChannel,
        SX1276IsChannelFree,
        SX1276Random,
        SX1276SetRxConfig,
        SX1276SetTxConfig,
        SX1276CheckRfFrequency,
        SX1276GetTimeOnAir,
        SX1276Send,
        SX1276SetSleep,
        SX1276SetStby,
        SX1276SetRx,
        SX1276StartCad,
        SX1276ReadRssi,
        SX1276Write,
        SX1276Read,
        SX1276WriteBuffer,
        SX1276ReadBuffer,
        SX1276SetMaxPayloadLength};

void SX1276IoInit(void)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    gpio_config_t sx1267_dio = {
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_PIN_INTR_POSEDGE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pin_bit_mask = GPIO_INPUT_PIN_SEL};
    ESP_ERROR_CHECK(gpio_config(&sx1267_dio));

    SX1276.Reset = RADIO_RESET;
    SX1276.DIO0 = RADIO_DIO_0;
    SX1276.DIO1 = RADIO_DIO_1;
    SX1276.DIO2 = RADIO_DIO_2;
}

void SX1276IoIrqInit(DioIrqHandler **irqHandlers)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT));
#ifdef RADIO_DIO_0
    ESP_ERROR_CHECK(gpio_isr_handler_add(RADIO_DIO_0, (void *)irqHandlers[0], NULL));
#endif
#ifdef RADIO_DIO_1
    ESP_ERROR_CHECK(gpio_isr_handler_add(RADIO_DIO_1, (void *)irqHandlers[1], NULL));
#endif
#ifdef RADIO_DIO_2
    ESP_ERROR_CHECK(gpio_isr_handler_add(RADIO_DIO_2, (void *)irqHandlers[2], NULL));
#endif
#ifdef RADIO_DIO_3
    ESP_ERROR_CHECK(gpio_isr_handler_add(RADIO_DIO_3, irqHandlers[3], NULL));
#endif
#ifdef RADIO_DIO_4
    ESP_ERROR_CHECK(gpio_isr_handler_add(RADIO_DIO_4, irqHandlers[4], NULL));
#endif
#ifdef RADIO_DIO_5
    ESP_ERROR_CHECK(gpio_isr_handler_add(RADIO_DIO_5, irqHandlers[5], NULL));
#endif
}

void SX1276IoDeInit(void)
{
}

uint8_t SX1276GetPaSelect(uint32_t channel)
{
    if (channel < RF_MID_BAND_THRESH)
    {
        return RF_PACONFIG_PASELECT_PABOOST;
    }
    else
    {
        return RF_PACONFIG_PASELECT_RFO;
    }
}

bool SX1276CheckRfFrequency(uint32_t frequency)
{
    // Implement check. Currently all frequencies are supported
    return true;
}
