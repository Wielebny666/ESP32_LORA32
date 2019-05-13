/*!
 * \file      sx1276-board.c
 *
 * \brief     Target board SX1276 driver implementation
 *
 * \copyright Revised BSD License, see section \ref LICENSE.
 *
 * \code
 *                ______                              _
 *               / _____)             _              | |
 *              ( (____  _____ ____ _| |_ _____  ____| |__
 *               \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 *               _____) ) ____| | | || |_| ____( (___| | | |
 *              (______/|_____)_|_|_| \__)_____)\____)_| |_|
 *              (C)2013-2017 Semtech
 *
 * \endcode
 *
 * \author    Miguel Luis ( Semtech )
 *
 * \author    Gregory Cristian ( Semtech )
 */
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "hardware.h"
#include "radio.h"
#include "sx1276-board.h"

static const char *TAG = "sx1276-board";

/*!
 * \brief Gets the board PA selection configuration
 *
 * \param [IN] power Selects the right PA according to the wanted power.
 * \retval PaSelect RegPaConfig PaSelect value
 */
static uint8_t SX1276GetPaSelect(int8_t power);

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
        SX1276SetTxContinuousWave,
        SX1276ReadRssi,
        SX1276Write,
        SX1276Read,
        SX1276WriteBuffer,
        SX1276ReadBuffer,
        SX1276SetMaxPayloadLength,
        SX1276SetPublicNetwork,
        SX1276GetWakeupTime,
        NULL, // void ( *IrqProcess )( void )
        NULL, // void ( *RxBoosted )( uint32_t timeout ) - SX126x Only
        NULL, // void ( *SetRxDutyCycle )( uint32_t rxTime, uint32_t sleepTime ) - SX126x Only
};

void SX1276IoInit(void)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

     gpio_config_t sx1267_dio = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        //.pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pin_bit_mask = GPIO_INPUT_PIN_SEL,
        .intr_type = GPIO_PIN_INTR_DISABLE};
    ESP_ERROR_CHECK(gpio_config(&sx1267_dio));

    // setup reset and slave select pin
    sx1267_dio.pin_bit_mask = BIT(RADIO_RESET);
    sx1267_dio.mode = GPIO_MODE_OUTPUT;
    sx1267_dio.pull_up_en = GPIO_PULLUP_DISABLE;
    sx1267_dio.pull_down_en = GPIO_PULLDOWN_DISABLE;
    sx1267_dio.intr_type = GPIO_PIN_INTR_DISABLE;
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
    ESP_ERROR_CHECK(gpio_isr_handler_add(RADIO_DIO_0, (gpio_isr_t)irqHandlers[0], NULL));
#endif
#ifdef RADIO_DIO_1
    ESP_ERROR_CHECK(gpio_isr_handler_add(RADIO_DIO_1, (gpio_isr_t)irqHandlers[1], NULL));
#endif
#ifdef RADIO_DIO_2
    ESP_ERROR_CHECK(gpio_isr_handler_add(RADIO_DIO_2, (gpio_isr_t)irqHandlers[2], NULL));
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

    gpio_config_t sx1267_dio = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pin_bit_mask = GPIO_INPUT_PIN_SEL,
        .intr_type = GPIO_PIN_INTR_POSEDGE};
    ESP_ERROR_CHECK(gpio_config(&sx1267_dio));
}

void SX1276IoDeInit(void)
{
    // disable pin interrupts
    gpio_config_t sx1276_io;
    sx1276_io.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    sx1276_io.mode = GPIO_MODE_INPUT;
    sx1276_io.pull_up_en = GPIO_PULLDOWN_DISABLE;
    sx1276_io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    sx1276_io.intr_type = GPIO_PIN_INTR_DISABLE;
    gpio_config(&sx1276_io);

#ifdef RADIO_DIO_0
    ESP_ERROR_CHECK(gpio_isr_handler_remove(RADIO_DIO_0));
#endif
#ifdef RADIO_DIO_1
    ESP_ERROR_CHECK(gpio_isr_handler_remove(RADIO_DIO_1));
#endif
#ifdef RADIO_DIO_2
    ESP_ERROR_CHECK(gpio_isr_handler_remove(RADIO_DIO_2));
#endif
#ifdef RADIO_DIO_3
    ESP_ERROR_CHECK(gpio_isr_handler_remove(RADIO_DIO_3));
#endif
#ifdef RADIO_DIO_4
    ESP_ERROR_CHECK(gpio_isr_handler_remove(RADIO_DIO_4));
#endif
#ifdef RADIO_DIO_5
    ESP_ERROR_CHECK(gpio_isr_handler_remove(RADIO_DIO_5));
#endif
}

uint32_t SX1276GetBoardTcxoWakeupTime(void)
{
    return BOARD_TCXO_WAKEUP_TIME;
}

void SX1276Reset(void)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    // Set RESET pin to 0
    ESP_ERROR_CHECK(gpio_set_level(SX1276.Reset, 0));

    // Wait 1 ms
    vTaskDelay(1 / portTICK_PERIOD_MS);

    // Configure RESET as input
    ESP_ERROR_CHECK(gpio_set_level(SX1276.Reset, 1));

    // Wait 6 ms
    vTaskDelay(6 / portTICK_PERIOD_MS);
}

void SX1276SetRfTxPower(int8_t power)
{
    uint8_t paConfig = 0;
    uint8_t paDac = 0;

    paConfig = SX1276Read(REG_PACONFIG);
    paDac = SX1276Read(REG_PADAC);

    paConfig = (paConfig & RF_PACONFIG_PASELECT_MASK) | SX1276GetPaSelect(power);

    if ((paConfig & RF_PACONFIG_PASELECT_PABOOST) == RF_PACONFIG_PASELECT_PABOOST)
    {
        if (power > 17)
        {
            paDac = (paDac & RF_PADAC_20DBM_MASK) | RF_PADAC_20DBM_ON;
        }
        else
        {
            paDac = (paDac & RF_PADAC_20DBM_MASK) | RF_PADAC_20DBM_OFF;
        }
        if ((paDac & RF_PADAC_20DBM_ON) == RF_PADAC_20DBM_ON)
        {
            if (power < 5)
            {
                power = 5;
            }
            if (power > 20)
            {
                power = 20;
            }
            paConfig = (paConfig & RF_PACONFIG_OUTPUTPOWER_MASK) | (uint8_t)((uint16_t)(power - 5) & 0x0F);
        }
        else
        {
            if (power < 2)
            {
                power = 2;
            }
            if (power > 17)
            {
                power = 17;
            }
            paConfig = (paConfig & RF_PACONFIG_OUTPUTPOWER_MASK) | (uint8_t)((uint16_t)(power - 2) & 0x0F);
        }
    }
    else
    {
        if (power > 0)
        {
            if (power > 15)
            {
                power = 15;
            }
            paConfig = (paConfig & RF_PACONFIG_MAX_POWER_MASK & RF_PACONFIG_OUTPUTPOWER_MASK) | (7 << 4) | (power);
        }
        else
        {
            if (power < -4)
            {
                power = -4;
            }
            paConfig = (paConfig & RF_PACONFIG_MAX_POWER_MASK & RF_PACONFIG_OUTPUTPOWER_MASK) | (0 << 4) | (power + 4);
        }
    }
    SX1276Write(REG_PACONFIG, paConfig);
    SX1276Write(REG_PADAC, paDac);
}

static uint8_t SX1276GetPaSelect(int8_t power)
{
    if (power > 14)
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
