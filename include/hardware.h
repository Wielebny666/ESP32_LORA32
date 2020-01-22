#ifndef _HARDWARE_H_
#define _HARDWARE_H_

#include "driver/spi_common.h"

#define RESERVER_JTAG_TDO GPIO_NUM_15
#define RESERVER_JTAG_TMS GPIO_NUM_14
#define RESERVER_JTAG_TCK GPIO_NUM_13
#define RESERVER_JTAG_TDI GPIO_NUM_12

#ifndef PICO
    #define RESERVE_SPI_FLASH_SCK GPIO_NUM_6
    #define RESERVE_SPI_FLASH_SDO GPIO_NUM_7
    #define RESERVE_SPI_FLASH_SDI GPIO_NUM_8
    #define RESERVE_SPI_FLASH_SHD GPIO_NUM_9
    #define RESERVE_SPI_FLASH_SWP GPIO_NUM_10
    #define RESERVE_SPI_FLASH_CSC GPIO_NUM_11
#endif

#ifdef WROOM
    #ifdef CC1101
        #define SPI_RADIO VSPI_HOST

        #define RADIO_MOSI GPIO_NUM_23
        #define RADIO_MISO GPIO_NUM_19
        #define RADIO_SCLK GPIO_NUM_18
        #define RADIO_NSS GPIO_NUM_5

        #define RADIO_DIO_0 GPIO_NUM_2
        //#define RADIO_DIO_1 GPIO_NUM_15
        #define RADIO_DIO_2 GPIO_NUM_4
    #elif SX1276
        //SX1276 SPI & IO
        #define RADIO_RESET GPIO_NUM_23

        #define RADIO_MOSI GPIO_NUM_27
        #define RADIO_MISO GPIO_NUM_19
        #define RADIO_SCLK GPIO_NUM_5
        #define RADIO_NSS GPIO_NUM_18

        #define RADIO_DIO_0 GPIO_NUM_26
        //outside
        #define RADIO_DIO_1 GPIO_NUM_15
        #define RADIO_DIO_2 GPIO_NUM_13

        #define GPIO_INPUT_PIN_SEL (1ULL << RADIO_DIO_0) | (1ULL << RADIO_DIO_1) | (1ULL << RADIO_DIO_2)
    #endif

    #ifdef AS3933
        #define RFID_SPI VSPI_HOST
        #define RFID_CLK_SPEED (SPI_MASTER_FREQ_10M/2)
        #define RFID_MOSI GPIO_NUM_23
        #define RFID_MISO GPIO_NUM_19
        #define RFID_SCLK GPIO_NUM_18
        #define RFID_CS GPIO_NUM_5
        
        #define RFID_DATA GPIO_NUM_4
        #define RFID_WAKE_UP GPIO_NUM_2
    #endif

    #define LEDC_LS_CH0_GPIO GPIO_NUM_2
#elif TTGO
    #ifdef SSD1306
        #define PIN_I2C_SDA GPIO_NUM_21 // SDA - GPIO21
        #define PIN_I2C_SCL GPIO_NUM_22 // SCL - GPIO22
    #endif

    #ifdef AS3933
        #define RFID_SPI HSPI_HOST
        #define RFID_MOSI GPIO_NUM_13
        #define RFID_MISO GPIO_NUM_12
        #define RFID_SCLK GPIO_NUM_14
        #define RFID_CS GPIO_NUM_15
        #define RFID_DATA GPIO_NUM_34
        #define RFID_WAKE_UP GPIO_NUM_25
    #endif

#ifdef SX1276
    //SX1276 SPI & IO
    #define RADIO_SPI HSPI_HOST
    #define RADIO_RESET GPIO_NUM_23
    #define RADIO_MOSI GPIO_NUM_27
    #define RADIO_MISO GPIO_NUM_19
    #define RADIO_SCLK GPIO_NUM_5
    #define RADIO_NSS GPIO_NUM_18

    #define RADIO_DIO_0 GPIO_NUM_26
    //outside
    #define RADIO_DIO_1 GPIO_NUM_34
    #define RADIO_DIO_2 GPIO_NUM_35

    #define GPIO_INPUT_PIN_SEL (1ULL << RADIO_DIO_0) | (1ULL << RADIO_DIO_1) | (1ULL << RADIO_DIO_2)

    #define RADIO_WAKEUP_LEVEL_DEFAULT 0
#endif

//RMT TX & RX
#ifdef SX1276_ASYNC
    #define RMT_TX_GPIO_NUM GPIO_NUM_25 //sx1276 Rx
    #define RMT_RX_GPIO_NUM GPIO_NUM_15 //sx1276 Tx
#endif

//GREEN LED
#define LEDC_LS_CH0_GPIO GPIO_NUM_25

//CC1101
#ifdef CC1101
#define RADIO_MOSI GPIO_NUM_27
#define RADIO_MISO GPIO_NUM_19
#define RADIO_SCLK GPIO_NUM_5
#define RADIO_NSS GPIO_NUM_18
#define RMT_TX_GPIO_NUM GPIO_NUM_17 //cc1101 Rx
#define RMT_RX_GPIO_NUM GPIO_NUM_16 //cc1101 Tx
#endif
#endif

#define ESP_INTR_FLAG_DEFAULT 0

/*!
 * Defines the time required for the TCXO to wakeup [ms].
 */
#define BOARD_TCXO_WAKEUP_TIME 0

#endif
