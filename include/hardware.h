#ifndef _HARDWARE_H_
#define _HARDWARE_H_

#define TTGO

// SDA - GPIO21
#define PIN_I2C_SDA GPIO_NUM_21
// SCL - GPIO22
#define PIN_I2C_SCL GPIO_NUM_22


#define RFID_MOSI GPIO_NUM_13
#define RFID_MISO GPIO_NUM_12
#define RFID_SCLK GPIO_NUM_14
#define RFID_CS GPIO_NUM_15


//SX1276 SPI & IO
#define RADIO_RESET GPIO_NUM_23

#define RADIO_MOSI GPIO_NUM_27
#define RADIO_MISO GPIO_NUM_19
#define RADIO_SCLK GPIO_NUM_5
#define RADIO_NSS GPIO_NUM_18

#define RADIO_DIO_0 GPIO_NUM_26
//outside
#define RADIO_DIO_1 GPIO_NUM_34
#define RADIO_DIO_2 GPIO_NUM_35
// #define RADIO_DIO_2 GPIO_NUM_23
// #define RADIO_DIO_3                                 GPIO_NUM_26
// #define RADIO_DIO_4                                 GPIO_NUM_26
// #define RADIO_DIO_5                                 GPIO_NUM_26

#define RADIO_WAKEUP_LEVEL_DEFAULT 0

//GREEN LED
#ifdef TTGO
#define LEDC_LS_CH0_GPIO GPIO_NUM_25
#else
#define LEDC_LS_CH0_GPIO GPIO_NUM_2
#endif

//RMT TX & RX
//#define RMT_TX_GPIO_NUM GPIO_NUM_17 //cc1101 Rx
#define RMT_TX_GPIO_NUM GPIO_NUM_25 //sx1276 Rx
//#define RMT_TX_GPIO_NUM GPIO_NUM_13
//#define RMT_RX_GPIO_NUM GPIO_NUM_16 //cc1101 Tx
#define RMT_RX_GPIO_NUM GPIO_NUM_15 //sx1276 Tx
//#define RMT_RX_GPIO_NUM GPIO_NUM_12

// #define PIN_NUM_TXD UART_NUM_2_TXD_DIRECT_GPIO_NUM
// #define PIN_NUM_RXD UART_NUM_2_RXD_DIRECT_GPIO_NUM

// #define GPIO_INPUT_IO_0 (PIN_NUM_RXD)
// #define GPIO_INPUT_PIN_SEL (1ULL << GPIO_INPUT_IO_0)
// #define GPIO_OUTPUT_IO_0 (PIN_NUM_TXD)
// #define GPIO_OUTPUT_PIN_SEL (1ULL << GPIO_OUTPUT_IO_0)

#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_PIN_SEL (1ULL << RADIO_DIO_0) | (1ULL << RADIO_DIO_1) | (1ULL << RADIO_DIO_2)

/*!
 * Defines the time required for the TCXO to wakeup [ms].
 */
#define BOARD_TCXO_WAKEUP_TIME 0

#endif