#ifndef _HARDWARE_H_
#define _HARDWARE_H_

// SDA - GPIO21
#define PIN_I2C_SDA GPIO_NUM_21
// SCL - GPIO22
#define PIN_I2C_SCL GPIO_NUM_22


//SX1276 SPI & IO
#define RADIO_RESET GPIO_NUM_23

#define RADIO_MOSI GPIO_NUM_27
#define RADIO_MISO GPIO_NUM_19
#define RADIO_SCLK GPIO_NUM_5
#define RADIO_NSS GPIO_NUM_18

#define RADIO_DIO_0 GPIO_NUM_26
#define RADIO_DIO_1 GPIO_NUM_2
#define RADIO_DIO_2 GPIO_NUM_15
// #define RADIO_DIO_2 GPIO_NUM_23
// #define RADIO_DIO_3                                 GPIO_NUM_26
// #define RADIO_DIO_4                                 GPIO_NUM_26
// #define RADIO_DIO_5                                 GPIO_NUM_26

//GREEN LED
#define LEDC_LS_CH0_GPIO GPIO_NUM_25

//RMT TX & RX
//#define RMT_TX_GPIO_NUM GPIO_NUM_17 //cc1101 Rx
#define RMT_TX_GPIO_NUM GPIO_NUM_15 //sx1276 Rx
//#define RMT_TX_GPIO_NUM GPIO_NUM_13
//#define RMT_RX_GPIO_NUM GPIO_NUM_16 //cc1101 Tx
#define RMT_RX_GPIO_NUM GPIO_NUM_15 //sx1276 Tx
//#define RMT_RX_GPIO_NUM GPIO_NUM_12
#endif