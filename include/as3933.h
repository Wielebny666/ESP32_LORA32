#ifndef __AS3933_H__
#define __AS3933_H__

#include <stdint.h>

typedef struct
{
    int8_t Rssi1Value;
    int8_t Rssi2Value;
    int8_t Rssi3Value;
} RfIdPacketHandler_t;

typedef struct
{
    bool Manchester;
    int16_t WakeUpPattern;
    bool Pattern_16bit; 
    uint8_t Bitrate;
    uint8_t Frequency;
    RfIdPacketHandler_t PacketHandler;
} RfIdSettings_t;

typedef struct as3933_s
{
    gpio_num_t WakeUp;
    gpio_num_t D_Clk;
    gpio_num_t Data;
    spi_device_handle_t Spi;
    RfIdSettings_t Settings;
} as3933_t;

/*!
 * Hardware IO IRQ callback function definition
 */
typedef void(DioIrqHandler)(void *context);

#define XTAL_FREQ 32000

void as3933_spi_init();
void as3933_cmd(cmd_t cmd);
void as3933_write(uint8_t addr, uint8_t data);
uint8_t as3933_read(uint8_t addr);
void as3933_write_buffer(uint16_t addr, uint8_t *buffer, uint8_t size);
void as3933_read_buffer(uint16_t addr, uint8_t *buffer, uint8_t size);

void as3933_reset();

#endif