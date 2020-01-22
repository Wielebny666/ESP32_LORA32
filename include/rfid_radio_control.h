#ifndef __RFID_RADIO_CONTROL_H__
#define __RFID_RADIO_CONTROL_H__

#include <stdint.h>
#include "esp_err.h"

typedef struct
{
	uint32_t frequency;
	uint8_t bitrate;
} rfid_radio_conig_t;

typedef struct
{
	bool manchaster;
	bool pattern_16bit;
	int16_t wake_up_pattern;
} rfid_packet_config_t;

esp_err_t radio_init();
void radio_setup();
void radio_reset();
void radio_wake_up_reset();
void radio_rssi_reset();
void radio_get_rssi(uint8_t *rssi1, uint8_t *rssi2, uint8_t *rssi3);
bool radio_check_init();
#endif // ! __RFID_RADIO_CONTROL_H__
