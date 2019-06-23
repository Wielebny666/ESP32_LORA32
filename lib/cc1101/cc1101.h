#ifndef __CC1101_H__
#define __CC1101_H__

#include "cc1101_regs.h"

void cc1101_spi_init();
void cc1101_init_config(void);
void cc1101_read_config(void);
void cc1101_print_config(void);

void cc1101_set_carrier_frequency( double frequencyInMHz);
double cc1101_get_carrier_frequency(void);

void cc1101_set_baud_rate( double baud_rate_in_khz);
double cc1101_get_baud_rate(void);

void cc1101_set_rx_channel_bandwidth( uint16_t bandwidth);
uint16_t cc1101_get_rx_channel_bandwidth(void);

void cc1101_set_modulation_format( mod_format_t modFormat);
mod_format_t cc1101_get_modulation_format(void);

void cc1101_set_manchester_encoding( uint8_t manchester_en);
uint8_t cc1101_get_manchester_encoding(void);

void cc1101_set_sync_mode( sync_mode_t sync_mode);
sync_mode_t cc1101_get_sync_mode(void);

void cc1101_set_packet_length( uint8_t packet_length);
uint8_t cc1101_get_packet_length(void);

void cc1101_set_crc_enable( uint8_t crc_en);
uint8_t cc1101_get_crc_enable(void);

void cc1101_set_white_data( uint8_t white_data);
uint8_t cc1101_get_white_data(void);

void cc1101_set_pkt_format( pkt_format_t pkt_format);
pkt_format_t cc1101_get_pkt_format(void);

void cc1101_set_length_config( pck_length_config_t pck_length_config);
pck_length_config_t cc1101_get_length_config(void);

void cc1101_set_txoff_mode( txoff_mode_t txoff_mode);
txoff_mode_t cc1101_get_txoff_mode(void);

void cc1101_set_rxoff_mode( rxoff_mode_t rxoff_mode);
rxoff_mode_t cc1101_get_rxoff_mode(void);

void cc1101_set_cca_mode( cca_mode_t cca_mode);
cca_mode_t cc1101_get_cca_mode(void);

void cc1101_set_sync_word( sync_word_t sync_word);
sync_word_t cc1101_get_sync_word(void);

void cc1101_set_forward_error_correction( uint8_t fec_en);
uint8_t cc1101_get_forward_error_correction(void);

void cc1101_set_num_preamble( num_preamble_t num_preamble);
num_preamble_t cc1101_get_num_preamble(void);

void cc1101_set_deviation_frequency( double frequencyInkHz);
double cc1101_get_deviation_frequency(void);

void cc1101_set_controls_address_check( adr_chk_t adr_chk);
adr_chk_t cc1101_get_controls_address_check(void);

void cc1101_set_append_status( uint8_t append_status);
uint8_t cc1101_get_append_status(void);

void cc1101_set_preamble_quality_threshold( uint8_t pqt);
uint8_t cc1101_get_preamble_quality_threshold(void);

void cc1101_set_crc_autoflush( uint8_t crc_autoflush);
uint8_t cc1101_get_crc_autoflush(void);

void cc1101_set_device_address( uint8_t device_addr);
uint8_t cc1101_get_device_address(void);

void cc1101_set_channel_number( uint8_t chan);
uint8_t cc1101_get_channel_number(void);

void cc1101_set_rx_attenuation( close_in_rx_t close_in_rx);
uint8_t cc1101_get_rx_attenuation(void);

void cc1101_set_pa_power_setting( uint8_t pa_power);
uint8_t cc1101_get_pa_power_setting(void);

void cc1101_set_filter_length( filter_length_t filter_length);
uint8_t cc1101_get_filter_length(void);

void cc1101_set_hysteresis_level( hyst_level_t hyst_level);
hyst_level_t cc1101_get_hysteresis_level(void);

void cc1101_set_magn_target( magn_target_t magn_target);
magn_target_t cc1101_get_magn_target(void);

uint8_t cc1101_get_rssi(void);
float cc1101_get_rssi_dbm(void);

uint8_t cc1101_get_tx_fifo_info(void);

void cc1101_reset(void);
void cc1101_set_idle_state(void);
void cc1101_set_flush_tx(void);
void cc1101_set_flush_rx(void);
void cc1101_set_tx_state(void);
void cc1101_set_rx_state(void);
void cc1101_set_calibrate_state(void);

void cc1101_init(void);
void cc1101_set_pa_table(void);
void cc1101_get_pa_table(void);

void send_data(void);

#endif