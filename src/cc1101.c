/*
 * cc1101.c
 *
 *  Created on: 13 kwi 2019
 *      Author: kurza
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "rom/ets_sys.h"

#include "cc1101.h"
#include "foreach.h"

#define TAG "cc1101"

DRAM_ATTR cc1101_init_cmd_t config_register[] = {
	//0x06 - Asserts when sync word has been sent / received, and de-asserts at the end of the packet. In RX, the pin will also deassert when a packet is discarded due to address or maximum length filtering or when the radio enters RXFIFO_OVERFLOW state. In TX the pin will de-assert if the TX FIFO underflows.
	// {CC1101_IOCFG2, 0x06}, //0x0D - Data output in async rx
	// {CC1101_IOCFG0, 0x06}, //Data input if set async tx
	// {CC1101_FIFOTHR, 0x47},
	// {CC1101_PKTCTRL1, 0x00},
	// {CC1101_PKTCTRL0, 0x01},
	// {CC1101_FREQ2, 0x10},
	// {CC1101_FREQ1, 0xB6},
	// {CC1101_FREQ0, 0x26},
	// {CC1101_MDMCFG4, 0xF5},
	// {CC1101_MDMCFG3, 0x83},
	// {CC1101_MDMCFG2, 0x30},
	// {CC1101_MDMCFG1, 0x02},
	// {CC1101_DEVIATN, 0x15},
	// {CC1101_MCSM1, 0x00},
	// {CC1101_MCSM0, 0x18},
	// {CC1101_FOCCFG, 0x14},
	// {CC1101_AGCCTRL0, 0x92},
	// {CC1101_WORCTRL, 0xFB},
	// {CC1101_FREND0, 0x11},
	// {CC1101_FSCAL3, 0xE9},
	// {CC1101_FSCAL2, 0x2A},
	// {CC1101_FSCAL1, 0x00},
	// {CC1101_FSCAL0, 0x1F},
	// {CC1101_TEST2, 0x81},
	// {CC1101_TEST1, 0x35},
	// {CC1101_TEST0, 0x09}
	{CC1101_IOCFG2, 0x0D},
	{CC1101_IOCFG0, 0x0C},
	{CC1101_FIFOTHR, 0x47},
	{CC1101_PKTCTRL0, 0x32},
	{CC1101_FSCTRL1, 0x06},
	{CC1101_FREQ2, 0x10},
	{CC1101_FREQ1, 0xB6},
	{CC1101_FREQ0, 0xDC},
	{CC1101_MDMCFG4, 0xF5},
	{CC1101_MDMCFG3, 0x83},
	{CC1101_MDMCFG2, 0x30},
	{CC1101_DEVIATN, 0x15},
	{CC1101_MCSM0, 0x18},
	{CC1101_FOCCFG, 0x14},
	{CC1101_AGCCTRL0, 0x93},
	{CC1101_WORCTRL, 0xFB},
	{CC1101_FREND0, 0x11},
	{CC1101_FSCAL3, 0xE9},
	{CC1101_FSCAL2, 0x2A},
	{CC1101_FSCAL1, 0x00},
	{CC1101_FSCAL0, 0x1F},
	{CC1101_TEST2, 0x81},
	{CC1101_TEST1, 0x35},
	{CC1101_TEST0, 0x09}};

DRAM_ATTR static uint8_t pa_table_power[] = {0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

DRAM_ATTR uint16_t chanBW_limits[] = {812, 650, 541, 464, 406, 325, 270, 232, 203, 162, 135, 116, 102, 81, 68, 58};

void cc1101_write_register(spi_device_handle_t spi, uint8_t cmd, const uint8_t byte)
{
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.length = 8;
	t.cmd = cmd;
	t.tx_buffer = &byte;

	ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &t));
}

void cc1101_write_burst_register(spi_device_handle_t spi, uint8_t cmd, const uint8_t *bytes, const uint8_t length)
{
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.length = length * 8;
	t.cmd = cmd | CC1101_WRITE_BURST;
	t.tx_buffer = bytes;

	ESP_ERROR_CHECK(spi_device_transmit(spi, &t));
}

uint8_t cc1101_read_status_register(spi_device_handle_t spi, uint8_t cmd)
{
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.length = 8;
	t.cmd = cmd | CC1101_READ_BURST;
	t.flags = SPI_TRANS_USE_RXDATA;

	ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &t));

	return t.rx_data[0];
}

uint8_t cc1101_read_config_register(spi_device_handle_t spi, uint8_t cmd)
{
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.length = 8;
	t.cmd = cmd | CC1101_READ_SINGLE;
	t.flags = SPI_TRANS_USE_RXDATA;

	ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &t));

	return t.rx_data[0];
}

uint8_t *cc1101_read_burst_register(spi_device_handle_t spi, uint8_t cmd, size_t length)
{
	ESP_LOGD(__FUNCTION__, "length %d", length);

	uint8_t *response = (uint8_t *)heap_caps_malloc(length, MALLOC_CAP_DMA);
	assert(response);
	memset(response, 0, length);

	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.length = length * 8;
	t.cmd = cmd | CC1101_READ_BURST;
	t.rx_buffer = response;

	ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &t));
	return response;
}

void cc1101_command_strobe(spi_device_handle_t spi, CC1101_command_strobe_t strobe)
{
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.length = 0;
	t.cmd = strobe;

	esp_err_t ret = spi_device_polling_transmit(spi, &t);
	ESP_ERROR_CHECK(ret);
}

void set_carrier_frequency(spi_device_handle_t spi, double frequencyInMHz)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	// bounds checking for cc1101 hardware limitations
	if (!(((frequencyInMHz >= 300) && (frequencyInMHz <= 348)) || ((frequencyInMHz >= 387) && (frequencyInMHz <= 464)) || ((frequencyInMHz >= 779) && (frequencyInMHz <= 928))))
	{
		ESP_LOGW(TAG, "Frequency out of bounds! Use 300-348, 387-464, 779-928MHz only!");
		return;
	}

	// trying to avoid any floating point issues
	double_t secondByteOverflow = fmod(frequencyInMHz, 26);
	uint8_t firstByteValue = (frequencyInMHz - secondByteOverflow) / 26;

	double_t thirdByteOverflow = fmod((secondByteOverflow * 255), 26);
	uint8_t secondByteValue = ((secondByteOverflow * 255) - thirdByteOverflow) / 26;

	double_t excessOverflow = fmod((thirdByteOverflow * 255), 26);
	uint8_t thirdByteValue = ((thirdByteOverflow * 255) - excessOverflow) / 26;

	cc1101_write_register(spi, CC1101_FREQ2, firstByteValue);
	cc1101_write_register(spi, CC1101_FREQ1, secondByteValue);
	cc1101_write_register(spi, CC1101_FREQ0, thirdByteValue);

	ESP_LOGD(TAG, "Set Carrier Frequency: %f. 1: %d, 2: %d, 3: %d", frequencyInMHz, firstByteValue, secondByteValue, thirdByteValue);
}

double get_carrier_frequency(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	double_t firstRegisterByte = cc1101_read_config_register(spi, CC1101_FREQ2);
	double_t secondRegisterByte = cc1101_read_config_register(spi, CC1101_FREQ1);
	double_t thirdRegisterByte = cc1101_read_config_register(spi, CC1101_FREQ0);

	firstRegisterByte = firstRegisterByte * 26;
	secondRegisterByte = secondRegisterByte / 255 * 26;
	thirdRegisterByte = thirdRegisterByte / 255 / 255 * 26;

	return firstRegisterByte + +secondRegisterByte + +thirdRegisterByte;
}

void set_baud_rate(spi_device_handle_t spi, double baud_rate_in_khz)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	uint8_t baudRateExponent = 0;
	uint8_t baudRateMantissa = 0;

	for (double_t tempExponent = 0; tempExponent < 16; tempExponent++)
	{
		double_t tempMantissa = (baud_rate_in_khz * 1000.0 * pow(2, 28) / (pow(2, tempExponent) * (CC1101_CLOCK_FREQUENCY * 1000000.0))) - 256 + .5;
		if (tempMantissa < 256)
		{
			baudRateExponent = tempExponent;
			baudRateMantissa = tempMantissa;
			break;
		}
	}

	mdmcfg4_t mdmcfg4 = {
		.reg = cc1101_read_config_register(spi, CC1101_MDMCFG4)};
	mdmcfg4.drate_e = baudRateExponent;

	cc1101_write_register(spi, CC1101_MDMCFG4, mdmcfg4.reg);
	cc1101_write_register(spi, CC1101_MDMCFG3, baudRateMantissa);
	ESP_LOGD(TAG, "Set Transmission Baud rate: %f. E: %d, M: %d", baud_rate_in_khz, baudRateExponent, baudRateMantissa);
}

double get_baud_rate(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mdmcfg4_t mdmcfg4 = {
		.reg = cc1101_read_config_register(spi, CC1101_MDMCFG4)};
	uint8_t baudRateExponent = mdmcfg4.drate_e;
	uint8_t baudRateMantissa = cc1101_read_config_register(spi, CC1101_MDMCFG3);

	double_t baudRate = 1000.0 * CC1101_CLOCK_FREQUENCY * (256 + baudRateMantissa) * pow(2, baudRateExponent) / pow(2, 28);

	return baudRate;
}

/// <summary>
/// Set Receiver Channel Filter Bandwidth
/// </summary>
/// <param name="bandwidth">812, 650, 541, 464, 406, 325, 270, 232, 203, 162, 135, 116, 102, 81, 68, 58</param>
void set_rx_channel_bandwidth(spi_device_handle_t spi, uint16_t bandwidth)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	uint8_t chanbwExponent = 3, chanbwMantissa = 3;

	for (uint8_t e_index = 0; e_index < 4; e_index++)
	{
		for (uint8_t m_index = 0; m_index < 4; m_index++)
		{
			if (bandwidth >= chanBW_limits[4 * e_index + m_index])
			{
				chanbwExponent = e_index;
				chanbwMantissa = m_index;
				e_index = 4;
				m_index = 4;
				break;
			}
		}
	}

	mdmcfg4_t mdmcfg4 = {
		.reg = cc1101_read_config_register(spi, CC1101_MDMCFG4)};
	mdmcfg4.chanbw_e = chanbwExponent;
	mdmcfg4.chanbw_m = chanbwMantissa;

	cc1101_write_register(spi, CC1101_MDMCFG4, mdmcfg4.reg);
	ESP_LOGD(TAG, "Set Rx Band Width: %d. E: %d, M: %d", bandwidth, chanbwExponent, chanbwMantissa);
}

uint16_t get_rx_channel_bandwidth(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mdmcfg4_t mdmcfg4 = {
		.reg = cc1101_read_config_register(spi, CC1101_MDMCFG4)};
	uint8_t bandWidthExponent = mdmcfg4.chanbw_e;
	uint8_t bandWidthMantissa = mdmcfg4.chanbw_m;
	//ESP_LOGD(TAG, "Get Band Width: %d. E: %d, M: %d   dr %d", mdmcfg4.reg, mdmcfg4.chanbw_e, mdmcfg4.chanbw_m, mdmcfg4.drate_e);
	double_t bandWidth = 1000.0 * CC1101_CLOCK_FREQUENCY / (8.0 * (4 + bandWidthMantissa) * pow(2, bandWidthExponent));
	return bandWidth;
}

void set_modulation_format(spi_device_handle_t spi, mod_format_t modFormat)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mdmcfg2_t mdmcfg2 = {
		.reg = cc1101_read_config_register(spi, CC1101_MDMCFG2)};
	mdmcfg2.mod_format = modFormat;

	cc1101_write_register(spi, CC1101_MDMCFG2, mdmcfg2.reg);
	ESP_LOGD(TAG, "Set Modulation Format: %d", modFormat);
}

mod_format_t get_modulation_format(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mdmcfg2_t mdmcfg2 = {
		.reg = cc1101_read_config_register(spi, CC1101_MDMCFG2)};
	return mdmcfg2.mod_format;
}

void set_manchester_encoding(spi_device_handle_t spi, uint8_t manchester_en)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mdmcfg2_t mdmcfg2 = {
		.reg = cc1101_read_config_register(spi, CC1101_MDMCFG2)};
	mdmcfg2.manchester_en = manchester_en;
	cc1101_write_register(spi, CC1101_MDMCFG2, mdmcfg2.reg);
	ESP_LOGD(TAG, "Set Manchester Encoding: %d.", mdmcfg2.manchester_en);
}

uint8_t get_manchester_encoding(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mdmcfg2_t mdmcfg2 = {
		.reg = cc1101_read_config_register(spi, CC1101_MDMCFG2)};
	return mdmcfg2.manchester_en;
}

void set_sync_mode(spi_device_handle_t spi, sync_mode_t sync_mode)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mdmcfg2_t mdmcfg2 = {
		.reg = cc1101_read_config_register(spi, CC1101_MDMCFG2)};
	mdmcfg2.sync_mode = sync_mode;
	cc1101_write_register(spi, CC1101_MDMCFG2, mdmcfg2.reg);
	ESP_LOGD(TAG, "Set Sync Mode: %d.", mdmcfg2.sync_mode);
}

sync_mode_t get_sync_mode(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mdmcfg2_t mdmcfg2 = {
		.reg = cc1101_read_config_register(spi, CC1101_MDMCFG2)};
	return mdmcfg2.sync_mode;
}

void set_packet_length(spi_device_handle_t spi, uint8_t packet_length)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	cc1101_write_register(spi, CC1101_PKTLEN, packet_length);
	ESP_LOGD(TAG, "Set Packet length: %d.", packet_length);
}

uint8_t get_packet_length(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktlen_t pktlen = {
		.reg = cc1101_read_config_register(spi, CC1101_PKTLEN)};
	return pktlen.reg;
}

void set_crc_enable(spi_device_handle_t spi, uint8_t crc_en)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktctrl0_t pktctrl0 = {
		.reg = cc1101_read_config_register(spi, CC1101_PKTCTRL0)};
	pktctrl0.crc_en = crc_en;
	cc1101_write_register(spi, CC1101_PKTCTRL0, pktctrl0.reg);
	ESP_LOGD(TAG, "Set CRC check: %d.", pktctrl0.crc_en);
}

uint8_t get_crc_enable(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktctrl0_t pktctrl0 = {
		.reg = cc1101_read_config_register(spi, CC1101_PKTCTRL0)};
	return pktctrl0.crc_en;
}

void set_white_data(spi_device_handle_t spi, uint8_t white_data)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktctrl0_t pktctrl0 = {
		.reg = cc1101_read_config_register(spi, CC1101_PKTCTRL0)};
	pktctrl0.white_data = white_data;
	cc1101_write_register(spi, CC1101_PKTCTRL0, pktctrl0.reg);
	ESP_LOGD(TAG, "Set Whitedata: %d.", pktctrl0.white_data);
}

uint8_t get_white_data(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktctrl0_t pktctrl0 = {
		.reg = cc1101_read_config_register(spi, CC1101_PKTCTRL0)};
	return pktctrl0.white_data;
}

void set_pkt_format(spi_device_handle_t spi, pkt_format_t pkt_format)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktctrl0_t pktctrl0 = {
		.reg = cc1101_read_config_register(spi, CC1101_PKTCTRL0)};
	pktctrl0.pkt_format = pkt_format;
	cc1101_write_register(spi, CC1101_PKTCTRL0, pktctrl0.reg);
	ESP_LOGD(TAG, "Set Rx, Tx mode configuration: %d.", pktctrl0.pkt_format);
}

pkt_format_t get_pkt_format(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktctrl0_t pktctrl0 = {
		.reg = cc1101_read_config_register(spi, CC1101_PKTCTRL0)};
	return pktctrl0.pkt_format;
}

void set_length_config(spi_device_handle_t spi, pck_length_config_t pck_length_config)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktctrl0_t pktctrl0 = {
		.reg = cc1101_read_config_register(spi, CC1101_PKTCTRL0)};
	pktctrl0.pck_length_config = pck_length_config;
	cc1101_write_register(spi, CC1101_PKTCTRL0, pktctrl0.reg);
	ESP_LOGD(TAG, "Set Packet Length Config: %d.", pktctrl0.pck_length_config);
}

pck_length_config_t get_length_config(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktctrl0_t pktctrl0 = {
		.reg = cc1101_read_config_register(spi, CC1101_PKTCTRL0)};
	return pktctrl0.pck_length_config;
}

void set_txoff_mode(spi_device_handle_t spi, txoff_mode_t txoff_mode)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mcsm1_t mcsm1 = {
		.reg = cc1101_read_config_register(spi, CC1101_MCSM1)};
	mcsm1.txoff_mode = txoff_mode;

	cc1101_write_register(spi, CC1101_MCSM1, mcsm1.reg);
	ESP_LOGD(TAG, "Set TxOff Mode %d.", mcsm1.txoff_mode);
}

txoff_mode_t get_txoff_mode(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mcsm1_t mcsm1 = {
		.reg = cc1101_read_config_register(spi, CC1101_MCSM1)};
	return mcsm1.txoff_mode;
}

void set_rxoff_mode(spi_device_handle_t spi, rxoff_mode_t rxoff_mode)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mcsm1_t mcsm1 = {
		.reg = cc1101_read_config_register(spi, CC1101_MCSM1)};
	mcsm1.rxoff_mode = rxoff_mode;

	cc1101_write_register(spi, CC1101_MCSM1, mcsm1.reg);
	ESP_LOGD(TAG, "Set RxOff Mode %d.", mcsm1.rxoff_mode);
}

rxoff_mode_t get_rxoff_mode(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mcsm1_t mcsm1 = {
		.reg = cc1101_read_config_register(spi, CC1101_MCSM1)};
	return mcsm1.rxoff_mode;
}

void set_cca_mode(spi_device_handle_t spi, cca_mode_t cca_mode)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mcsm1_t mcsm1 = {
		.reg = cc1101_read_config_register(spi, CC1101_MCSM1)};
	mcsm1.cca_mode = cca_mode;

	cc1101_write_register(spi, CC1101_MCSM1, mcsm1.reg);
	ESP_LOGD(TAG, "Set CCA Mode %d.", mcsm1.cca_mode);
}

cca_mode_t get_cca_mode(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mcsm1_t mcsm1 = {
		.reg = cc1101_read_config_register(spi, CC1101_MCSM1)};
	return mcsm1.cca_mode;
}

void set_sync_word(spi_device_handle_t spi, sync_word_t sync_word)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	cc1101_write_register(spi, CC1101_SYNC0, sync_word.sync0);
	cc1101_write_register(spi, CC1101_SYNC1, sync_word.sync1);
	ESP_LOGD(TAG, "Set Sync Word %d.", sync_word.sync);
}

sync_word_t get_sync_word(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	sync_word_t sync_word = {
		.sync0 = cc1101_read_config_register(spi, CC1101_SYNC0),
		.sync1 = cc1101_read_config_register(spi, CC1101_SYNC1)};
	return sync_word;
}

void set_forward_error_correction(spi_device_handle_t spi, uint8_t fec_en)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mdmcfg1_t mdmcfg1 = {
		.reg = cc1101_read_config_register(spi, CC1101_MDMCFG1)};
	mdmcfg1.fec_en = fec_en;

	cc1101_write_register(spi, CC1101_MDMCFG1, mdmcfg1.reg);
	ESP_LOGD(TAG, "Set Forward Error Correction %d.", mdmcfg1.fec_en);
}

uint8_t get_forward_error_correction(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mdmcfg1_t mdmcfg1 = {
		.reg = cc1101_read_config_register(spi, CC1101_MDMCFG1)};
	return mdmcfg1.fec_en;
}

void set_num_preamble(spi_device_handle_t spi, num_preamble_t num_preamble)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mdmcfg1_t mdmcfg1 = {
		.reg = cc1101_read_config_register(spi, CC1101_MDMCFG1)};
	mdmcfg1.num_preamble = num_preamble;

	cc1101_write_register(spi, CC1101_MDMCFG1, mdmcfg1.reg);
	ESP_LOGD(TAG, "Set Preamble bytes %d.", mdmcfg1.num_preamble);
}

num_preamble_t get_num_preamble(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mdmcfg1_t mdmcfg1 = {
		.reg = cc1101_read_config_register(spi, CC1101_MDMCFG1)};
	return mdmcfg1.num_preamble;
}

void set_deviation_frequency(spi_device_handle_t spi, double frequencyInkHz)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	deviatn_t deviatn = {
		.deviation_e = 0,
		.deviation_m = 0};

	for (int tempExponent = 0; tempExponent < 4; tempExponent++)
	{
		double_t tempMantissa = frequencyInkHz * 1000.0 * pow(2, 17) / (pow(2, tempExponent) * (CC1101_CLOCK_FREQUENCY * 1000000.0)) - 8;
		if (tempMantissa < 8)
		{
			deviatn.deviation_e = tempExponent;
			deviatn.deviation_m = tempMantissa < 0 ? 0 : tempMantissa;
			break;
		}
	}

	cc1101_write_register(spi, CC1101_DEVIATN, deviatn.reg);

	ESP_LOGD(TAG, "Set Deviation Frequency %f. E: %d, M: %d ", frequencyInkHz, deviatn.deviation_e, deviatn.deviation_m);
}

double get_deviation_frequency(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	deviatn_t deviatn = {
		.reg = cc1101_read_config_register(spi, CC1101_DEVIATN)};

	double_t deviation = 1000.0 * CC1101_CLOCK_FREQUENCY * (8 + deviatn.deviation_m) * pow(2, deviatn.deviation_e) / pow(2, 17);
	return deviation;
}

void set_controls_address_check(spi_device_handle_t spi, adr_chk_t adr_chk)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktctrl1_t pktctrl1 = {
		.reg = cc1101_read_config_register(spi, CC1101_PKTCTRL1)};
	pktctrl1.adr_chk = adr_chk;

	cc1101_write_register(spi, CC1101_PKTCTRL1, pktctrl1.reg);
	ESP_LOGD(TAG, "Set Address Check: %d.", pktctrl1.adr_chk);
}

adr_chk_t get_controls_address_check(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktctrl1_t pktctrl1 = {
		.reg = cc1101_read_config_register(spi, CC1101_PKTCTRL1)};
	return pktctrl1.adr_chk;
}

void set_append_status(spi_device_handle_t spi, uint8_t append_status)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktctrl1_t pktctrl1 = {
		.reg = cc1101_read_config_register(spi, CC1101_PKTCTRL1)};
	pktctrl1.append_status = append_status;

	cc1101_write_register(spi, CC1101_PKTCTRL1, pktctrl1.reg);
	ESP_LOGD(TAG, "Set Append Status: %d.", pktctrl1.append_status);
}

uint8_t get_append_status(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktctrl1_t pktctrl1 = {
		.reg = cc1101_read_config_register(spi, CC1101_PKTCTRL1)};

	return pktctrl1.append_status;
}

void set_preamble_quality_threshold(spi_device_handle_t spi, uint8_t pqt)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktctrl1_t pktctrl1 = {
		.reg = cc1101_read_config_register(spi, CC1101_PKTCTRL1)};
	pktctrl1.pqt = pqt;

	cc1101_write_register(spi, CC1101_PKTCTRL1, pktctrl1.reg);
	ESP_LOGD(TAG, "Set Preamble Quality Threshold: %d.", pktctrl1.pqt);
}

uint8_t get_preamble_quality_threshold(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktctrl1_t pktctrl1 = {
		.reg = cc1101_read_config_register(spi, CC1101_PKTCTRL1)};

	return pktctrl1.pqt;
}

void set_crc_autoflush(spi_device_handle_t spi, uint8_t crc_autoflush)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktctrl1_t pktctrl1 = {
		.reg = cc1101_read_config_register(spi, CC1101_PKTCTRL1)};
	pktctrl1.crc_autoflush = crc_autoflush;

	cc1101_write_register(spi, CC1101_PKTCTRL1, pktctrl1.reg);
	ESP_LOGD(TAG, "Set CRC Autoflush: %d.", pktctrl1.crc_autoflush);
}

uint8_t get_crc_autoflush(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktctrl1_t pktctrl1 = {
		.reg = cc1101_read_config_register(spi, CC1101_PKTCTRL1)};

	return pktctrl1.crc_autoflush;
}

void set_device_address(spi_device_handle_t spi, uint8_t device_addr)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	addr_t addr = {
		.device_addr = device_addr};

	cc1101_write_register(spi, CC1101_ADDR, addr.reg);
	ESP_LOGD(TAG, "Set Device Address: %d.", addr.device_addr);
}

uint8_t get_device_address(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	addr_t addr = {
		.reg = cc1101_read_config_register(spi, CC1101_ADDR)};

	return addr.device_addr;
}

void set_channel_number(spi_device_handle_t spi, uint8_t chan)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	channr_t channr = {
		.chan = chan};

	cc1101_write_register(spi, CC1101_CHANNR, channr.reg);
	ESP_LOGD(TAG, "Set Channel Number: %d.", channr.chan);
}

uint8_t get_channel_number(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	channr_t channr = {
		.reg = cc1101_read_config_register(spi, CC1101_CHANNR)};

	return channr.chan;
}

void set_rx_attenuation(spi_device_handle_t spi, close_in_rx_t close_in_rx)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	fifothr_t fifothr = {
		.reg = cc1101_read_config_register(spi, CC1101_FIFOTHR)};
	fifothr.close_in_rx = close_in_rx;

	cc1101_write_register(spi, CC1101_FIFOTHR, fifothr.reg);
	ESP_LOGD(TAG, "Set RX Attenuationr: %d.", fifothr.close_in_rx);
}

uint8_t get_rx_attenuation(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	fifothr_t fifothr = {
		.reg = cc1101_read_config_register(spi, CC1101_FIFOTHR)};

	return fifothr.close_in_rx;
}

void set_pa_power_setting(spi_device_handle_t spi, uint8_t pa_power)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	frend0_t frend0 = {
		.reg = cc1101_read_config_register(spi, CC1101_FREND0)};
	frend0.pa_power = pa_power;

	cc1101_write_register(spi, CC1101_FREND0, frend0.reg);
	ESP_LOGD(TAG, "Set PA power setting: %d.", frend0.pa_power);
}

uint8_t get_pa_power_setting(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	frend0_t frend0 = {
		.reg = cc1101_read_config_register(spi, CC1101_FREND0)};

	return frend0.pa_power;
}

void set_filter_length(spi_device_handle_t spi, filter_length_t filter_length)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	agcctrl0_t agcctrl0 = {
		.reg = cc1101_read_config_register(spi, CC1101_AGCCTRL0)};
	agcctrl0.filter_length = filter_length;

	cc1101_write_register(spi, CC1101_AGCCTRL0, agcctrl0.reg);
	ESP_LOGD(TAG, "Set Filter Length: %d.", agcctrl0.filter_length);
}

uint8_t get_filter_length(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	agcctrl0_t agcctrl0 = {
		.reg = cc1101_read_config_register(spi, CC1101_AGCCTRL0)};

	return agcctrl0.filter_length;
}

void set_hysteresis_level(spi_device_handle_t spi, hyst_level_t hyst_level)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	agcctrl0_t agcctrl0 = {
		.reg = cc1101_read_config_register(spi, CC1101_AGCCTRL0)};
	agcctrl0.hyst_level = hyst_level;

	cc1101_write_register(spi, CC1101_AGCCTRL0, agcctrl0.reg);
	ESP_LOGD(TAG, "Set Filter Length: %d.", agcctrl0.hyst_level);
}

hyst_level_t get_hysteresis_level(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	agcctrl0_t agcctrl0 = {
		.reg = cc1101_read_config_register(spi, CC1101_AGCCTRL0)};

	return agcctrl0.hyst_level;
}

void set_magn_target(spi_device_handle_t spi, magn_target_t magn_target)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	agcctrl2_t agcctrl2 = {
		.reg = cc1101_read_config_register(spi, CC1101_AGCCTRL2)};
	agcctrl2.magn_target = magn_target;

	cc1101_write_register(spi, CC1101_AGCCTRL2, agcctrl2.reg);
	ESP_LOGD(TAG, "Set Filter Length: %d.", agcctrl2.magn_target);
}

magn_target_t get_magn_target(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	agcctrl2_t agcctrl2 = {
		.reg = cc1101_read_config_register(spi, CC1101_AGCCTRL2)};

	return agcctrl2.magn_target;
}

uint8_t get_rssi(spi_device_handle_t spi)
{
	return cc1101_read_status_register(spi, CC1101_RSSI);
}

float get_rssi_dbm(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	uint8_t rssi = get_rssi(spi);
	if (rssi < 128)
	{
		return (rssi / 2) - 74;
	}
	else
	{
		return ((rssi - 256) / 2) - 74;
	}
}

uint8_t get_tx_fifo_info(spi_device_handle_t spi)
{
	return cc1101_read_status_register(spi, CC1101_TXBYTES) & 0b01111111;
}

void init(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	foreach (cc1101_init_cmd_t *cfg, config_register)
	{
		cc1101_write_register(spi, cfg->cmd, cfg->data);
	}
	ESP_LOGD(TAG, "Set init settings.");
}

void set_pa_table(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);
	cc1101_write_burst_register(spi, CC1101_PATABLE, pa_table_power, sizeof(pa_table_power));
	ESP_LOGD(TAG, "Set PA power table.");
}

void get_pa_table(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	uint8_t *pa_table = cc1101_read_burst_register(spi, CC1101_PATABLE, sizeof(pa_table_power));

	for (uint8_t i = 0; i < sizeof(pa_table_power); i++)
	{
		ESP_LOGD(TAG, "%x - %x", i, *(pa_table + i));
	}

	free(pa_table);
	ESP_LOGD(TAG, "Get PA power table.");
}

void reset(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	cc1101_command_strobe(spi, CC1101_SRES);
}

void set_idle_state(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	cc1101_command_strobe(spi, CC1101_SIDLE);
	ESP_LOGD(TAG, "Set Idle state");
}

void set_flush_tx(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	cc1101_command_strobe(spi, CC1101_SFTX);
	ESP_LOGD(TAG, "Set flush TX");
}

void set_flush_rx(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	cc1101_command_strobe(spi, CC1101_SFRX);
	ESP_LOGD(TAG, "Set flush RX");
}

void set_tx_state(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	cc1101_command_strobe(spi, CC1101_STX);
	ESP_LOGD(TAG, "Set Tx state");
}

void set_rx_state(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	cc1101_command_strobe(spi, CC1101_SRX);
	ESP_LOGD(TAG, "Set Rx state");
}

void set_calibrate_state(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	cc1101_command_strobe(spi, CC1101_SCAL);
	ESP_LOGD(TAG, "Set Ralibrate state");
}

void set_command_state(spi_device_handle_t spi, CC1101_command_strobe_t command_strobe)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	cc1101_command_strobe(spi, command_strobe);
	ESP_LOGD(TAG, "Set Command: %d", command_strobe);
}

CC1101_marc_state_t get_marc_state(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);
	CC1101_marc_state_t state = cc1101_read_status_register(spi, CC1101_MARCSTATE) & 0x1F;
	ESP_LOGD(TAG, "Marc State is %d", state);

	return state;
}

void send_data(spi_device_handle_t spi)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	DRAM_ATTR static uint8_t data1[] = {0x00, 0x55, 0x55, 0xAA, 0xAA};
	DRAM_ATTR static uint8_t data2[] = {0x00, 0xAA, 0xAA, 0x55, 0x55};

	cc1101_write_burst_register(spi, CC1101_TXFIFO, data1, 5);
	set_command_state(spi, CC1101_STX);

	while (get_marc_state(spi) != CC1101_STATE_IDLE)
	{
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
	set_flush_tx(spi);

	cc1101_write_burst_register(spi, CC1101_TXFIFO, data2, 5);
	set_command_state(spi, CC1101_STX);

	while (get_marc_state(spi) != CC1101_STATE_IDLE)
	{
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}

	set_flush_tx(spi);
}