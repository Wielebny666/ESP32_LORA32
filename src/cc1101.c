/*
 * cc1101.c
 *
 *  Created on: 13 kwi 2019
 *      Author: kurza
 */
#ifdef CC1101
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"

#include "cc1101_regs.h"
#include "cc1101_defs.h"
#include "foreach.h"

#include "hardware.h"

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
	{CC1101_PKTCTRL0, 0x30},
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

DRAM_ATTR uint8_t pa_table_power[] = {0x00, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
DRAM_ATTR uint16_t chanBW_limits[] = {812, 650, 541, 464, 406, 325, 270, 232, 203, 162, 135, 116, 102, 81, 68, 58};

static spi_device_handle_t cc1101_spi;

uint16_t convert_hex_to_manchester(uint8_t hex);

cc1101_t cc1101;

void cc1101_spi_init()
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	spi_bus_config_t buscfg = {
		.miso_io_num = RADIO_MISO,
		.mosi_io_num = RADIO_MOSI,
		.sclk_io_num = RADIO_SCLK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1};

	spi_device_interface_config_t devcfg = {
		.clock_speed_hz = SPI_MASTER_FREQ_10M / 2, //Clock out at 10 MHz
		.mode = 0,								   //SPI mode 0
		.spics_io_num = RADIO_NSS,				   //CS pin
		.queue_size = 1, //We want to be able to queue 7 transactions at a time
		.command_bits = 8};

	//Initialize the SPI bus
	ESP_ERROR_CHECK(spi_bus_initialize(SPI_RADIO, &buscfg, 0));
	//Attach the Device to the SPI bus
	ESP_ERROR_CHECK(spi_bus_add_device(SPI_RADIO, &devcfg, &cc1101_spi));
}

void cc1101_write_register(uint8_t cmd, const uint8_t byte)
{
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.length = 8;
	t.cmd = cmd;
	t.tx_buffer = &byte;

	ESP_ERROR_CHECK(spi_device_polling_transmit(cc1101_spi, &t));
}

void cc1101_write_burst_register(uint8_t cmd, const uint8_t *bytes, const uint8_t length)
{
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.length = length * 8;
	t.cmd = cmd | CC1101_WRITE_BURST;
	t.tx_buffer = bytes;

	ESP_ERROR_CHECK(spi_device_transmit(cc1101_spi, &t));
}

uint8_t cc1101_read_status_register(uint8_t cmd)
{
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.length = 8;
	t.cmd = cmd | CC1101_READ_BURST;
	t.flags = SPI_TRANS_USE_RXDATA;

	ESP_ERROR_CHECK(spi_device_polling_transmit(cc1101_spi, &t));

	return t.rx_data[0];
}

uint8_t cc1101_read_config_register(uint8_t cmd)
{
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.length = 8;
	t.cmd = cmd | CC1101_READ_SINGLE;
	t.flags = SPI_TRANS_USE_RXDATA;

	ESP_ERROR_CHECK(spi_device_polling_transmit(cc1101_spi, &t));

	return t.rx_data[0];
}

void cc1101_read_burst_register(uint8_t cmd, uint8_t *bytes, uint8_t length)
{
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.length = length * 8;
	t.cmd = cmd | CC1101_READ_BURST;
	t.rx_buffer = bytes;

	ESP_ERROR_CHECK(spi_device_polling_transmit(cc1101_spi, &t));
}

void cc1101_command_strobe(CC1101_command_strobe_t strobe)
{
	spi_transaction_t t;
	memset(&t, 0, sizeof(t));
	t.length = 0;
	t.cmd = strobe;

	esp_err_t ret = spi_device_polling_transmit(cc1101_spi, &t);
	ESP_ERROR_CHECK(ret);
}

void cc1101_init_config(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	foreach (cc1101_init_cmd_t *cfg, config_register)
	{
		cc1101_write_register(cfg->cmd, cfg->data);
	}
	ESP_LOGD(TAG, "Set init settings.");
}

void cc1101_read_config(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);
	uint8_t test = cc1101_read_config_register(0);
	ESP_LOGD(TAG, "test %x", test);
	cc1101_read_burst_register(0, (uint8_t *)&cc1101, sizeof(cc1101));
}

void cc1101_print_config()
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	ESP_LOGD(TAG, "%s %#04x", "iocfg2  ", cc1101.iocfg2.reg);
	ESP_LOGD(TAG, "%s %#04x", "iocfg1  ", cc1101.iocfg1.reg);
	ESP_LOGD(TAG, "%s %#04x", "iocfg0  ", cc1101.iocfg0.reg);
	ESP_LOGD(TAG, "%s %#04x", "fifothr ", cc1101.fifothr.reg);
	ESP_LOGD(TAG, "%s %#04x", "sync1   ", cc1101.sync1.sync_15_8);
	ESP_LOGD(TAG, "%s %#04x", "sync0   ", cc1101.sync0.sync_7_0);
	ESP_LOGD(TAG, "%s %#04x", "pktlen  ", cc1101.pktlen.pktlen_length);
	ESP_LOGD(TAG, "%s %#04x", "pktctrl1", cc1101.pktctrl1.reg);
	ESP_LOGD(TAG, "%s %#04x", "pktctrl0", cc1101.pktctrl0.reg);
	ESP_LOGD(TAG, "%s %#04x", "addr    ", cc1101.addr.device_addr);
	ESP_LOGD(TAG, "%s %#04x", "channr  ", cc1101.channr.chan);
	ESP_LOGD(TAG, "%s %#04x", "fsctrl1 ", cc1101.fsctrl1.freq_if);
	ESP_LOGD(TAG, "%s %#04x", "fsctrl0 ", cc1101.fsctrl0.freqoff);
	ESP_LOGD(TAG, "%s %#04x", "freq2   ", cc1101.freq2.reg);
	ESP_LOGD(TAG, "%s %#04x", "freq1   ", cc1101.freq1.freq_8_15);
	ESP_LOGD(TAG, "%s %#04x", "freq0   ", cc1101.freq0.freq_0_7);
	ESP_LOGD(TAG, "%s %#04x", "mdmcfg4 ", cc1101.mdmcfg4.reg);
	ESP_LOGD(TAG, "%s %#04x", "mdmcfg3 ", cc1101.mdmcfg3.drate_m);
	ESP_LOGD(TAG, "%s %#04x", "mdmcfg2 ", cc1101.mdmcfg2.reg);
	ESP_LOGD(TAG, "%s %#04x", "mdmcfg1 ", cc1101.mdmcfg1.reg);
	ESP_LOGD(TAG, "%s %#04x", "mdmcfg0 ", cc1101.mdmcfg0.chanspc_m);
	ESP_LOGD(TAG, "%s %#04x", "deviatn ", cc1101.deviatn.reg);
	ESP_LOGD(TAG, "%s %#04x", "mcsm2   ", cc1101.mcsm2.mcsm2);
	ESP_LOGD(TAG, "%s %#04x", "mcsm1   ", cc1101.mcsm1.reg);
	ESP_LOGD(TAG, "%s %#04x", "mcsm0   ", cc1101.mcsm0.reg);
	ESP_LOGD(TAG, "%s %#04x", "foccfg  ", cc1101.foccfg.foccfg);
	ESP_LOGD(TAG, "%s %#04x", "bscfg   ", cc1101.bscfg.bscfg);
	ESP_LOGD(TAG, "%s %#04x", "agcctrl2", cc1101.agcctrl2.reg);
	ESP_LOGD(TAG, "%s %#04x", "agcctrl1", cc1101.agcctrl1.reg);
	ESP_LOGD(TAG, "%s %#04x", "agcctrl0", cc1101.agcctrl0.reg);
}

void cc1101_set_carrier_frequency(double frequencyInMHz)
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

	cc1101_write_register(CC1101_FREQ2, firstByteValue);
	cc1101_write_register(CC1101_FREQ1, secondByteValue);
	cc1101_write_register(CC1101_FREQ0, thirdByteValue);

	ESP_LOGD(TAG, "Set Carrier Frequency: %f. 1: %d, 2: %d, 3: %d", frequencyInMHz, firstByteValue, secondByteValue, thirdByteValue);
}

double cc1101_get_carrier_frequency(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	double_t firstRegisterByte = cc1101_read_config_register(CC1101_FREQ2);
	double_t secondRegisterByte = cc1101_read_config_register(CC1101_FREQ1);
	double_t thirdRegisterByte = cc1101_read_config_register(CC1101_FREQ0);

	firstRegisterByte = firstRegisterByte * 26;
	secondRegisterByte = secondRegisterByte / 255 * 26;
	thirdRegisterByte = thirdRegisterByte / 255 / 255 * 26;

	return firstRegisterByte + +secondRegisterByte + +thirdRegisterByte;
}

void cc1101_set_baud_rate(double baud_rate_in_khz)
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
		.reg = cc1101_read_config_register(CC1101_MDMCFG4)};
	mdmcfg4.drate_e = baudRateExponent;

	cc1101_write_register(CC1101_MDMCFG4, mdmcfg4.reg);
	cc1101_write_register(CC1101_MDMCFG3, baudRateMantissa);
	ESP_LOGD(TAG, "Set Transmission Baud rate: %f. E: %d, M: %d", baud_rate_in_khz, baudRateExponent, baudRateMantissa);
}

double cc1101_get_baud_rate(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mdmcfg4_t mdmcfg4 = {
		.reg = cc1101_read_config_register(CC1101_MDMCFG4)};
	uint8_t baudRateExponent = mdmcfg4.drate_e;
	uint8_t baudRateMantissa = cc1101_read_config_register(CC1101_MDMCFG3);

	double_t baudRate = 1000.0 * CC1101_CLOCK_FREQUENCY * (256 + baudRateMantissa) * pow(2, baudRateExponent) / pow(2, 28);

	return baudRate;
}

/// <summary>
/// Set Receiver Channel Filter Bandwidth
/// </summary>
/// <param name="bandwidth">812, 650, 541, 464, 406, 325, 270, 232, 203, 162, 135, 116, 102, 81, 68, 58</param>
void cc1101_set_rx_channel_bandwidth(uint16_t bandwidth)
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
		.reg = cc1101_read_config_register(CC1101_MDMCFG4)};
	mdmcfg4.chanbw_e = chanbwExponent;
	mdmcfg4.chanbw_m = chanbwMantissa;

	cc1101_write_register(CC1101_MDMCFG4, mdmcfg4.reg);
	ESP_LOGD(TAG, "Set Rx Band Width: %d. E: %d, M: %d", bandwidth, chanbwExponent, chanbwMantissa);
}

uint16_t cc1101_get_rx_channel_bandwidth(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mdmcfg4_t mdmcfg4 = {
		.reg = cc1101_read_config_register(CC1101_MDMCFG4)};
	uint8_t bandWidthExponent = mdmcfg4.chanbw_e;
	uint8_t bandWidthMantissa = mdmcfg4.chanbw_m;
	//ESP_LOGD(TAG, "Get Band Width: %d. E: %d, M: %d   dr %d", mdmcfg4.reg, mdmcfg4.chanbw_e, mdmcfg4.chanbw_m, mdmcfg4.drate_e);
	double_t bandWidth = 1000.0 * CC1101_CLOCK_FREQUENCY / (8.0 * (4 + bandWidthMantissa) * pow(2, bandWidthExponent));
	return bandWidth;
}

void cc1101_set_modulation_format(mod_format_t modFormat)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mdmcfg2_t mdmcfg2 = {
		.reg = cc1101_read_config_register(CC1101_MDMCFG2)};
	mdmcfg2.mod_format = modFormat;

	cc1101_write_register(CC1101_MDMCFG2, mdmcfg2.reg);
	ESP_LOGD(TAG, "Set Modulation Format: %d", modFormat);
}

mod_format_t cc1101_get_modulation_format(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mdmcfg2_t mdmcfg2 = {
		.reg = cc1101_read_config_register(CC1101_MDMCFG2)};
	return mdmcfg2.mod_format;
}

void cc1101_set_manchester_encoding(uint8_t manchester_en)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mdmcfg2_t mdmcfg2 = {
		.reg = cc1101_read_config_register(CC1101_MDMCFG2)};
	mdmcfg2.manchester_en = manchester_en;
	cc1101_write_register(CC1101_MDMCFG2, mdmcfg2.reg);
	ESP_LOGD(TAG, "Set Manchester Encoding: %d.", mdmcfg2.manchester_en);
}

uint8_t cc1101_get_manchester_encoding(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mdmcfg2_t mdmcfg2 = {
		.reg = cc1101_read_config_register(CC1101_MDMCFG2)};
	return mdmcfg2.manchester_en;
}

void cc1101_set_sync_mode(sync_mode_t sync_mode)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mdmcfg2_t mdmcfg2 = {
		.reg = cc1101_read_config_register(CC1101_MDMCFG2)};
	mdmcfg2.sync_mode = sync_mode;
	cc1101_write_register(CC1101_MDMCFG2, mdmcfg2.reg);
	ESP_LOGD(TAG, "Set Sync Mode: %d.", mdmcfg2.sync_mode);
}

sync_mode_t cc1101_get_sync_mode(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mdmcfg2_t mdmcfg2 = {
		.reg = cc1101_read_config_register(CC1101_MDMCFG2)};
	return mdmcfg2.sync_mode;
}

void cc1101_set_packet_length(uint8_t packet_length)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	cc1101_write_register(CC1101_PKTLEN, packet_length);
	ESP_LOGD(TAG, "Set Packet length: %d.", packet_length);
}

uint8_t cc1101_get_packet_length(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktlen_t pktlen = {
		.pktlen_length = cc1101_read_config_register(CC1101_PKTLEN)};
	return pktlen.pktlen_length;
}

void cc1101_set_crc_enable(uint8_t crc_en)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktctrl0_t pktctrl0 = {
		.reg = cc1101_read_config_register(CC1101_PKTCTRL0)};
	pktctrl0.crc_en = crc_en;
	cc1101_write_register(CC1101_PKTCTRL0, pktctrl0.reg);
	ESP_LOGD(TAG, "Set CRC check: %d.", pktctrl0.crc_en);
}

uint8_t cc1101_get_crc_enable(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktctrl0_t pktctrl0 = {
		.reg = cc1101_read_config_register(CC1101_PKTCTRL0)};
	return pktctrl0.crc_en;
}

void cc1101_set_white_data(uint8_t white_data)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktctrl0_t pktctrl0 = {
		.reg = cc1101_read_config_register(CC1101_PKTCTRL0)};
	pktctrl0.white_data = white_data;
	cc1101_write_register(CC1101_PKTCTRL0, pktctrl0.reg);
	ESP_LOGD(TAG, "Set Whitedata: %d.", pktctrl0.white_data);
}

uint8_t cc1101_get_white_data(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktctrl0_t pktctrl0 = {
		.reg = cc1101_read_config_register(CC1101_PKTCTRL0)};
	return pktctrl0.white_data;
}

void cc1101_set_pkt_format(pkt_format_t pkt_format)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktctrl0_t pktctrl0 = {
		.reg = cc1101_read_config_register(CC1101_PKTCTRL0)};
	pktctrl0.pkt_format = pkt_format;
	cc1101_write_register(CC1101_PKTCTRL0, pktctrl0.reg);
	ESP_LOGD(TAG, "Set Rx, Tx mode configuration: %d.", pktctrl0.pkt_format);
}

pkt_format_t cc1101_get_pkt_format(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktctrl0_t pktctrl0 = {
		.reg = cc1101_read_config_register(CC1101_PKTCTRL0)};
	return pktctrl0.pkt_format;
}

void cc1101_set_length_config(pck_length_config_t pck_length_config)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktctrl0_t pktctrl0 = {
		.reg = cc1101_read_config_register(CC1101_PKTCTRL0)};
	pktctrl0.pck_length_config = pck_length_config;
	cc1101_write_register(CC1101_PKTCTRL0, pktctrl0.reg);
	ESP_LOGD(TAG, "Set Packet Length Config: %d.", pktctrl0.pck_length_config);
}

pck_length_config_t cc1101_get_length_config(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktctrl0_t pktctrl0 = {
		.reg = cc1101_read_config_register(CC1101_PKTCTRL0)};
	return pktctrl0.pck_length_config;
}

void cc1101_set_txoff_mode(txoff_mode_t txoff_mode)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mcsm1_t mcsm1 = {
		.reg = cc1101_read_config_register(CC1101_MCSM1)};
	mcsm1.txoff_mode = txoff_mode;

	cc1101_write_register(CC1101_MCSM1, mcsm1.reg);
	ESP_LOGD(TAG, "Set TxOff Mode %d.", mcsm1.txoff_mode);
}

txoff_mode_t cc1101_get_txoff_mode(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mcsm1_t mcsm1 = {
		.reg = cc1101_read_config_register(CC1101_MCSM1)};
	return mcsm1.txoff_mode;
}

void cc1101_set_rxoff_mode(rxoff_mode_t rxoff_mode)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mcsm1_t mcsm1 = {
		.reg = cc1101_read_config_register(CC1101_MCSM1)};
	mcsm1.rxoff_mode = rxoff_mode;

	cc1101_write_register(CC1101_MCSM1, mcsm1.reg);
	ESP_LOGD(TAG, "Set RxOff Mode %d.", mcsm1.rxoff_mode);
}

rxoff_mode_t cc1101_get_rxoff_mode(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mcsm1_t mcsm1 = {
		.reg = cc1101_read_config_register(CC1101_MCSM1)};
	return mcsm1.rxoff_mode;
}

void cc1101_set_cca_mode(cca_mode_t cca_mode)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mcsm1_t mcsm1 = {
		.reg = cc1101_read_config_register(CC1101_MCSM1)};
	mcsm1.cca_mode = cca_mode;

	cc1101_write_register(CC1101_MCSM1, mcsm1.reg);
	ESP_LOGD(TAG, "Set CCA Mode %d.", mcsm1.cca_mode);
}

cca_mode_t cc1101_get_cca_mode(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mcsm1_t mcsm1 = {
		.reg = cc1101_read_config_register(CC1101_MCSM1)};
	return mcsm1.cca_mode;
}

void cc1101_set_sync_word(sync_word_t sync_word)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	cc1101_write_register(CC1101_SYNC0, sync_word.sync0);
	cc1101_write_register(CC1101_SYNC1, sync_word.sync1);
	ESP_LOGD(TAG, "Set Sync Word %d.", sync_word.sync);
}

sync_word_t cc1101_get_sync_word(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	sync_word_t sync_word = {
		.sync0 = cc1101_read_config_register(CC1101_SYNC0),
		.sync1 = cc1101_read_config_register(CC1101_SYNC1)};
	return sync_word;
}

void cc1101_set_forward_error_correction(uint8_t fec_en)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mdmcfg1_t mdmcfg1 = {
		.reg = cc1101_read_config_register(CC1101_MDMCFG1)};
	mdmcfg1.fec_en = fec_en;

	cc1101_write_register(CC1101_MDMCFG1, mdmcfg1.reg);
	ESP_LOGD(TAG, "Set Forward Error Correction %d.", mdmcfg1.fec_en);
}

uint8_t cc1101_get_forward_error_correction(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mdmcfg1_t mdmcfg1 = {
		.reg = cc1101_read_config_register(CC1101_MDMCFG1)};
	return mdmcfg1.fec_en;
}

void cc1101_set_num_preamble(num_preamble_t num_preamble)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mdmcfg1_t mdmcfg1 = {
		.reg = cc1101_read_config_register(CC1101_MDMCFG1)};
	mdmcfg1.num_preamble = num_preamble;

	cc1101_write_register(CC1101_MDMCFG1, mdmcfg1.reg);
	ESP_LOGD(TAG, "Set Preamble bytes %d.", mdmcfg1.num_preamble);
}

num_preamble_t cc1101_get_num_preamble(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	mdmcfg1_t mdmcfg1 = {
		.reg = cc1101_read_config_register(CC1101_MDMCFG1)};
	return mdmcfg1.num_preamble;
}

void cc1101_set_deviation_frequency(double frequencyInkHz)
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

	cc1101_write_register(CC1101_DEVIATN, deviatn.reg);

	ESP_LOGD(TAG, "Set Deviation Frequency %f. E: %d, M: %d ", frequencyInkHz, deviatn.deviation_e, deviatn.deviation_m);
}

double cc1101_get_deviation_frequency(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	deviatn_t deviatn = {
		.reg = cc1101_read_config_register(CC1101_DEVIATN)};

	double_t deviation = 1000.0 * CC1101_CLOCK_FREQUENCY * (8 + deviatn.deviation_m) * pow(2, deviatn.deviation_e) / pow(2, 17);
	return deviation;
}

void cc1101_set_controls_address_check(adr_chk_t adr_chk)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktctrl1_t pktctrl1 = {
		.reg = cc1101_read_config_register(CC1101_PKTCTRL1)};
	pktctrl1.adr_chk = adr_chk;

	cc1101_write_register(CC1101_PKTCTRL1, pktctrl1.reg);
	ESP_LOGD(TAG, "Set Address Check: %d.", pktctrl1.adr_chk);
}

adr_chk_t get_controls_address_check(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktctrl1_t pktctrl1 = {
		.reg = cc1101_read_config_register(CC1101_PKTCTRL1)};
	return pktctrl1.adr_chk;
}

void cc1101_set_append_status(uint8_t append_status)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktctrl1_t pktctrl1 = {
		.reg = cc1101_read_config_register(CC1101_PKTCTRL1)};
	pktctrl1.append_status = append_status;

	cc1101_write_register(CC1101_PKTCTRL1, pktctrl1.reg);
	ESP_LOGD(TAG, "Set Append Status: %d.", pktctrl1.append_status);
}

uint8_t cc1101_get_append_status(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktctrl1_t pktctrl1 = {
		.reg = cc1101_read_config_register(CC1101_PKTCTRL1)};

	return pktctrl1.append_status;
}

void cc1101_set_preamble_quality_threshold(uint8_t pqt)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktctrl1_t pktctrl1 = {
		.reg = cc1101_read_config_register(CC1101_PKTCTRL1)};
	pktctrl1.pqt = pqt;

	cc1101_write_register(CC1101_PKTCTRL1, pktctrl1.reg);
	ESP_LOGD(TAG, "Set Preamble Quality Threshold: %d.", pktctrl1.pqt);
}

uint8_t cc1101_get_preamble_quality_threshold(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktctrl1_t pktctrl1 = {
		.reg = cc1101_read_config_register(CC1101_PKTCTRL1)};

	return pktctrl1.pqt;
}

void cc1101_set_crc_autoflush(uint8_t crc_autoflush)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktctrl1_t pktctrl1 = {
		.reg = cc1101_read_config_register(CC1101_PKTCTRL1)};
	pktctrl1.crc_autoflush = crc_autoflush;

	cc1101_write_register(CC1101_PKTCTRL1, pktctrl1.reg);
	ESP_LOGD(TAG, "Set CRC Autoflush: %d.", pktctrl1.crc_autoflush);
}

uint8_t cc1101_get_crc_autoflush(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	pktctrl1_t pktctrl1 = {
		.reg = cc1101_read_config_register(CC1101_PKTCTRL1)};

	return pktctrl1.crc_autoflush;
}

void cc1101_set_device_address(uint8_t device_addr)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	addr_t addr = {
		.device_addr = device_addr};

	cc1101_write_register(CC1101_ADDR, addr.device_addr);
	ESP_LOGD(TAG, "Set Device Address: %d.", addr.device_addr);
}

uint8_t cc1101_get_device_address(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	return cc1101_read_config_register(CC1101_ADDR);
}

void cc1101_set_channel_number(uint8_t chan)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	channr_t channr = {
		.chan = chan};

	cc1101_write_register(CC1101_CHANNR, channr.chan);
	ESP_LOGD(TAG, "Set Channel Number: %d.", channr.chan);
}

uint8_t cc1101_get_channel_number(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	return cc1101_read_config_register(CC1101_CHANNR);
}

void cc1101_set_rx_attenuation(close_in_rx_t close_in_rx)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	fifothr_t fifothr = {
		.reg = cc1101_read_config_register(CC1101_FIFOTHR)};
	fifothr.close_in_rx = close_in_rx;

	cc1101_write_register(CC1101_FIFOTHR, fifothr.reg);
	ESP_LOGD(TAG, "Set RX Attenuationr: %d.", fifothr.close_in_rx);
}

uint8_t cc1101_get_rx_attenuation(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	fifothr_t fifothr = {
		.reg = cc1101_read_config_register(CC1101_FIFOTHR)};

	return fifothr.close_in_rx;
}

void cc1101_set_pa_power_setting(uint8_t pa_power)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	frend0_t frend0 = {
		.reg = cc1101_read_config_register(CC1101_FREND0)};
	frend0.pa_power = pa_power;

	cc1101_write_register(CC1101_FREND0, frend0.reg);
	ESP_LOGD(TAG, "Set PA power setting: %d.", frend0.pa_power);
}

uint8_t cc1101_get_pa_power_setting(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	frend0_t frend0 = {
		.reg = cc1101_read_config_register(CC1101_FREND0)};

	return frend0.pa_power;
}

void cc1101_set_filter_length(filter_length_t filter_length)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	agcctrl0_t agcctrl0 = {
		.reg = cc1101_read_config_register(CC1101_AGCCTRL0)};
	agcctrl0.filter_length = filter_length;

	cc1101_write_register(CC1101_AGCCTRL0, agcctrl0.reg);
	ESP_LOGD(TAG, "Set Filter Length: %d.", agcctrl0.filter_length);
}

uint8_t cc1101_get_filter_length(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	agcctrl0_t agcctrl0 = {
		.reg = cc1101_read_config_register(CC1101_AGCCTRL0)};

	return agcctrl0.filter_length;
}

void cc1101_set_hysteresis_level(hyst_level_t hyst_level)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	agcctrl0_t agcctrl0 = {
		.reg = cc1101_read_config_register(CC1101_AGCCTRL0)};
	agcctrl0.hyst_level = hyst_level;

	cc1101_write_register(CC1101_AGCCTRL0, agcctrl0.reg);
	ESP_LOGD(TAG, "Set Filter Length: %d.", agcctrl0.hyst_level);
}

hyst_level_t cc1101_get_hysteresis_level(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	agcctrl0_t agcctrl0 = {
		.reg = cc1101_read_config_register(CC1101_AGCCTRL0)};

	return agcctrl0.hyst_level;
}

void cc1101_set_magn_target(magn_target_t magn_target)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	agcctrl2_t agcctrl2 = {
		.reg = cc1101_read_config_register(CC1101_AGCCTRL2)};
	agcctrl2.magn_target = magn_target;

	cc1101_write_register(CC1101_AGCCTRL2, agcctrl2.reg);
	ESP_LOGD(TAG, "Set Filter Length: %d.", agcctrl2.magn_target);
}

magn_target_t cc1101_get_magn_target(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	agcctrl2_t agcctrl2 = {
		.reg = cc1101_read_config_register(CC1101_AGCCTRL2)};

	return agcctrl2.magn_target;
}

uint8_t cc1101_get_rssi(void)
{
	return cc1101_read_status_register(CC1101_RSSI);
}

float cc1101_get_rssi_dbm(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	uint8_t rssi = cc1101_get_rssi();
	if (rssi < 128)
	{
		return (rssi / 2) - 74;
	}
	else
	{
		return ((rssi - 256) / 2) - 74;
	}
}

uint8_t cc1101_get_tx_fifo_info(void)
{
	return cc1101_read_status_register(CC1101_TXBYTES) & 0b01111111;
}

void cc1101_set_pa_table(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);
	cc1101_write_burst_register(CC1101_PATABLE, pa_table_power, sizeof(pa_table_power));
	ESP_LOGD(TAG, "Set PA power table.");
}

void cc1101_get_pa_table(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	uint8_t pa_table[8];
	cc1101_read_burst_register(CC1101_PATABLE, pa_table, sizeof(pa_table_power));

	for (uint8_t i = 0; i < sizeof(pa_table_power); i++)
	{
		ESP_LOGD(TAG, "%x - %x", i, pa_table[i]);
	}
	ESP_LOGD(TAG, "Get PA power table.");
}

void cc1101_reset(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	cc1101_command_strobe(CC1101_SRES);
}

void cc1101_set_idle_state(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	cc1101_command_strobe(CC1101_SIDLE);
	ESP_LOGD(TAG, "Set Idle state");
}

void cc1101_set_flush_tx(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	cc1101_command_strobe(CC1101_SFTX);
	ESP_LOGD(TAG, "Set flush TX");
}

void cc1101_set_flush_rx(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	cc1101_command_strobe(CC1101_SFRX);
	ESP_LOGD(TAG, "Set flush RX");
}

void cc1101_set_tx_state(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	cc1101_command_strobe(CC1101_STX);
	ESP_LOGD(TAG, "Set Tx state");
}

void cc1101_set_rx_state(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	cc1101_command_strobe(CC1101_SRX);
	ESP_LOGD(TAG, "Set Rx state");
}

void cc1101_set_calibrate_state(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	cc1101_command_strobe(CC1101_SCAL);
	ESP_LOGD(TAG, "Set Ralibrate state");
}

void cc1101_set_command_state(CC1101_command_strobe_t command_strobe)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	cc1101_command_strobe(command_strobe);
	ESP_LOGD(TAG, "Set Command: %d", command_strobe);
}

CC1101_marc_state_t cc1101_get_marc_state(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);
	CC1101_marc_state_t state = cc1101_read_status_register(CC1101_MARCSTATE) & 0x1F;
	ESP_LOGD(TAG, "Marc State is %d", state);

	return state;
}

void send_data(void)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	DRAM_ATTR static uint8_t data1[] = {0x00, 0x55, 0x55, 0xAA, 0xAA};
	DRAM_ATTR static uint8_t data2[] = {0x00, 0xAA, 0xAA, 0x55, 0x55};

	cc1101_write_burst_register(CC1101_TXFIFO, data1, 5);
	cc1101_set_command_state(CC1101_STX);

	while (cc1101_get_marc_state() != CC1101_STATE_IDLE)
	{
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}
	cc1101_set_flush_tx();

	cc1101_write_burst_register(CC1101_TXFIFO, data2, 5);
	cc1101_set_command_state(CC1101_STX);

	while (cc1101_get_marc_state() != CC1101_STATE_IDLE)
	{
		vTaskDelay(100 / portTICK_PERIOD_MS);
	}

	cc1101_set_flush_tx();
}
#endif