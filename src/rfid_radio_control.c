#include "freertos/FreeRTOS.h"
#include "esp_log.h"

#include "spi_iface.h"
#include "rfid_controller.h"
#include "rfid_radio_control.h"
#include "as3933.h"

#include "hardware.h"

static const char *TAG = "rfid_radio_control";

#define RR_CONTROL_CHECK(a, ret_val, str, ...)                                \
	if (!(a))                                                                 \
	{                                                                         \
		ESP_LOGE(TAG, "%s(%u): " str, __FUNCTION__, __LINE__, ##__VA_ARGS__); \
		return (ret_val);                                                     \
	}

esp_err_t radio_init()
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);
	spi_communication_info_t comm = {
		.port = RFID_SPI,
		.miso = RFID_MISO,
		.mosi = RFID_MOSI,
		.sclk = RFID_SCLK,
		.cs = RFID_CS,
		.spi_clock_speed_hz = RFID_CLK_SPEED};

	void *radio_handler = NULL;
	esp_err_t error = spi_init(RFID1, &radio_handler);
	RR_CONTROL_CHECK((radio_handler != NULL),
					 ESP_ERR_INVALID_STATE,
					 "rfid controller initialization fail.");
	RR_CONTROL_CHECK((error == ESP_OK),
					 ESP_ERR_INVALID_STATE,
					 "rfid controller initialization fail, returns(0x%x).",
					 (uint32_t)error);
	error = spi_setup(RFID1, (void *)&comm);
	RR_CONTROL_CHECK((error == ESP_OK),
					 ESP_ERR_INVALID_STATE,
					 "rfid controller setup fail, returns(0x%x).",
					 (uint32_t)error);
	return error;
}

void radio_setup()
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);
	as3933_reset();
	//	as3933_clear_wake_up();
	//	uint8_t test5 = as3933_read(5);
	//	uint8_t test6 = as3933_read(6);
	//	ESP_LOGD(TAG, "%x %x", test5, test6);

	as3933_set_listening_mode(LM_STANDARD);
	as3933_band_select(125000);
	as3933_set_freq_tolerance(TIGHT);
	as3933_set_gain_reduction(GAIN_REDUCTION_12DB);
	as3933_set_antenna_damper(RESISTOR_3KOM);
	as3933_set_off_timer(OFF_1MS);
	as3933_enable_antenna_damper(true);

	as3933_set_manchaster_decode(false);
	as3933_set_patern_correlation(WK_FREQ_DET_ONLY);

	as3933_set_capacity(1, 5);
	as3933_set_capacity(2, 15);
	as3933_set_capacity(3, 17);

	as3933_set_channel(1, true);
	as3933_set_channel(2, true);
	as3933_set_channel(3, true);

	//as3933_route_res_freq_on_dat(1, true);
	as3933_reset_rssi();
	as3933_clear_wake_up();
}

void radio_reset()
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);
	as3933_reset();
}

void radio_wake_up_reset()
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);
	as3933_clear_wake_up();
}

void radio_rssi_reset()
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);
	as3933_reset_rssi();
}

bool radio_check_init()
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);
	uint16_t wu_patt = as3933_get_wakeup_pattern_16bit();
	RR_CONTROL_CHECK((wu_patt == 0x6996),
					 ESP_ERR_INVALID_STATE,
					 "rfid controller initialization fail, returns(0x%x).",
					 (uint32_t)wu_patt);
	return wu_patt == 0x6996;
}

void radio_get_rssi(uint8_t *rssi1, uint8_t *rssi2, uint8_t *rssi3)
{

	ESP_LOGD(TAG, "%s", __FUNCTION__);

	*rssi1 = as3933_get_rssi(1);
	*rssi2 = as3933_get_rssi(2);
	*rssi3 = as3933_get_rssi(3);
}