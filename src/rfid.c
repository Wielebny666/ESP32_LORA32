
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include "driver/rmt.h"
#include "esp_log.h"

#include "rfid.h"
#include "rfid_radio_control.h"
#include "demodulator.h"

#include "hardware.h"

static const char *TAG = "rfid";

typedef enum
{
	RX,
	RX_PENDING,
	RX_FINISH,
	RX_TIMEOUT,
	RX_ERROR,
	TX,
	CALLIBRATE,
	IDDLE
} rfid_states_t;

static TaskHandle_t rmt_rx_task_handler;
rfid_configuration_t *rfid_task_param;

volatile rfid_states_t rfid_state = IDDLE;

static void on_begin();
static void on_finish();
static void on_pending(rmt_item32_t *items, size_t qty);
static void on_timeout();
static void on_error();
static void IRAM_ATTR wake_up_isr_handler(void *arg);

void rfid_init(uint8_t freq, uint8_t bitrate, bool manchaster)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	radio_init();
	radio_setup();
	radio_check_init();
}

void rfid_io_init(uint8_t w_up_io_num)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	gpio_config_t io_conf = {
		.intr_type = GPIO_INTR_POSEDGE,
		.pin_bit_mask = (1ULL << w_up_io_num),
		.mode = GPIO_MODE_INPUT,
		.pull_down_en = 1};
	ESP_ERROR_CHECK(gpio_config(&io_conf));

	//install gpio isr service
	ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT));
	//hook isr handler for specific gpio pin
	ESP_ERROR_CHECK(gpio_isr_handler_add(w_up_io_num, wake_up_isr_handler, (void *)&w_up_io_num));
}

void rfid_task(void *pvParameter)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);

	assert(pvParameter);

	rfid_configuration_t configuration = *(rfid_configuration_t *)pvParameter;
	rfid_task_param = &configuration;

	demodulator_configuration_t *demod_config = malloc(sizeof(*demod_config));
	demod_config->rmt_gpio_num = RFID_DATA;
	demod_config->ctrl_gpio_num = RFID_WAKE_UP;
	demod_config->rmt_clk_div = 1;
	demod_config->rmt_channel = RMT_CHANNEL_0;
	demod_config->rx_buff_items = 8 * 64 * 20;
	demod_config->budrate_filter = 120000;
	demod_config->min_items_count = 1;
	demod_config->measure_idle_threshold_us = 80;
	demod_config->begin_measure_callback = &on_begin;
	demod_config->pending_measure_callback = &on_pending;
	demod_config->finish_measure_callback = &on_finish;
	demod_config->timeout_measure_callback = &on_timeout;
	demod_config->error_measure_callback = &on_error;

	xTaskCreate(rmt_rx_task, "rmt_rx_task", 1024 * 5, demod_config, 40, &rmt_rx_task_handler);
	vTaskDelay(100 / portTICK_PERIOD_MS);
	rfid_init(rfid_task_param->frequency, rfid_task_param->bitrate, rfid_task_param->manchester);
	rfid_io_init(RFID_WAKE_UP);

	while (1)
	{
		switch (rfid_state)
		{
		case RX:
			ESP_LOGD(TAG, "RX");
			rfid_state = IDDLE;
			break;
		case RX_PENDING:
			ESP_LOGD(TAG, "RX_PENDING");
			rfid_state = IDDLE;
			break;
		case RX_FINISH:
			ESP_LOGD(TAG, "RX_FINISH");
			rfid_state = IDDLE;
			break;
		case RX_TIMEOUT:
		case RX_ERROR:
			ESP_LOGD(TAG, "RX_ERROR");
			rfid_state = IDDLE;
			break;
		case TX:
			break;
		case CALLIBRATE:
			break;
		case IDDLE:
		default:
			ESP_LOGD(TAG, "IDDLE");
			vTaskDelay(250 / portTICK_PERIOD_MS);
			break;
		}
	}
}

static void wake_up_isr_handler(void *arg)
{
	assert(rmt_rx_task_handler);
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xTaskNotifyFromISR(rmt_rx_task_handler, 0, eNoAction, &xHigherPriorityTaskWoken);
}

static void on_begin()
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);
	rfid_state = RX;
}

static void on_pending(rmt_item32_t *items, size_t qty)
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);
	if (rfid_task_param->wake_up_callback != NULL)
	{
		uint8_t rssi1, rssi2, rssi3;
		radio_get_rssi(&rssi1, &rssi2, &rssi3);

		rfid_task_param->wake_up_callback(rssi1, rssi2, rssi3);
	}
	radio_wake_up_reset();
	print_rx_data(items, qty);
	rfid_state = RX_PENDING;
}

static void on_finish()
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);
	if (rfid_task_param->wake_up_callback != NULL)
	{
		uint8_t rssi1, rssi2, rssi3;
		radio_get_rssi(&rssi1, &rssi2, &rssi3);

		rfid_task_param->wake_up_callback(rssi1, rssi2, rssi3);
	}
	radio_wake_up_reset();
	rfid_state = RX_FINISH;
}

static void on_timeout()
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);
	radio_wake_up_reset();
	rfid_state = RX_TIMEOUT;
}

static void on_error()
{
	ESP_LOGD(TAG, "%s", __FUNCTION__);
	if (rfid_task_param->wake_up_callback != NULL)
	{
		uint8_t rssi1, rssi2, rssi3;
		radio_get_rssi(&rssi1, &rssi2, &rssi3);

		rfid_task_param->wake_up_callback(rssi1, rssi2, rssi3);
	}
	radio_wake_up_reset();
	rfid_state = RX_ERROR;
}
