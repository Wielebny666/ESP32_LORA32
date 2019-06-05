// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_log.h"

// #include "driver/spi_master.h"

// #include "cc1101_async.h"
// #include "cc1101.h"

// #define PIN_NUM_MISO 19
// #define PIN_NUM_MOSI 23
// #define PIN_NUM_CLK 18
// #define PIN_NUM_CS 5

// static spi_device_handle_t spi;

// void spi_init()
// {
//     spi_bus_config_t buscfg = {
//         .miso_io_num = PIN_NUM_MISO,
//         .mosi_io_num = PIN_NUM_MOSI,
//         .sclk_io_num = PIN_NUM_CLK,
//         .quadwp_io_num = -1,
//         .quadhd_io_num = -1};

//     spi_device_interface_config_t devcfg = {
//         .clock_speed_hz = 1 * 1000 * 1000, //Clock out at 10 MHz
//         .mode = 3,                         //SPI mode 0
//         .spics_io_num = PIN_NUM_CS,        //CS pin
//         .queue_size = 1,                   //We want to be able to queue 7 transactions at a time
//         .command_bits = 8};

//     //Initialize the SPI bus
//     ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, 0));
//     //Attach the Device to the SPI bus
//     ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg, &spi));
// }

// void print_config()
// {
//     printf("Frequency is %f Hz\n", get_carrier_frequency(spi));
//     printf("Baudrate is %f bps\n", get_baud_rate(spi));
//     printf("Bandwidth is %d kHz\n", get_rx_channel_bandwidth(spi));
//     printf("Modulation format is %d\n", get_modulation_format(spi));
//     printf("Sync mode is %d\n", get_sync_mode(spi));
//     // printf("Packet Length is %d\n", get_packet_length(spi));
//     // printf("Length config is %d\n", get_length_config(spi));
//     // printf("RSSI %f dBm\n", get_rssi_dbm(spi));
//     // printf("CCA mode %d\n", get_cca_mode(spi));
//     // printf("TxOff mode %d\n", get_txoff_mode(spi));
//     // printf("RxOff mode %d\n", get_rxoff_mode(spi));

//     // printf("FEC  %d\n", get_forward_error_correction(spi));
//     // printf("Whitedata %d\n", get_white_data(spi));
//     // printf("CRC %d\n", get_crc_enable(spi));
//     // printf("Address control %d\n", get_controls_address_check(spi));
//     // printf("Dev freq is %f kHz\n", get_deviation_frequency(spi));
// }

// void cc1101_rx_async(void *pvParameter)
// {
//     spi_init();
//     vTaskDelay(200 / portTICK_PERIOD_MS);
//     reset(spi);
//     init(spi);
//     set_pa_table(spi);
//     set_baud_rate(3);
//     set_magn_target(MAGN_42DB);
//     set_filter_length(FILTER_16DB);
//     set_rx_channel_bandwidth(58);
//     set_rx_attenuation(ATTENUATION_18DB);
//     set_hysteresis_level(HYSTERESIS_LARGE);
//     print_config();
//     vTaskDelay(200 / portTICK_PERIOD_MS);
//     set_idle_state(spi);
//     vTaskDelay(400 / portTICK_PERIOD_MS);
//     set_calibrate_state(spi);
//     vTaskDelay(400 / portTICK_PERIOD_MS);
//     set_rx_state(spi);

//     while (1)
//     {
//         //printf("RSSI %f dBm\n", get_rssi_dbm(spi));
//         vTaskDelay(10000 / portTICK_PERIOD_MS);
//     }
//     vTaskDelete(NULL);
// }

// void cc1101_tx_async(void *pvParameter)
// {
//     spi_init();
//     vTaskDelay(200 / portTICK_PERIOD_MS);
//     reset(spi);
//     init(spi);
//     set_pa_table(spi);
//     print_config();
//     vTaskDelay(200 / portTICK_PERIOD_MS);
//     set_idle_state(spi);
//     vTaskDelay(200 / portTICK_PERIOD_MS);
//     set_tx_state(spi);

//     while (1)
//     {
//         printf("RSSI %f dBm\n", get_rssi_dbm(spi));

//         vTaskDelay(500 / portTICK_PERIOD_MS);
//     }
//     vTaskDelete(NULL);
// }