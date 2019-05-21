#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "hardware.h"
#include "as3933.h"

as3933_init_cmd_t as3933_config_register[] = {
    // AS3933 default settings for approx. 9m LF range
    {R0, 0x6E},
    {R1, 0x2A}, // AGC_T-LIM = 0 Had improved stability (RSSI value constant), but was causing lost LF. Changed back
    {R2, 0x20},
    {R3, 0x3F},
    {R4, 0x30}, // reduce Off time.
    {R5, 0x69},
    {R6, 0x96},
    {R7, 0x3F},
    {R8, 0x00},
    {R9, 0x00},
    {R16, 0x00},
    {R17, 0x00},
    {R18, 0x00},
    {R19, 0x00}};

static const char *TAG = "as3933";

static spi_device_handle_t as3933;

uint16_t convert_hex_to_manchester(uint8_t hex);

void as3933_spi_init()
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    spi_bus_config_t buscfg = {
        .miso_io_num = RFID_MISO,
        .mosi_io_num = RFID_MOSI,
        .sclk_io_num = RFID_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1};

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = SPI_MASTER_FREQ_10M / 2, //Clock out at 10 MHz
        .mode = 1,                                 //SPI mode 1
        .spics_io_num = RFID_CS,                   //CS pin
        .flags = SPI_DEVICE_POSITIVE_CS,
        .queue_size = 1, //We want to be able to queue 7 transactions at a time
        .command_bits = 8};

    //Initialize the SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(HSPI_HOST, &buscfg, 0));
    //Attach the Device to the SPI bus
    ESP_ERROR_CHECK(spi_bus_add_device(HSPI_HOST, &devcfg, &as3933));
}

void as3933_cmd(cmd_t cmd)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 0;
    t.cmd = cmd | DIRECT_COMMAND;
    //t.flags = SPI_TRANS_USE_TXDATA;

    ESP_ERROR_CHECK(spi_device_polling_transmit(as3933, &t));
}

void as3933_write(uint8_t addr, uint8_t data)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.cmd = addr & WRITE;
    t.flags = SPI_TRANS_USE_TXDATA;
    t.tx_data[0] = data;

    ESP_ERROR_CHECK(spi_device_polling_transmit(as3933, &t));
}

uint8_t as3933_read(uint8_t addr)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = 8;
    t.cmd = addr | READ;
    t.flags = SPI_TRANS_USE_RXDATA;

    ESP_ERROR_CHECK(spi_device_polling_transmit(as3933, &t));
    return t.rx_data[0];
}

void as3933_write_buffer(uint16_t addr, uint8_t *buffer, uint8_t size)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = size * 8;
    t.cmd = addr | 0x80;
    t.tx_buffer = buffer;

    ESP_ERROR_CHECK(spi_device_polling_transmit(as3933, &t));
}

void as3933_read_buffer(uint16_t addr, uint8_t *buffer, uint8_t size)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length = size * 8;
    t.cmd = addr & 0x7F;
    t.rx_buffer = buffer;

    ESP_ERROR_CHECK(spi_device_polling_transmit(as3933, &t));
}

void as3933_reset()
{
    as3933_cmd(PRESET_DEFAULT);
}

void as3933_init()
{
}

void as3933_crystal_osc_select(bool select)
{
}

void as3933_set_channel(uint8_t channel, bool value)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r0_t r0 = {
        .reg = as3933_read(R0)};

    switch (channel)
    {
    case 1:
        r0.en_a1 = value;
        break;
    case 2:
        r0.en_a2 = value;
        break;
    case 3:
        r0.en_a3 = value;
        break;
    default:
        break;
    }
    as3933_write(R0, r0.reg);
}

void as3933_set_manchaster_decode(bool select)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r1_t r1 = {
        .reg = as3933_read(R1)};

    r1.en_manch = select;
    as3933_write(R1, r1.reg);
}

void as3933_set_patern_correlation(wake_up_mode_t mode)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r1_t r1 = {
        .reg = as3933_read(R1)};
    switch (mode)
    {
    case WK_FREQ_DET_ONLY:
        r1.en_wpat = false;
        r1.en_pat2 = false;
        break;
    case WK_SINGLE_PATTERN:
        r1.en_wpat = true;
        r1.en_pat2 = false;
        break;
    }

    as3933_write(R1, r1.reg);
}

void as3933_set_wakeup_pattern_16bit(uint16_t wakeup_node_id)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    as3933_write(R6, (uint8_t)wakeup_node_id);
    as3933_write(R5, (uint8_t)(wakeup_node_id >> 8));
}

void as3933_set_wakeup_pattern_8bit(uint8_t wakeup_node_id)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    uint16_t address = convert_hex_to_manchester(wakeup_node_id);
    as3933_set_wakeup_pattern_16bit(address);
}

void as3933_set_bitrate(uint8_t value)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    if (value < 4)
        value = 4;
    else if (value > 32)
        value = 32;

    r7_t r7 = {
        .reg = as3933_read(R7)};
    r7.t_hbit = value;
    as3933_write(R7, r7.reg);
}

void as3933_band_select(uint32_t freq)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r8_t r8 = {
        .reg = as3933_read(R8)};

    if (freq >= 15000 && freq < 23000)
    {
        r8.band_sel = RANGE_15_23KHZ;
    }
    if (freq >= 23000 && freq < 40000)
    {
        r8.band_sel = RANGE_23_40KHZ;
    }
    if (freq >= 40000 && freq < 65000)
    {
        r8.band_sel = RANGE_40_65KHZ;
    }
    if (freq >= 65000 && freq < 95000)
    {
        r8.band_sel = RANGE_65_95KHZ;
    }
    if (freq >= 95000 && freq <= 150000)
    {
        r8.band_sel = RANGE_95_150KHZ;
    }
    as3933_write(R8, r8.reg);
}

void as3933_route_res_freq_on_dat(uint8_t channel, bool value)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r16_t r16 = {
        .reg = as3933_read(R16)};

    switch (channel)
    {
    case 0:
        r16.lc_osc_mux1 = value;
        r16.lc_osc_mux2 = value;
        r16.lc_osc_mux3 = value;
        break;
    case 1:
        r16.lc_osc_mux1 = value;
        r16.lc_osc_mux2 = false;
        r16.lc_osc_mux3 = false;
        break;
    case 2:
        r16.lc_osc_mux1 = false;
        r16.lc_osc_mux2 = value;
        r16.lc_osc_mux3 = false;
        break;
    case 3:
        r16.lc_osc_mux1 = false;
        r16.lc_osc_mux2 = false;
        r16.lc_osc_mux3 = value;
        break;
    default:
        break;
    }
    as3933_write(R16, r16.reg);
}

void as3933_route_clock_on_dat(bool value)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r2_t r2 = {
        .reg = as3933_read(R2)};
    r2.display_clk = value ? 0b11 : 0;
    as3933_write(R2, r2.reg);

    r16_t r16 = {
        .reg = as3933_read(R16)};
    r16.clock_gen_dis = value;
    as3933_write(R16, r16.reg);
}

void as3933_set_xtal_osc(bool value)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r1_t r1 = {
        .reg = as3933_read(R1)};
    r1.en_xtal = value;
    as3933_write(R1, r1.reg);
}

// 0 - 31 pF
void as3933_set_capacity(uint8_t channel, uint8_t value)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    if (value > 31)
        value = 31;

    switch (channel)
    {
    case 1:
    {
        r17_t r17 = {
            .cap_ch1 = value};
        as3933_write(R17, r17.reg);
        break;
    }
    case 2:
    {
        r18_t r18 = {
            .cap_ch2 = value};
        as3933_write(R18, r18.reg);
        break;
    }
    case 3:
    {
        r19_t r19 = {
            .cap_ch3 = value};
        as3933_write(R19, r19.reg);
        break;
    }
    default:
        break;
    }
}

void as3933_set_config()
{
}

uint8_t as3933_get_rssi(uint8_t channel)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    uint8_t resp;
    switch (channel)
    {
    case 1:
    {
        r10_t r10 = {
            .reg = as3933_read(R10)};
        resp = r10.rssi1;
        break;
    }
    case 2:
    {
        r11_t r11 = {
            .reg = as3933_read(R11)};
        resp = r11.rssi2;
        break;
    }
    case 3:
    {
        r12_t r12 = {
            .reg = as3933_read(R12)};
        resp = r12.rssi3;
        break;
    }
    default:
        resp = 0;
        break;
    }
    return resp;
}

bool as3933_get_rc_osc_calibrate_status()
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r14_t r14 = {
        .reg = as3933_read(R14)};
    return r14.rc_cal_ok;
}

void as3933_agc(bool value)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r9_t r9 = {
        .reg = as3933_read(R9)};
    r9.block_agc = value;
    as3933_write(R9, r9.reg);
}

void as3933_set_min_preamble_length(fs_slc_t len)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r3_t r3 = {
        .reg = as3933_read(R3)};
    r3.fs_slc = len;
    as3933_write(R3, r3.reg);
}

void as3933_set_listening_mode(listening_mode_t mode)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r0_t r0 = {
        .reg = as3933_read(R0)};

    switch (mode)
    {
    case LM_STANDARD:
        r0.on_off = false;
        r0.mux_123 = false;
        break;
    case LM_SCANNING:
        r0.on_off = false;
        r0.mux_123 = true;
        break;
    case LM_ON_OFF:
        r0.on_off = true;
        r0.mux_123 = false;
        break;
    default:
        return false;
    }
    as3933_write(R0, r0.reg);
}

void as3933_set_freq_tolerance(s_wu1_t value)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r2_t r2 = {
        .reg = as3933_read(R2)};
    r2.s_wu1 = value;
    as3933_write(R2, r2.reg);
}

void as3933_set_gain_reduction(gr_t value)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r4_t r4 = {
        .reg = as3933_read(R4)};
    r4.gr = value;
    as3933_write(R4, r4.reg);
}

void as3933_enable_antenna_damper(bool value)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r1_t r1 = {
        .reg = as3933_read(R1)};
    r1.att_on = value;
    as3933_write(R1, r1.reg);
}

void as3933_set_antenna_damper(r_val_t value)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r4_t r4 = {
        .reg = as3933_read(R4)};

    r4.r_val = value;

    as3933_write(R4, r4.reg);
}

void as3933_set_comparator_hysteresis(comp_hyst_t value)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r3_t r3 = {
        .reg = as3933_read(R3)};
    r3.hy_pos = value;
    r3.hy_20m = value >> 1;
    as3933_write(R3, r3.reg);
}

void as3933_clear_wake_up()
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    as3933_cmd(CLEAR_WAKE);
}

void as3933_calibrate_rco_lc()
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    as3933_cmd(CALIB_RCO_LC);
}

void as3933_reset_rssi()
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    as3933_cmd(RESET_RSSI);
}

uint16_t convert_hex_to_manchester(uint8_t hex)
{
    int8_t i = 8;
    uint16_t ret = 0;
    while (i > 0)
    {
        i--;
        ret <<= 2;
        if (hex & (1 << i))
        {
            ret |= 2;
        }
        else
        {
            ret |= 1;
        }
    }
    return ret;
}

bool as3933_rc_osc_self_calibrate()
{
    as3933_set_xtal_osc(false);
    as3933_calibrate_rco_lc();
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    return as3933_get_rc_osc_calibrate_status();
}

static void IRAM_ATTR as3933_on_wake_up_irq(void *context)
{
}