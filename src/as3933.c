#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "rfid/include/spi_iface.h"
//#include "hardware.h"
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

//static spi_device_handle_t as3933;

uint16_t convert_hex_to_manchester(uint8_t hex);

// void as3933_spi_init()
// {
//     ESP_LOGD(TAG, "%s", __FUNCTION__);

//     spi_bus_config_t buscfg = {
//         .miso_io_num = RFID_MISO,
//         .mosi_io_num = RFID_MOSI,
//         .sclk_io_num = RFID_SCLK,
//         .quadwp_io_num = -1,
//         .quadhd_io_num = -1};

//     spi_device_interface_config_t devcfg = {
//         .clock_speed_hz = SPI_MASTER_FREQ_10M / 2, //Clock out at 10 MHz
//         .mode = 1,                                 //SPI mode 1
//         .spics_io_num = RFID_CS,                   //CS pin
//         .flags = SPI_DEVICE_POSITIVE_CS,
//         .queue_size = 1, //We want to be able to queue 7 transactions at a time
//         .command_bits = 8};

//     //Initialize the SPI bus
//     ESP_ERROR_CHECK(spi_bus_initialize(RFID_SPI, &buscfg, 0));
//     //Attach the Device to the SPI bus
//     ESP_ERROR_CHECK(spi_bus_add_device(RFID_SPI, &devcfg, &as3933));
// }

// void as3933_w_up_irq_init(){

// }

// void spi_cmd(cmd_t cmd)
// {
//     spi_transaction_t t;
//     memset(&t, 0, sizeof(t));
//     t.length = 0;
//     t.cmd = cmd | DIRECT_COMMAND;
//     //t.flags = SPI_TRANS_USE_TXDATA;

//     ESP_ERROR_CHECK(spi_device_polling_transmit(as3933, &t));
// }

// void spi_write_byte(uint8_t addr, uint8_t data)
// {
//     spi_transaction_t t;
//     memset(&t, 0, sizeof(t));
//     t.length = 8;
//     t.cmd = addr & WRITE;
//     t.flags = SPI_TRANS_USE_TXDATA;
//     t.tx_data[0] = data;

//     ESP_ERROR_CHECK(spi_device_polling_transmit(as3933, &t));
// }

// uint8_t spi_read_byte(uint8_t addr)
// {
//     spi_transaction_t t;
//     memset(&t, 0, sizeof(t));
//     t.length = 8;
//     t.cmd = addr | READ;
//     t.flags = SPI_TRANS_USE_RXDATA;

//     ESP_ERROR_CHECK(spi_device_polling_transmit(as3933, &t));
//     return t.rx_data[0];
// }

// void spi_write_byte_buffer(uint16_t addr, uint8_t *buffer, uint8_t size)
// {
//     spi_transaction_t t;
//     memset(&t, 0, sizeof(t));
//     t.length = size * 8;
//     t.cmd = addr | 0x80;
//     t.tx_buffer = buffer;

//     ESP_ERROR_CHECK(spi_device_polling_transmit(as3933, &t));
// }

// void spi_read_byte_buffer(uint16_t addr, uint8_t *buffer, uint8_t size)
// {
//     spi_transaction_t t;
//     memset(&t, 0, sizeof(t));
//     t.length = size * 8;
//     t.cmd = addr & 0x7F;
//     t.rx_buffer = buffer;

//     ESP_ERROR_CHECK(spi_device_polling_transmit(as3933, &t));
// }

void as3933_reset()
{
    spi_cmd(PRESET_DEFAULT);
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

    r0_t r0;
    spi_read_byte(R0, &r0.reg);

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
    spi_write_byte(R0, r0.reg);
}

bool as3933_get_channel(uint8_t channel)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r0_t r0;

    spi_read_byte(R0, &r0.reg);

    switch (channel)
    {
    case 1:
        return r0.en_a1;
        break;
    case 2:
        return r0.en_a2;
        break;
    case 3:
        return r0.en_a3;
        break;
    default:
        return NULL;
        break;
    }
}

void as3933_set_manchaster_decode(bool select)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r1_t r1;
    spi_read_byte(R1, &r1.reg);

    r1.en_manch = select;
    spi_write_byte(R1, r1.reg);
}

void as3933_set_patern_correlation(wake_up_mode_t mode)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r1_t r1;
    spi_read_byte(R1, &r1.reg);

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

    spi_write_byte(R1, r1.reg);
}

void as3933_set_wakeup_pattern_16bit(uint16_t wakeup_node_id)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    spi_write_byte(R6, (uint8_t)wakeup_node_id);
    spi_write_byte(R5, (uint8_t)(wakeup_node_id >> 8));
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

    r7_t r7;
    spi_read_byte(R7, &r7.reg);
    r7.t_hbit = value;
    spi_write_byte(R7, r7.reg);
}

void as3933_band_select(uint32_t freq)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r8_t r8;
    spi_read_byte(R8, &r8.reg);

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
    spi_write_byte(R8, r8.reg);
}

void as3933_route_res_freq_on_dat(uint8_t channel, bool value)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r16_t r16;
    spi_read_byte(R16, &r16.reg);

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
    spi_write_byte(R16, r16.reg);
}

void as3933_route_clock_on_dat(bool value)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r2_t r2;
    spi_read_byte(R2, &r2.reg);
    r2.display_clk = value ? 0b11 : 0;
    spi_write_byte(R2, r2.reg);

    r16_t r16;
    spi_read_byte(R16, &r16.reg);
    r16.clock_gen_dis = value;
    spi_write_byte(R16, r16.reg);
}

void as3933_set_xtal_osc(bool value)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r1_t r1;
    spi_read_byte(R1, &r1.reg);
    r1.en_xtal = value;
    spi_write_byte(R1, r1.reg);
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
        spi_write_byte(R17, r17.reg);
        break;
    }
    case 2:
    {
        r18_t r18 = {
            .cap_ch2 = value};
        spi_write_byte(R18, r18.reg);
        break;
    }
    case 3:
    {
        r19_t r19 = {
            .cap_ch3 = value};
        spi_write_byte(R19, r19.reg);
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
        r10_t r10;
        spi_read_byte(R10, &r10.reg);
        resp = r10.rssi1;
    }
    break;
    case 2:
    {
        r11_t r11;
        spi_read_byte(R11, &r11.reg);
        resp = r11.rssi2;
    }
    break;
    case 3:
    {
        r12_t r12;
        spi_read_byte(R12, &r12.reg);
        resp = r12.rssi3;
    }
    break;
    default:
        resp = 0;
        break;
    }
    return resp;
}

bool as3933_get_rc_osc_calibrate_status()
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r14_t r14;
    spi_read_byte(R14, &r14.reg);
    return r14.rc_cal_ok;
}

//Enable Data slicer absolute reference
void as3933_set_data_slicer(bool value)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r1_t r1;
    spi_read_byte(R1, &r1.reg);
    r1.abs_hy = value;
    spi_write_byte(R1, r1.reg);
}

//Data slicer absolute threshold reduction
void as3933_set_data_slicer_threshold_reduction(bool value)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r2_t r2;
    spi_read_byte(R2, &r2.reg);
    r2.s_abs = value;
    spi_write_byte(R2, r2.reg);
}

void as3933_set_block_agc(bool value)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r9_t r9;
    spi_read_byte(R9, &r9.reg);
    r9.block_agc = value;
    spi_write_byte(R9, r9.reg);
}

bool as3933_get_block_agc(void)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r9_t r9;
    spi_read_byte(R9, &r9.reg);
    return r9.block_agc;
}

void as3933_set_min_preamble_length(fs_slc_t len)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r3_t r3;
    spi_read_byte(R3, &r3.reg);
    r3.fs_slc = len;
    spi_write_byte(R3, r3.reg);
}

void as3933_set_listening_mode(listening_mode_t mode)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r0_t r0;
    spi_read_byte(R0, &r0.reg);

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
    spi_write_byte(R0, r0.reg);
}

void as3933_set_freq_tolerance(s_wu1_t value)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r2_t r2;
    spi_read_byte(R2, &r2.reg);

    r2.s_wu1 = value;
    spi_write_byte(R2, r2.reg);
}

void as3933_set_gain_reduction(gr_t value)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r4_t r4;
    spi_read_byte(R4, &r4.reg);

    r4.gr = value;
    spi_write_byte(R4, r4.reg);
}

void as3933_enable_antenna_damper(bool value)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r1_t r1;
    spi_read_byte(R1, &r1.reg);

    r1.att_on = value;
    spi_write_byte(R1, r1.reg);
}

void as3933_set_antenna_damper(r_val_t value)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r4_t r4;
    spi_read_byte(R4, &r4.reg);

    r4.r_val = value;

    spi_write_byte(R4, r4.reg);
}

//OFF time in ON/OFF operation mode
void as3933_set_off_timer(t_off_t value)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r4_t r4;
    spi_read_byte(R4, &r4.reg);

    r4.t_off = value;

    spi_write_byte(R4, r4.reg);
}

void as3933_set_comparator_hysteresis(comp_hyst_t value)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    r3_t r3;
    spi_read_byte(R3, &r3.reg);
    r3.hy_pos = value;
    r3.hy_20m = value >> 1;
    spi_write_byte(R3, r3.reg);
}

void as3933_clear_wake_up()
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    spi_cmd(CLEAR_WAKE);
}

void as3933_calibrate_rco_lc()
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    spi_cmd(CALIB_RCO_LC);
}

void as3933_reset_rssi()
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    spi_cmd(RESET_RSSI);
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
