#ifndef __AS3933_H__
#define __AS3933_H__

#include <stdint.h>
#include "as3933_regs.h"

typedef enum
{
	LM_STANDARD,
	LM_SCANNING,
	LM_ON_OFF
} listening_mode_t;

typedef enum
{
	WK_FREQ_DET_ONLY,
	WK_SINGLE_PATTERN
} wake_up_mode_t;

#define XTAL_FREQ 32000

//void as3933_spi_init(as3933_conf_t config);
//void as3933_w_up_irq_init();
//void as3933_cmd(cmd_t cmd);
//void as3933_write(uint8_t addr, uint8_t data);
//uint8_t as3933_read(uint8_t addr);
//void as3933_write_buffer(uint16_t addr, uint8_t *buffer, uint8_t size);
//void as3933_read_buffer(uint16_t addr, uint8_t *buffer, uint8_t size);
//void as3933_init(const as3933_hal_t *device);
void as3933_reset();
void as3933_band_select(uint32_t freq);

void as3933_set_channel(uint8_t channel, bool value);
bool as3933_get_channel(uint8_t channel);
void as3933_set_manchaster_decode(bool select);
void as3933_set_patern_correlation(wake_up_mode_t mode);
void as3933_set_wakeup_pattern_16bit(uint16_t wakeup_node_id);
uint16_t as3933_get_wakeup_pattern_16bit();
void as3933_set_wakeup_pattern_8bit(uint8_t wakeup_node_id);
void as3933_set_bitrate(uint8_t value);
void as3933_band_select(uint32_t freq);
void as3933_route_res_freq_on_dat(uint8_t channel, bool value);
void as3933_route_clock_on_dat(bool value);
void as3933_set_xtal_osc(bool value);
// 0 - 31 pF
void as3933_set_capacity(uint8_t channel, uint8_t value);
uint8_t as3933_get_rssi(uint8_t channel);
bool as3933_get_rc_osc_calibrate_status();
void as3933_set_data_slicer(bool value);
void as3933_set_data_slicer_threshold_reduction(bool value);

void as3933_set_block_agc(bool value);
bool as3933_get_block_agc(void);
void as3933_set_min_preamble_length(fs_slc_t len);
void as3933_set_listening_mode(listening_mode_t mode);
void as3933_set_freq_tolerance(s_wu1_t value);
void as3933_set_gain_reduction(gr_t value);
void as3933_enable_antenna_damper(bool value);
void as3933_set_antenna_damper(r_val_t value);
void as3933_set_off_timer(t_off_t value);
void as3933_set_comparator_hysteresis(comp_hyst_t value);

//commands
void as3933_clear_wake_up();
void as3933_calibrate_rco_lc();
void as3933_reset_rssi();

bool as3933_rc_osc_self_calibrate();
#endif