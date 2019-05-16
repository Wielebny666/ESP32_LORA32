#ifndef __AS3933_H__
#define __AS3933_H__

#include <stdint.h>

typedef struct
{
    uint8_t cmd;
    uint8_t data;
} as3933_init_cmd_t;

typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t unused : 1;
    uint8_t en_a1 : 1;
    uint8_t en_a3 : 1;
    uint8_t en_a2 : 1;
    uint8_t mux_123 : 1;
    uint8_t on_off : 1;
    uint8_t dat_mask : 1;
    uint8_t patt : 1;
} r0_t;

typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t en_xtal : 1;
    uint8_t en_wpat : 1;
    uint8_t en_pat2 : 1;
    uint8_t en_manch : 1;
    uint8_t att_on : 1;
    uint8_t agc_ud : 1;
    uint8_t agc_tlim : 1;
    uint8_t abs_hy : 1;
} r1_t;

typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t s_wu1 : 2;
    uint8_t display_clk : 2;
    uint8_t reserved : 1;
    uint8_t g_boost : 1;
    uint8_t en_ext_clk : 1;
    uint8_t s_abs : 1;
} r2_t;

typedef enum
{
    SYMBOL_RATE_4096,
    SYMBOL_RATE_2184,
    SYMBOL_RATE_1490,
    SYMBOL_RATE_1130,
    SYMBOL_RATE_910,
    SYMBOL_RATE_762,
    SYMBOL_RATE_655,
    SYMBOL_RATE_512
} fs_env_t;

typedef enum
{
    PREAMBLE_LEN_0_8MS = 0,
    PREAMBLE_LEN_1_15MS,
    PREAMBLE_LEN_1_55MS,
    PREAMBLE_LEN_1_9MS,
    PREAMBLE_LEN_2_3MS,
    PREAMBLE_LEN_2_65MS,
    PREAMBLE_LEN_3_0MS,
    PREAMBLE_LEN_3_5MS
} fs_slc_t;

typedef struct __attribute__((__packed__, aligned(1)))
{
    fs_env_t fs_env : 3;
    fs_slc_t fs_slc : 3;
    uint8_t hy_pos : 1;
    uint8_t hy_20m : 1;
} r3_t;

typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t gr : 4;
    uint8_t r_val : 2;
    uint8_t t_off : 2;
} r4_t;

typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t patt2b;
} r5_t;

typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t patt1b;
} r6_t;

typedef enum
{
    TIMEOUT_0MS,
    TIMEOUT_50MS,
    TIMEOUT_100MS,
    TIMEOUT_150MS,
    TIMEOUT_200MS,
    TIMEOUT_250MS,
    TIMEOUT_300MS,
    TIMEOUT_350MS
} t_out_t;

typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t t_hbit : 5;
    t_out_t t_out : 3;
} r7_t;

typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t t_auto : 3;
    uint8_t reserved : 2;
    uint8_t band_sel : 3;
} r8_t;

typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t reserved : 7;
    uint8_t block_agc : 1;
} r9_t;

typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t rssi1 : 5;
    uint8_t reserved : 3;
} r10_t;

typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t rssi2 : 5;
    uint8_t reserved : 3;
} r11_t;

typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t rssi3 : 5;
    uint8_t reserved : 3;
} r12_t;

typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t f_wake;
} r13_t;

typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t rc_osc_taps : 6;
    uint8_t rc_cal_ko : 1;
    uint8_t rc_cal_ok : 1;
} r14_t;

typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t reserved : 3;
    uint8_t lc_osc_ko : 1;
    uint8_t lc_osc_ok : 1;
} r15_t;

typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t lc_osc_mux : 3;
    uint8_t n_a_4 : 1;
    uint8_t rc_osc_max : 1;
    uint8_t rc_osc_min : 1;
    uint8_t n_a_6 : 1;
    uint8_t clock_gen_dis : 1;
} r16_t;

typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t cap_ch1 : 5;
    uint8_t n_a_4 : 1;
} r17_t;

typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t cap_ch2 : 5;
    uint8_t n_a_4 : 1;
} r18_t;

typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t cap_ch3 : 5;
    uint8_t n_a_4 : 1;
} r19_t;

typedef enum
{
    CLEAR_WAKE = 0,
    RESET_RSSI = 1,
    CALIB_RCOSC = 2,
    CLEAR_FALSE = 3,
    PRESET_DEFAULT = 4,
    CALIB_RCO_LC = 5
} cmd_t;

void as3933_spi_init();
void as3933_cmd(cmd_t cmd);
void as3933_write(uint8_t addr, uint8_t data);
uint8_t as3933_read(uint8_t addr);
void as3933_write_buffer(uint16_t addr, uint8_t *buffer, uint8_t size);
void as3933_read_buffer(uint16_t addr, uint8_t *buffer, uint8_t size);

void as3933_reset();

#endif