#ifndef __AS3933_REGS_H__
#define __AS3933_REGS_H__

#include <stdint.h>

#define R0 0
#define R1 1
#define R2 2
#define R3 3
#define R4 4
#define R5 5
#define R6 6
#define R7 7
#define R8 8
#define R9 9
#define R10 10
#define R11 11
#define R12 12
#define R13 13
#define R14 14
#define R15 15
#define R16 16
#define R17 17
#define R18 18
#define R19 19

#define READ	0x40
#define WRITE	0x3F
#define DIRECT_COMMAND 0xC0


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
    enum
    {
        PAT16,
        PAT32
    } patt : 1;
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

typedef struct __attribute__((__packed__, aligned(1)))
{
    enum
    {
        SYMBOL_RATE_4096,
        SYMBOL_RATE_2184,
        SYMBOL_RATE_1490,
        SYMBOL_RATE_1130,
        SYMBOL_RATE_910,
        SYMBOL_RATE_762,
        SYMBOL_RATE_655,
        SYMBOL_RATE_512
    } fs_env : 3;
    enum
    {
        PREAMBLE_LEN_0_8MS = 0,
        PREAMBLE_LEN_1_15MS,
        PREAMBLE_LEN_1_55MS,
        PREAMBLE_LEN_1_9MS,
        PREAMBLE_LEN_2_3MS,
        PREAMBLE_LEN_2_65MS,
        PREAMBLE_LEN_3_0MS,
        PREAMBLE_LEN_3_5MS
    } fs_slc : 3;
    uint8_t hy_pos : 1;
    uint8_t hy_20m : 1;
} r3_t;

typedef struct __attribute__((__packed__, aligned(1)))
{
    enum
    {
        NO_GAIN = 0,
        GAIN_REDUCTION_4DB = 0b00000101,
        GAIN_REDUCTION_8DB = 0b00000111,
        GAIN_REDUCTION_12DB = 0b00001001,
        GAIN_REDUCTION_16DB = 0b00001011,
        GAIN_REDUCTION_20DB = 0b00001101,
        GAIN_REDUCTION_24DB = 0b00001111
    } gr : 4;
    enum
    {
        RESISTOR_1KOM,
        RESISTOR_3KOM,
        RESISTOR_9KOM,
        RESISTOR_27KOM
    } r_val : 2;
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

typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t t_hbit : 5;
    enum
    {
        TIMEOUT_0MS,
        TIMEOUT_50MS,
        TIMEOUT_100MS,
        TIMEOUT_150MS,
        TIMEOUT_200MS,
        TIMEOUT_250MS,
        TIMEOUT_300MS,
        TIMEOUT_350MS
    } t_out : 3;
} r7_t;

typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t t_auto : 3;
    uint8_t reserved : 2;
    enum
    {
        RANGE_95_150KHZ,
        RANGE_65_95KHZ,
        RANGE_40_65KHZ,
        RANGE_23_40KHZ,
        RANGE_15_23KHZ = 7
    } band_sel : 3;
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

#endif