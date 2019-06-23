
#ifndef __CC1101_REGS_H__
#define __CC1101_REGS_H__

#include <stdint.h>

typedef enum
{
    GDOx_RX_FIFO = 0x00,                  /* assert above threshold, deassert when below         */
    GDOx_RX_FIFO_EOP = 0x01,              /* assert above threshold or EOP                       */
    GDOx_TX_FIFO = 0x02,                  /* assert above threshold, deassert below thr          */
    GDOx_TX_THR_FULL = 0x03,              /* asserts when TX FIFO is full. De-asserts when       */
                                          /* the TX FIFO is drained below TXFIFO_THR.            */
    GDOx_RX_OVER = 0x04,                  /* asserts when RX overflow, deassert when flushed     */
    GDOx_TX_UNDER = 0x05,                 /* asserts when RX underflow, deassert when flushed    */
    GDOx_SYNC_WORD = 0x06,                /* assert SYNC sent/recv, deasserts on EOP             */
                                          /* In RX, de-assert on overflow or bad address         */
                                          /* In TX, de-assert on underflow                       */
    GDOx_RX_OK = 0x07,                    /* assert when RX PKT with CRC ok, de-assert on 1byte  */
                                          /* read from RX Fifo                                   */
    GDOx_PREAMB_OK = 0x08,                /* assert when preamble quality reached : PQI/PQT ok   */
    GDOx_CCA = 0x09,                      /* Clear channel assessment. High when RSSI level is   */
                                          /* below threshold (dependent on the current CCA_MODE) */
    GDOx_PLL = 0x0A,                      /*Lock detector output. The PLL is in lock if the lock detector output has a positive transition or is constantly logic high.*/
                                          /*To check for PLL lock the lock detector output should be used as an interrupt for the MCU.*/
    GDOx_SERIAL_SYNC_CLOCK = 0x11,        /*Serial Clock. Synchronous to the data in synchronous serial mode.*/
                                          /*In RX mode, data is set up on the falling edge by CC1101 when GDOx_INV=0.*/
                                          /*In TX mode, data is sampled by CC1101 on the rising edge of the serial clock when GDOx_INV=0.*/
    GDOx_SERIAL_SYNC_DATA_OUTPUT = 0x0C,  /*Serial Synchronous Data Output. Used for synchronous serial mode.*/
    GDOx_SERIAL_ASYNC_DATA_OUTPUT = 0x0D, /*Serial Data Output. Used for asynchronous serial mode.*/
    GDOx_RSSI_OK = 0x0E,                  /*Carrier sense. High if RSSI level is above threshold. Cleared when entering IDLE mode.*/
    GDOx_CRC_MATCH = 0x0F,                /*CRC_OK. The last CRC comparison matched. Cleared when entering/restarting RX mode.*/
    GDOx_CHIP_RDY = 0x29,                 /* CHIP_RDY     */
    GDOx_XOSC_STABLE = 0x2B,              /* XOSC_STABLE  */
    GDOx_CLK_XOSC_1 = 0x30,               /* CLK_XOSC/1   */
    GDOx_CLK_XOSC_1p5 = 0x31,             /* CLK_XOSC/1.5 */
    GDOx_CLK_XOSC_2 = 0x32,               /* CLK_XOSC/2   */
    GDOx_CLK_XOSC_3 = 0x33,               /* CLK_XOSC/3   */
    GDOx_CLK_XOSC_4 = 0x34,               /* CLK_XOSC/4   */
    GDOx_CLK_XOSC_6 = 0x35,               /* CLK_XOSC/6   */
    GDOx_CLK_XOSC_8 = 0x36,               /* CLK_XOSC/8   */
    GDOx_CLK_XOSC_12 = 0x37,              /* CLK_XOSC/12  */
    GDOx_CLK_XOSC_16 = 0x38,              /* CLK_XOSC/16  */
    GDOx_CLK_XOSC_24 = 0x39,              /* CLK_XOSC/24  */
    GDOx_CLK_XOSC_32 = 0x3A,              /* CLK_XOSC/32  */
    GDOx_CLK_XOSC_48 = 0x3B,              /* CLK_XOSC/48  */
    GDOx_CLK_XOSC_64 = 0x3C,              /* CLK_XOSC/64  */
    GDOx_CLK_XOSC_96 = 0x3D,              /* CLK_XOSC/96  */
    GDOx_CLK_XOSC_128 = 0x3E,             /* CLK_XOSC/128 */
    GDOx_CLK_XOSC_192 = 0x3F              /* CLK_XOSC/192 */
} gdox_cfg_t;

typedef union __attribute__((__packed__, aligned(1))) {
    uint8_t reg;
    struct __attribute__((__packed__, aligned(1)))
    {
        gdox_cfg_t gdox_cfg : 6;
        uint8_t gdox_inv : 1;
        uint8_t _reserved_7 : 1;
    };
} iocfgx_t;

typedef enum
{
    ATTENUATION_0DB,
    ATTENUATION_6DB,
    ATTENUATION_12DB,
    ATTENUATION_18DB
} close_in_rx_t;

typedef union __attribute__((__packed__, aligned(1))) {
    uint8_t reg;
    struct __attribute__((__packed__, aligned(1)))
    {
        uint8_t fifo_thr : 4;
        close_in_rx_t close_in_rx : 2;
        uint8_t adc_retention : 1;
        uint8_t _reserved_7 : 1;
    };
} fifothr_t;

typedef union __attribute__((__packed__, aligned(1))) {
    uint16_t sync;
    struct __attribute__((__packed__, aligned(1)))
    {
        uint8_t sync0;
        uint8_t sync1;
    };
} sync_word_t;

typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t sync_15_8;
} sync1_t;

typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t sync_7_0;
} sync0_t;

typedef struct
{
    uint8_t pktlen_length;
} pktlen_t;

typedef enum
{
    ADDR_NO_CHECK,
    ADDR_CHECK_NO_BROADCAST,
    ADDR_CHECK_BROADCAST_0,
    ADDR_CHECK_NO_BROADCAST_0_255
} adr_chk_t;

typedef union __attribute__((__packed__, aligned(1))) {
    uint8_t reg;
    struct __attribute__((__packed__, aligned(1)))
    {
        adr_chk_t adr_chk : 2;
        uint8_t append_status : 1;
        uint8_t crc_autoflush : 1;
        uint8_t _reserved_4 : 1;
        uint8_t pqt : 3;
    };
} pktctrl1_t;

typedef enum pkt_format_e
{
    PKT_FORMAT_NORMAL,
    PKT_FORMAT_SYNC_SERIAL,
    PKT_FORMAT_RANDOM,
    PKT_FORMAT_ASYNC_SERIAL
} pkt_format_t;

typedef enum pck_length_config_e
{
    LENGTH_FIXED,
    LENGTH_VARIABLE,
    LENGTH_INFINITE,
    LENGTH_RESERVERD
} pck_length_config_t;

typedef union __attribute__((__packed__, aligned(1))) {
    uint8_t reg;
    struct __attribute__((__packed__, aligned(1)))
    {
        pck_length_config_t pck_length_config : 2;
        uint8_t crc_en : 1;
        uint8_t _reserved_3 : 1;
        pkt_format_t pkt_format : 2;
        uint8_t white_data : 1;
        uint8_t _reserved_7 : 1;
    };
} pktctrl0_t;

// Device Address
typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t device_addr;
} addr_t;

// Channel Number
typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t chan;
} channr_t;

// Frequency Synthesizer Control
typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t freq_if : 5;
} fsctrl1_t;

// Frequency Synthesizer Control
typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t freqoff;
} fsctrl0_t;

// Frequency Control Word, High Byte
typedef union __attribute__((__packed__, aligned(1))) {
    uint8_t reg;
    struct __attribute__((__packed__, aligned(1)))
    {
        uint8_t freq_16_21 : 6;
        uint8_t freq_22_23 : 2;
    };
} freq2_t;

// Frequency Control Word, Mid Byte
typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t freq_8_15;
} freq1_t;

// Frequency Control Word, Low Byte
typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t freq_0_7;
} freq0_t;

// Modem Configuration
typedef union __attribute__((__packed__, aligned(1))) {
    uint8_t reg;
    struct __attribute__((__packed__, aligned(1)))
    {
        uint8_t drate_e : 4;
        uint8_t chanbw_m : 2;
        uint8_t chanbw_e : 2;
    };
} mdmcfg4_t;

// Modem Configuration
typedef struct
{
    uint8_t drate_m;
} mdmcfg3_t;

/* Sync word type */
typedef enum
{
    NO_SYNC = 0,             // No preamble/sync
    SYNC_15_OVER_16,         // 15/16 sync word bits detected
    SYNC_16_OVER_16,         // 16/16 sync word bits detected
    SYNC_30_over_32,         // 30/32 sync word bits detected
    SYNC_CARRIER,            // No preamble/sync, carrier-sense above threshold
    SYNC_15_OVER_16_CARRIER, // 15/16 + carrier-sense above threshold
    SYNC_16_OVER_16_CARRIER, // 16/16 + carrier-sense above threshold
    SYNC_30_over_32_CARRIER  // 30/32 + carrier-sense above threshold
} sync_mode_t;

/* Different modulation types */
typedef enum
{
    RADIO_MOD_2FSK = 0,
    RADIO_MOD_GFSK = 1,
    RADIO_MOD_OOK = 3,
    RADIO_MOD_4FSK = 4,
    RADIO_MOD_MSK = 7
} mod_format_t;

// Modem Configuration
typedef union __attribute__((__packed__, aligned(1))) {
    uint8_t reg;
    struct __attribute__((__packed__, aligned(1)))
    {
        sync_mode_t sync_mode : 3;
        uint8_t manchester_en : 1;
        mod_format_t mod_format : 3;
        uint8_t dem_dcfilt_off : 1;
    };
} mdmcfg2_t;

typedef enum
{
    PREAMBLE_2,
    PREAMBLE_3,
    PREAMBLE_4,
    PREAMBLE_6,
    PREAMBLE_8,
    PREAMBLE_12,
    PREAMBLE_16,
    PREAMBLE_24
} num_preamble_t;

// Modem Configuration
typedef union __attribute__((__packed__, aligned(1))) {
    uint8_t reg;
    struct __attribute__((__packed__, aligned(1)))
    {
        uint8_t chanspc_e : 2;
        uint8_t _reserved_2_3 : 2;
        num_preamble_t num_preamble : 3;
        uint8_t fec_en : 1;
    };
} mdmcfg1_t;

// Modem Configuration
typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t chanspc_m;
} mdmcfg0_t;

// Modem Deviation Setting
typedef union __attribute__((__packed__, aligned(1))) {
    uint8_t reg;
    struct __attribute__((__packed__, aligned(1)))
    {
        uint8_t deviation_m : 3;
        uint8_t _reserved_3 : 1;
        uint8_t deviation_e : 3;
        uint8_t _reserved_7 : 1;
    };
} deviatn_t;

// Main Radio Control State Machine Configuration
typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t mcsm2;
} mcsm2_t;

/*Clear channel indication*/
typedef enum
{
    CCA_MODE_ALWAYS = 0,
    CCA_MODE_RSSI_BELOW_TH,
    CCA_MODE_PKT_RX,
    CCA_MODE_RSSI__BELOW_TH_OR_PKT_RX,
} cca_mode_t;

typedef enum
{
    TXOFF_MODE_IDLE = 0,
    TXOFF_MODE_FSTXON,
    TXOFF_MODE_STAY_TX,
    TXOFF_MODE_RX
} txoff_mode_t;

typedef enum
{
    RXOFF_MODE_IDLE = 0,
    RXOFF_MODE_FSTXON,
    RXOFF_MODE_TX,
    RXOFF_MODE_STAY_RX
} rxoff_mode_t;

// Main Radio Control State Machine Configuration
typedef union __attribute__((__packed__, aligned(1))) {
    uint8_t reg;
    struct __attribute__((__packed__, aligned(1)))
    {
        txoff_mode_t txoff_mode : 2;
        rxoff_mode_t rxoff_mode : 2;
        cca_mode_t cca_mode : 2;
        uint8_t _reserved_6_7 : 2;
    };
} mcsm1_t;

typedef enum
{
    TIMEOUT_1,
    TIMEOUT_16,
    TIMEOUT_64,
    TIMEOUT_256
} po_timeout_t;

typedef enum
{
    AUTOCAL_NEVER,
    AUTOCAL_IDLE_TO_RX_TX,
    AUTOCAL_RX_TX_TO_IDLE,
    AUTOCAL_RX_TX_TO_IDLE_EVERY_4TH
} fs_autocal_t;

// Main Radio Control State Machine Configuration
typedef union __attribute__((__packed__, aligned(1))) {
    uint8_t reg;
    struct __attribute__((__packed__, aligned(1)))
    {
        uint8_t xosc_force_on : 1;
        uint8_t pin_ctrl_en : 1;
        po_timeout_t po_timeout : 2;
        fs_autocal_t fs_autocal : 2;
        uint8_t _reserved_6_7 : 2;
    };
} mcsm0_t;

// Frequency Offset Compensation Configuration
typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t foccfg;

} foccfg_t;

// Bit Synchronization Configuration
typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t bscfg;
} bscfg_t;

typedef enum
{
    MAGN_24DB = 0,
    MAGN_27DB,
    MAGN_30DB,
    MAGN_33DB,
    MAGN_36DB,
    MAGN_38DB,
    MAGN_40DB,
    MAGN_42DB
} magn_target_t;

// AGC Control
typedef union __attribute__((__packed__, aligned(1))) {
    uint8_t reg;
    struct __attribute__((__packed__, aligned(1)))
    {
        magn_target_t magn_target : 3;
        uint8_t max_lna_gain : 3;
        uint8_t max_dvga_gain : 2;
    };
} agcctrl2_t;

// AGC Control
typedef union __attribute__((__packed__, aligned(1))) {
    uint8_t reg;
    struct __attribute__((__packed__, aligned(1)))
    {
        uint8_t carrier_sense_abs_thr : 4;
        uint8_t carrier_sense_rel_thr : 2;
        uint8_t agc_lna_priority : 1;
        uint8_t _reserved_7 : 1;
    };
} agcctrl1_t;

typedef enum
{
    FILTER_4DB = 0,
    FILTER_8DB,
    FILTER_12DB,
    FILTER_16DB,
} filter_length_t;

typedef enum
{
    HYSTERESIS_NONE = 0,
    HYSTERESIS_LOW,
    HYSTERESIS_MEDIUM,
    HYSTERESIS_LARGE,
} hyst_level_t;

// AGC Control
typedef union __attribute__((__packed__, aligned(1))) {
    uint8_t reg;
    struct __attribute__((__packed__, aligned(1)))
    {
        filter_length_t filter_length : 2;
        uint8_t agc_freeze : 2;
        uint8_t wait_time : 2;
        hyst_level_t hyst_level : 2;
    };
} agcctrl0_t;

// Low Byte Event0 Timeout
typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t event0;
} worevt0_t;

// Wake On Radio Control
typedef union __attribute__((__packed__, aligned(1))) {
    uint8_t reg;
    struct __attribute__((__packed__, aligned(1)))
    {
        uint8_t wor_res : 2;
        uint8_t _reserved_2 : 1;
        uint8_t rc_cal : 1;
        uint8_t event1 : 3;
        uint8_t rc_pd : 1;
    };
} worctrl_t;

// Front End RX Configuration
typedef union __attribute__((__packed__, aligned(1))) {
    uint8_t reg;
    struct __attribute__((__packed__, aligned(1)))
    {
        uint8_t mix_current : 2;
        uint8_t lodiv_buf_current_rx : 2;
        uint8_t lna2mix_current : 2;
        uint8_t lna_current : 2;
    };
} frend1_t;

// Front End TX Configuration
typedef union __attribute__((__packed__, aligned(1))) {
    uint8_t reg;
    struct __attribute__((__packed__, aligned(1)))
    {
        uint8_t pa_power : 3;
        uint8_t _reserved_3 : 1;
        uint8_t lodiv_buf_current_tx : 2;
        uint8_t _reserved_6_7 : 2;
    };
} frend0_t;

// Frequency Synthesizer Calibration
typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t fscal3;
} fscal3_t;

// Frequency Synthesizer Calibration
typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t fscal2;
} fscal2_t;

// Frequency Synthesizer Calibration
typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t fscal1;
} fscal1_t;

// Frequency Synthesizer Calibration
typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t fscal0;
} fscal0_t;

// RC Oscillator Configuration
typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t rcctrl1;
} rcctrl1_t;

// RC Oscillator Configuration
typedef struct __attribute__((__packed__, aligned(1)))
{
    uint8_t rcctrl0;
} rcctrl0_t;

/* Available data rates */
typedef enum rate_e
{
    RATE_50,
    RATE_110,
    RATE_300,
    RATE_600,
    RATE_1200,
    RATE_2400,
    RATE_4800,
    RATE_9600,
    RATE_14400,
    RATE_19200,
    RATE_28800,
    RATE_38400,
    RATE_57600,
    RATE_76800,
    RATE_115200,
    NUM_RATE
} rate_t;

typedef struct __attribute__((__packed__, aligned(1)))
{
    iocfgx_t iocfg2;
    iocfgx_t iocfg1;
    iocfgx_t iocfg0;
    fifothr_t fifothr;
    sync1_t sync1;
    sync0_t sync0;
    pktlen_t pktlen;
    pktctrl1_t pktctrl1;
    pktctrl0_t pktctrl0;
    addr_t addr;
    channr_t channr;
    fsctrl1_t fsctrl1;
    fsctrl0_t fsctrl0;
    freq2_t freq2;
    freq1_t freq1;
    freq0_t freq0;
    mdmcfg4_t mdmcfg4;
    mdmcfg3_t mdmcfg3;
    mdmcfg2_t mdmcfg2;
    mdmcfg1_t mdmcfg1;
    mdmcfg0_t mdmcfg0;
    deviatn_t deviatn;
    mcsm2_t mcsm2;
    mcsm1_t mcsm1;
    mcsm0_t mcsm0;
    foccfg_t foccfg;
    bscfg_t bscfg;
    agcctrl2_t agcctrl2;
    agcctrl1_t agcctrl1;
    agcctrl0_t agcctrl0;
} cc1101_t;

extern cc1101_t cc1101;

#endif