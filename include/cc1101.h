/*
 * cc1101.h
 *
 *  Created on: 13 kwi 2019
 *      Author: kurza
 */

#ifndef MAIN_INCLUDE_CC1101_H_
#define MAIN_INCLUDE_CC1101_H_

#include "driver/spi_master.h"

#define CC1101_CLOCK_FREQUENCY (uint8_t)26

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

typedef enum CC1101_marc_state_e
{
    CC1101_STATE_SLEEP = 0,
    CC1101_STATE_IDLE,
    CC1101_STATE_XOFF,
    CC1101_STATE_VCOON_MC,
    CC1101_STATE_REGON_MC,
    CC1101_STATE_MANCAL,
    CC1101_STATE_VCOON,
    CC1101_STATE_REGON,
    CC1101_STATE_STARTCAL,
    CC1101_STATE_BWBOOST,
    CC1101_STATE_FS_LOCK,
    CC1101_STATE_IFADCON,
    CC1101_STATE_ENDCAL,
    CC1101_STATE_RX,
    CC1101_STATE_RX_END,
    CC1101_STATE_RX_RST,
    CC1101_STATE_TXRX_SWITCH,
    CC1101_STATE_RXFIFO_OVERFLOW,
    CC1101_STATE_FSTXON,
    CC1101_STATE_TX,
    CC1101_STATE_TX_END,
    CC1101_STATE_RXTX_SWITCH,
    CC1101_STATE_TXFIFO_UNDERFLOW
} CC1101_marc_state_t;

// Configuration Registers
#define CC1101_IOCFG2 0x00   // GDO2 output pin configuration
#define CC1101_IOCFG1 0x01   // GDO1 output pin configuration
#define CC1101_IOCFG0 0x02   // GDO0 output pin configuration
#define CC1101_FIFOTHR 0x03  // RX FIFO and TX FIFO thresholds
#define CC1101_SYNC1 0x04    // Sync word, high byte
#define CC1101_SYNC0 0x05    // Sync word, low byte
#define CC1101_PKTLEN 0x06   // Packet length
#define CC1101_PKTCTRL1 0x07 // Packet automation control
#define CC1101_PKTCTRL0 0x08 // Packet automation control
#define CC1101_ADDR 0x09     // Device address
#define CC1101_CHANNR 0x0A   // Channel number
#define CC1101_FSCTRL1 0x0B  // Frequency synthesizer control
#define CC1101_FSCTRL0 0x0C  // Frequency synthesizer control
#define CC1101_FREQ2 0x0D    // Frequency control word, high byte
#define CC1101_FREQ1 0x0E    // Frequency control word, middle byte
#define CC1101_FREQ0 0x0F    // Frequency control word, low byte
#define CC1101_MDMCFG4 0x10  // Modem configuration
#define CC1101_MDMCFG3 0x11  // Modem configuration
#define CC1101_MDMCFG2 0x12  // Modem configuration
#define CC1101_MDMCFG1 0x13  // Modem configuration
#define CC1101_MDMCFG0 0x14  // Modem configuration
#define CC1101_DEVIATN 0x15  // Modem deviation setting
#define CC1101_MCSM2 0x16    // Main Radio Cntrl State Machine config
#define CC1101_MCSM1 0x17    // Main Radio Cntrl State Machine config
#define CC1101_MCSM0 0x18    // Main Radio Cntrl State Machine config
#define CC1101_FOCCFG 0x19   // Frequency Offset Compensation config
#define CC1101_BSCFG 0x1A    // Bit Synchronization configuration
#define CC1101_AGCCTRL2 0x1B // AGC control
#define CC1101_AGCCTRL1 0x1C // AGC control
#define CC1101_AGCCTRL0 0x1D // AGC control
#define CC1101_WOREVT1 0x1E  // High byte Event 0 timeout
#define CC1101_WOREVT0 0x1F  // Low byte Event 0 timeout
#define CC1101_WORCTRL 0x20  // Wake On Radio control
#define CC1101_FREND1 0x21   // Front end RX configuration
#define CC1101_FREND0 0x22   // Front end TX configuration
#define CC1101_FSCAL3 0x23   // Frequency synthesizer calibration
#define CC1101_FSCAL2 0x24   // Frequency synthesizer calibration
#define CC1101_FSCAL1 0x25   // Frequency synthesizer calibration
#define CC1101_FSCAL0 0x26   // Frequency synthesizer calibration
#define CC1101_RCCTRL1 0x27  // RC oscillator configuration
#define CC1101_RCCTRL0 0x28  // RC oscillator configuration
#define CC1101_FSTEST 0x29   // Frequency synthesizer cal control
#define CC1101_PTEST 0x2A    // Production test
#define CC1101_AGCTEST 0x2B  // AGC test
#define CC1101_TEST2 0x2C    // Various test settings
#define CC1101_TEST1 0x2D    // Various test settings
#define CC1101_TEST0 0x2E    // Various test settings

// Strobe commands
// #define CC1101_SRES 0x30    // Reset chip.
// #define CC1101_SFSTXON 0x31 // Enable/calibrate freq synthesizer
// #define CC1101_SXOFF 0x32   // Turn off crystal oscillator.
// #define CC1101_SCAL 0x33    // Calibrate freq synthesizer & disable
// #define CC1101_SRX 0x34     // Enable RX.
// #define CC1101_STX 0x35     // Enable TX.
// #define CC1101_SIDLE 0x36   // Exit RX / TX
// #define CC1101_SAFC 0x37    // AFC adjustment of freq synthesizer
// #define CC1101_SWOR 0x38    // Start automatic RX polling sequence
// #define CC1101_SPWD 0x39    // Enter pwr down mode when CSn goes hi
// #define CC1101_SFRX 0x3A    // Flush the RX FIFO buffer.
// #define CC1101_SFTX 0x3B    // Flush the TX FIFO buffer.
// #define CC1101_SWORRST 0x3C // Reset real time clock.
// #define CC1101_SNOP 0x3D    // No operation.

typedef enum CC1101_command_strobe_e
{
    CC1101_SRES = 0x30,    // Reset chip.
    CC1101_SFSTXON = 0x31, // Enable/calibrate freq synthesizer
    CC1101_SXOFF = 0x32,   // Turn off crystal oscillator.
    CC1101_SCAL = 0x33,    // Calibrate freq synthesizer & disable
    CC1101_SRX = 0x34,     // Enable RX.
    CC1101_STX = 0x35,     // Enable TX.
    CC1101_SIDLE = 0x36,   // Exit RX / TX
    CC1101_SAFC = 0x37,    // AFC adjustment of freq synthesizer
    CC1101_SWOR = 0x38,    // Start automatic RX polling sequence
    CC1101_SPWD = 0x39,    // Enter pwr down mode when CSn goes hi
    CC1101_SFRX = 0x3A,    // Flush the RX FIFO buffer.
    CC1101_SFTX = 0x3B,    // Flush the TX FIFO buffer.
    CC1101_SWORRST = 0x3C, // Reset real time clock.
    CC1101_SNOP = 0x3D     // No operation.
} CC1101_command_strobe_t;

// Status registers
#define CC1101_PARTNUM 0x30        // Part number
#define CC1101_VERSION 0x31        // Current version number
#define CC1101_FREQEST 0x32        // Frequency offset estimate
#define CC1101_LQI 0x33            // Demodulator estimate for link quality
#define CC1101_RSSI 0x34           // Received signal strength indication
#define CC1101_MARCSTATE 0x35      // Control state machine state
#define CC1101_WORTIME1 0x36       // High byte of WOR timer
#define CC1101_WORTIME0 0x37       // Low byte of WOR timer
#define CC1101_PKTSTATUS 0x38      // Current GDOx status and packet status
#define CC1101_VCO_VC_DAC 0x39     // Current setting from PLL cal module
#define CC1101_TXBYTES 0x3A        // Underflow and # of bytes in TXFIFO
#define CC1101_RXBYTES 0x3B        // Overflow and # of bytes in RXFIFO
#define CC1101_RCCTRL1_STATUS 0x3C // Last RC Oscillator Calibration Result
#define CC1101_RCCTRL0_STATUS 0x3D // Last RC Oscillator Calibration Result
#define CC1101_NUM_RXBYTES 0x7F    // Mask "# of bytes" field in _RXBYTES

// Other memory locations
#define CC1101_PATABLE 0x3E
#define CC1101_TXFIFO 0x3F
#define CC1101_RXFIFO 0x3F

// Masks for appended status bytes
#define CC1101_LQI_RX 0x01 // Position of LQI byte
#define CC1101_CRC_OK 0x80 // Mask "CRC_OK" bit within LQI byte

// Definitions to support burst/single access:
#define CC1101_WRITE_BURST 0x40
#define CC1101_READ_SINGLE 0x80
#define CC1101_READ_BURST 0xC0

/*
 The CC1101 needs a bunch of command/argument values to be initialized. They are stored in this struct.
*/
typedef struct
{
    uint8_t cmd;
    uint8_t data;
} cc1101_init_cmd_t;

// typedef struct
// {
//     union {
//         uint8_t iocfg2;
//         struct
//         {
//             uint8_t unused : 1;
//             uint8_t gdo2_inv : 1;
//             uint8_t gdo2_cfg : 6;
//         };
//     };
// }

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

typedef union {
    uint8_t reg;
    struct
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

typedef union {
    uint8_t reg;
    struct
    {
        uint8_t drate_m : 8;
    };
} mdmcfg3_t;

typedef union __attribute__((__packed__, aligned(1))) {
    uint8_t reg;
    struct __attribute__((__packed__, aligned(1)))
    {
        uint8_t drate_e : 4;
        uint8_t chanbw_m : 2;
        uint8_t chanbw_e : 2;
    };
} mdmcfg4_t;

typedef union {
    uint8_t reg;
    struct
    {
        uint8_t pktlen_length : 8;
    };
} pktlen_t;

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

typedef union __attribute__((__packed__, aligned(1))) {
    uint8_t reg;
    struct __attribute__((__packed__, aligned(1)))
    {
        uint8_t device_addr;
    };
} addr_t;

typedef union __attribute__((__packed__, aligned(1))) {
    uint8_t reg;
    struct __attribute__((__packed__, aligned(1)))
    {
        uint8_t chan;
    };
} channr_t;

typedef enum
{
    ATTENUATION_0dB,
    ATTENUATION_6dB,
    ATTENUATION_12dB,
    ATTENUATION_18dB
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
    uint8_t reg;
    struct __attribute__((__packed__, aligned(1)))
    {
        uint8_t pa_power : 3;
        uint8_t _reserved_3 : 1;
        uint8_t lodiv_buf_current_tx : 2;
        uint8_t _reserved_6_7 : 2;
    };
} frend0_t;

typedef union __attribute__((__packed__, aligned(1))) {
    uint16_t sync;
    struct
    {
        uint8_t sync0;
        uint8_t sync1;
    };
} sync_word_t;

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

typedef union __attribute__((__packed__, aligned(1))) {
    uint8_t reg;
    struct __attribute__((__packed__, aligned(1)))
    {
        magn_target_t magn_target : 3;
        uint8_t max_lna_gain : 3;
        uint8_t max_dvga_gain : 2;
    };
} agcctrl2_t;

void set_carrier_frequency(spi_device_handle_t spi, double frequencyInMHz);
double get_carrier_frequency(spi_device_handle_t spi);

void set_baud_rate(spi_device_handle_t spi, double baud_rate_in_khz);
double get_baud_rate(spi_device_handle_t spi);

void set_rx_channel_bandwidth(spi_device_handle_t spi, uint16_t bandwidth);
uint16_t get_rx_channel_bandwidth(spi_device_handle_t spi);

void set_modulation_format(spi_device_handle_t spi, mod_format_t modFormat);
mod_format_t get_modulation_format(spi_device_handle_t spi);

void set_manchester_encoding(spi_device_handle_t spi, uint8_t manchester_en);
uint8_t get_manchester_encoding(spi_device_handle_t spi);

void set_sync_mode(spi_device_handle_t spi, sync_mode_t sync_mode);
sync_mode_t get_sync_mode(spi_device_handle_t spi);

void set_packet_length(spi_device_handle_t spi, uint8_t packet_length);
uint8_t get_packet_length(spi_device_handle_t spi);

void set_crc_enable(spi_device_handle_t spi, uint8_t crc_en);
uint8_t get_crc_enable(spi_device_handle_t spi);

void set_white_data(spi_device_handle_t spi, uint8_t white_data);
uint8_t get_white_data(spi_device_handle_t spi);

void set_pkt_format(spi_device_handle_t spi, pkt_format_t pkt_format);
pkt_format_t get_pkt_format(spi_device_handle_t spi);

void set_length_config(spi_device_handle_t spi, pck_length_config_t pck_length_config);
pck_length_config_t get_length_config(spi_device_handle_t spi);

void set_txoff_mode(spi_device_handle_t spi, txoff_mode_t txoff_mode);
txoff_mode_t get_txoff_mode(spi_device_handle_t spi);

void set_rxoff_mode(spi_device_handle_t spi, rxoff_mode_t rxoff_mode);
rxoff_mode_t get_rxoff_mode(spi_device_handle_t spi);

void set_cca_mode(spi_device_handle_t spi, cca_mode_t cca_mode);
cca_mode_t get_cca_mode(spi_device_handle_t spi);

void set_sync_word(spi_device_handle_t spi, sync_word_t sync_word);
sync_word_t get_sync_word(spi_device_handle_t spi);

void set_forward_error_correction(spi_device_handle_t spi, uint8_t fec_en);
uint8_t get_forward_error_correction(spi_device_handle_t spi);

void set_num_preamble(spi_device_handle_t spi, num_preamble_t num_preamble);
num_preamble_t get_num_preamble(spi_device_handle_t spi);

void set_deviation_frequency(spi_device_handle_t spi, double frequencyInkHz);
double get_deviation_frequency(spi_device_handle_t spi);

void set_controls_address_check(spi_device_handle_t spi, adr_chk_t adr_chk);
adr_chk_t get_controls_address_check(spi_device_handle_t spi);

void set_append_status(spi_device_handle_t spi, uint8_t append_status);
uint8_t get_append_status(spi_device_handle_t spi);

void set_preamble_quality_threshold(spi_device_handle_t spi, uint8_t pqt);
uint8_t get_preamble_quality_threshold(spi_device_handle_t spi);

void set_crc_autoflush(spi_device_handle_t spi, uint8_t crc_autoflush);
uint8_t get_crc_autoflush(spi_device_handle_t spi);

void set_device_address(spi_device_handle_t spi, uint8_t device_addr);
uint8_t get_device_address(spi_device_handle_t spi);

void set_channel_number(spi_device_handle_t spi, uint8_t chan);
uint8_t get_channel_number(spi_device_handle_t spi);

void set_rx_attenuation(spi_device_handle_t spi, close_in_rx_t close_in_rx);
uint8_t get_rx_attenuation(spi_device_handle_t spi);

void set_pa_power_setting(spi_device_handle_t spi, uint8_t pa_power);
uint8_t get_pa_power_setting(spi_device_handle_t spi);

void set_filter_length(spi_device_handle_t spi, filter_length_t filter_length);
uint8_t get_filter_length(spi_device_handle_t spi);

void set_hysteresis_level(spi_device_handle_t spi, hyst_level_t hyst_level);
hyst_level_t get_hysteresis_level(spi_device_handle_t spi);

void set_magn_target(spi_device_handle_t spi, magn_target_t magn_target);
magn_target_t get_magn_target(spi_device_handle_t spi);

uint8_t get_rssi(spi_device_handle_t spi);
float get_rssi_dbm(spi_device_handle_t spi);

uint8_t get_tx_fifo_info(spi_device_handle_t spi);

void reset(spi_device_handle_t spi);
void set_idle_state(spi_device_handle_t spi);
void set_flush_tx(spi_device_handle_t spi);
void set_flush_rx(spi_device_handle_t spi);
void set_tx_state(spi_device_handle_t spi);
void set_rx_state(spi_device_handle_t spi);
void set_calibrate_state(spi_device_handle_t spi);

void init(spi_device_handle_t spi);
void set_pa_table(spi_device_handle_t spi);
void get_pa_table(spi_device_handle_t spi);

void send_data(spi_device_handle_t spi);

#endif /* MAIN_INCLUDE_CC1101_H_ */
