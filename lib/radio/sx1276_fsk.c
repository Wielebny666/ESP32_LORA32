#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "sx1276_fsk_regs.h"
#include "sx1276_fsk_defs.h"
#include "sx1276_fsk_misc.h"
#include "sx1276_fsk.h"
#include "sx1276_hal.h"
#include "../include/radio.h"

// Default settings
fsk_settings_t fsk_settings =
    {
        434000000, // RFFrequency
        9600,      // Bitrate
        50000,     // Fdev
        20,        // Power
        100000,    // RxBw
        150000,    // RxBwAfc
        false,     // CrcOn
        false,      // AfcOn
        255        // PayloadLength (set payload size to the maximum for variable mode, else set the exact payload length)
};

/*!
 * SX1276 FSK registers variable
 */
sx1276_reg_t *sx1276;

/*!
 * Local RF buffer for communication support
 */
static uint8_t RFBuffer[RF_BUFFER_SIZE];

/*!
 * Chunk size of data write in buffer 
 */
static uint8_t data_chunk_size = 32;

/*!
 * RF state machine variable
 */
static uint8_t RFState = RF_STATE_IDLE;

/*!
 * Rx management support variables
 */

/*!
 * PacketTimeout holds the RF packet timeout
 * SyncSize = [0..8]
 * VariableSize = [0;1]
 * AddressSize = [0;1]
 * PayloadSize = [0..RF_BUFFER_SIZE]
 * CrcSize = [0;2]
 * PacketTimeout = ( ( 8 * ( VariableSize + AddressSize + PayloadSize + CrcSize ) / BR ) * 1000.0 ) + 1
 * Computed timeout is in miliseconds
 */
static uint32_t PacketTimeout;

/*!
 * Preamble2SyncTimeout
 * Preamble2SyncTimeout = ( ( 8 * ( PremableSize + SyncSize ) / RFBitrate ) * 1000.0 ) + 1
 * Computed timeout is in miliseconds
 */
static uint32_t Preamble2SyncTimeout;

static bool PreambleDetected = false;
static bool SyncWordDetected = false;
static bool PacketDetected = false;
static uint16_t rx_packet_size = 0;
static uint8_t RxBytesRead = 0;
static uint8_t tx_bytes_sent = 0;
static double RxPacketRssiValue;
static uint32_t RxPacketAfcValue;
static uint8_t RxGain = 1;
static uint32_t RxTimeoutTimer = 0;
static uint32_t Preamble2SyncTimer = 0;

/*!
 * Tx management support variables
 */
static uint16_t tx_packet_size = 0;
static uint32_t TxTimeoutTimer = 0;

static const char *TAG = "sx1276_fsk";

void sx1276_fsk_init(void)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    RFState = RF_STATE_IDLE;

    //SX1276FskSetDefaults( );

    sx1276_read_buffer(REG_OPMODE, (uint8_t *)sx1276 + 1, 0x70 - 1);

    // Set the device in FSK mode and Sleep Mode
    sx1276->RegOpMode.bits.LowFrequencyModeOn = true;
    sx1276->RegOpMode.bits.ModulationType = MODULATIONTYPE_OOK;
    //sx1276->RegOpMode.bits.LongRangeMode = LORA;
    sx1276->RegOpMode.bits.Mode = OPMODE_SLEEP;
    sx1276_write(REG_OPMODE, sx1276->RegOpMode.reg);

    sx1276->RegPaRamp.bits.ModulationShaping = NO_SHAPING;
    sx1276->RegPaRamp.bits.PaRamp = PARAMP_0010_US;// _3400_US;
    sx1276_write(REG_PARAMP, sx1276->RegPaRamp.reg);

    sx1276->RegLna.bits.LnaGain = LNA_GAIN_G1;
    sx1276_write(REG_LNA, sx1276->RegLna.reg);

    if (fsk_settings.AfcOn == true)
    {
        sx1276->RegRxConfig.bits.AfcAutoOn = true;
    }
    else
    {
        sx1276->RegRxConfig.bits.AfcAutoOn = false;
    }

    sx1276->RegRxConfig.bits.AgcAutoOn = false;
    sx1276->RegRxConfig.bits.RestartRxOnCollision = false;
    sx1276->RegRxConfig.bits.RxTrigger = RXTRIGER_PREAMBLEDETECT;

    sx1276->RegPreamble.RegPreambleMsb = 0;
    sx1276->RegPreamble.RegPreambleLsb = 0x08;

    sx1276->RegPreambleDetect.bits.PreambleDetectorOn = true;
    sx1276->RegPreambleDetect.bits.PreambleDetectorSize = DETECTORSIZE_2;
    sx1276->RegPreambleDetect.bits.PreambleDetectorTol = 0x0A;

    sx1276->RegRssiThresh.reg = 0xFF;

    sx1276->RegSyncConfig.bits.AutoRestartRxMode = 2;
    sx1276->RegSyncConfig.bits.PreamblePolarity = PREAMBLE_0xAA;
    sx1276->RegSyncConfig.bits.SyncOn = false;
    sx1276->RegSyncConfig.bits.SyncSize = SYNCSIZE_4;

    sx1276->RegSyncValue.SyncValue1.SyncValue = 0x69;
    sx1276->RegSyncValue.SyncValue2.SyncValue = 0x81;
    sx1276->RegSyncValue.SyncValue3.SyncValue = 0x7E;
    sx1276->RegSyncValue.SyncValue4.SyncValue = 0x96;

    sx1276->RegPacketConfig1.bits.PacketFormat = FIXED_LENGTH; // VARIABLE_LENGTH; //
    sx1276->RegPacketConfig1.bits.DcFree = NONE;
    sx1276->RegPacketConfig1.bits.CrcAutoClearOff = false;
    sx1276->RegPacketConfig1.bits.AddressFiltering = false;
    sx1276->RegPacketConfig1.bits.CrcWhiteningType = false;
    sx1276->RegPacketConfig1.bits.CrcOn = fsk_settings.CrcOn;

    //sx1276->RegDioMapping1.bits.Dio2Mapping = 1;
    sx1276->RegPacketConfig2.bits.DataMode = PACKET_MODE; // CONTINUOUS_MODE;
    sx1276->RegOokPeak.bits.BitSyncOn = 0;

    //     SX1276FskGetPacketCrcOn( ); // Update CrcOn on FskSettings

    sx1276->RegPayloadLength.reg = fsk_settings.PayloadLength;

    // we can now update the registers with our configuration
    sx1276_write_buffer(REG_OPMODE, (uint8_t *)sx1276 + 1, 0x70 - 1);

    // then we need to set the RF settings
    sx1276_fsk_set_rf_frequency(fsk_settings.RFFrequency);
    sx1276_fsk_set_bitrate(fsk_settings.Bitrate);
    sx1276_fsk_set_fdev(fsk_settings.Fdev);

    sx1276_fsk_set_dcc_bw((uint8_t *)&sx1276->RegRxBw, 0, fsk_settings.RxBw);
    sx1276_fsk_set_dcc_bw((uint8_t *)&sx1276->RegAfcBw, 0, fsk_settings.RxBwAfc);
    sx1276_fsk_set_rssi_offset(0);

    // #if( ( MODULE_SX1276RF1IAS == 1 ) || ( MODULE_SX1276RF1KAS == 1 ) )
    // if( FskSettings.RFFrequency > 860000000 )
    // {
    //     SX1276FskSetPAOutput( RF_PACONFIG_PASELECT_RFO );
    //     SX1276FskSetPa20dBm( false );
    //     FskSettings.Power = 14;
    //     SX1276FskSetRFPower( FskSettings.Power );
    // }
    // else
    // {
    sx1276_fsk_set_pa_output(RF_PACONFIG_PASELECT_PABOOST);
    //sx1276_fsk_set_pa_output(RF_PACONFIG_PASELECT_RFO);
    sx1276_fsk_set_pa_20dBm(true);
    fsk_settings.Power = 20;
    sx1276_fsk_set_rf_power(fsk_settings.Power);
    // }
    // #elif( MODULE_SX1276RF1JAS == 1 )
    //     if( FskSettings.RFFrequency > 860000000 )
    //     {
    //         SX1276FskSetPAOutput( RF_PACONFIG_PASELECT_PABOOST );
    //         SX1276FskSetPa20dBm( true );
    //         FskSettings.Power = 20;
    //         SX1276FskSetRFPower( FskSettings.Power );
    //     }
    //     else
    //     {
    //         SX1276FskSetPAOutput( RF_PACONFIG_PASELECT_RFO );
    //         SX1276FskSetPa20dBm( false );
    //         FskSettings.Power = 14;
    //         SX1276FskSetRFPower( FskSettings.Power );
    //     }
    // #endif
    //sx1276_fsk_set_opmode(RF_OPMODE_SLEEP);
    sx1276_fsk_set_opmode(RF_OPMODE_STANDBY);

    sx1276_read_buffer(REG_OPMODE, (uint8_t *)sx1276 + 1, 0x70 - 1);
    //     // Calibrate the HF
    //     SX1276FskRxCalibrate( );
    //sx1276_fsk_set_opmode(RF_OPMODE_TRANSMITTER);
    //sx1276_fsk_set_opmode(RF_OPMODE_SYNTHESIZER_TX);
    sx1276_fsk_rx_calibrate();
}

void SX1276FskSetDefaults(void)
{
    // REMARK: See SX1276 datasheet for modified default values.

    sx1276_read(REG_VERSION, &sx1276->RegVersion.Version);
}

void sx1276_fsk_set_opmode(uint8_t opMode)
{
    static uint8_t opModePrev = OPMODE_STANDBY;

    opModePrev = sx1276->RegOpMode.bits.Mode;

    if (opMode != opModePrev)
    {
        sx1276->RegOpMode.bits.Mode = opMode;
        sx1276_write(REG_OPMODE, sx1276->RegOpMode.reg);
    }
}

uint8_t sx1276_fsk_get_opmode(void)
{
    sx1276_read(REG_OPMODE, (uint8_t *)&sx1276->RegOpMode);

    return sx1276->RegOpMode.bits.Mode;
}

int32_t sx1276_fsk_read_fei(void)
{
    sx1276_read_buffer(REG_FEIMSB, (uint8_t *)&sx1276->RegFei, 2); // Reads the FEI value

    return (int32_t)(double)(((uint16_t)sx1276->RegFei.RegFeiMsb << 8) | (uint16_t)sx1276->RegFei.RegFeiLsb) * (double)FREQ_STEP;
}

int32_t sx1276_fsk_read_afc(void)
{
    sx1276_read_buffer(REG_AFCMSB, (uint8_t *)&sx1276->RegAfc, 2); // Reads the AFC value
    return (int32_t)(double)(((uint16_t)sx1276->RegAfc.RegAfcMsb << 8) | (uint16_t)sx1276->RegAfc.RegAfcLsb) * (double)FREQ_STEP;
}

uint8_t sx1276_fsk_read_rx_gain(void)
{
    sx1276_read(REG_LNA, (uint8_t *)&sx1276->RegLna);
    return sx1276->RegLna.bits.LnaGain;
}

double sx1276_fsk_read_rssi(void)
{
    sx1276_read(REG_RSSIVALUE, (uint8_t *)&sx1276->RegRssiValue); // Reads the RSSI value

    return -(double)((double)sx1276->RegRssiValue.reg / 2.0);
}
/*
uint8_t SX1276FskGetPacketRxGain( void )
{
    return RxGain;
}

double SX1276FskGetPacketRssi( void )
{
    return RxPacketRssiValue;
}

uint32_t SX1276FskGetPacketAfc( void )
{
    return RxPacketAfcValue;
}

void SX1276FskStartRx( void )
{
    SX1276FskSetRFState( RF_STATE_RX_INIT );
}
*/
void sx1276_fsk_get_rx_packet(void *buffer, uint16_t *size)
{
    *size = rx_packet_size;
    rx_packet_size = 0;
    memcpy((void *)buffer, (void *)RFBuffer, (size_t)*size);
}

void sx1276_fsk_set_tx_packet(const void *buffer, uint16_t size)
{
    tx_packet_size = size;
    memcpy((void *)RFBuffer, buffer, (size_t)tx_packet_size);

    RFState = RF_STATE_TX_INIT;
}

// Remark: SX1276 must be fully initialized before calling this function
uint16_t sx1276_fsk_get_packet_payload_size(void)
{
    uint16_t syncSize;
    uint16_t variableSize;
    uint16_t addressSize;
    uint16_t payloadSize;
    uint16_t crcSize;

    syncSize = sx1276->RegSyncConfig.bits.SyncSize + 1;
    variableSize = sx1276->RegPacketConfig1.bits.PacketFormat;
    addressSize = (sx1276->RegPacketConfig1.bits.AddressFiltering != 0x00) ? 1 : 0;
    payloadSize = sx1276->RegPayloadLength.reg;
    crcSize = (sx1276->RegPacketConfig1.bits.CrcOn == true) ? 2 : 0;

    return syncSize + variableSize + addressSize + payloadSize + crcSize;
}

// Remark: SX1276 must be fully initialized before calling this function
uint16_t sx1276_fsk_get_packet_header_size(void)
{
    uint16_t preambleSize;
    uint16_t syncSize;

    preambleSize = ((uint16_t)sx1276->RegPreamble.RegPreambleMsb << 8) | (uint16_t)sx1276->RegPreamble.RegPreambleLsb;
    syncSize = sx1276->RegSyncConfig.bits.SyncSize + 1;

    return preambleSize + syncSize;
}
/*
uint8_t SX1276FskGetRFState( void )
{
    return RFState;
}

void SX1276FskSetRFState( uint8_t state )
{
    RFState = state;
}
*/
void sx1267_fsk_get_irg_flags()
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    sx1276_read_buffer(REG_IRQFLAGS1, (uint8_t *)&sx1276->RegIrqFlags1, 2);
    if (sx1276->RegIrqFlags1.bits.TxReady)
        ESP_LOGD(TAG, "TxReady");
    if (sx1276->RegIrqFlags2.bits.FifoEmpty)
        ESP_LOGD(TAG, "FifoEmpty");
    if (sx1276->RegIrqFlags2.bits.FifoLevel)
        ESP_LOGD(TAG, "FifoLevel");
    if (sx1276->RegIrqFlags2.bits.PacketSent)
        ESP_LOGD(TAG, "PacketSent");
    if (sx1276->RegIrqFlags2.bits.PayloadReady)
        ESP_LOGD(TAG, "PayloadReady");

    uint8_t opmode = sx1276_fsk_get_opmode();

    if (opmode == OPMODE_SLEEP)
        ESP_LOGD(TAG, "OPMODE_SLEEP");
    if (opmode == OPMODE_STANDBY)
        ESP_LOGD(TAG, "OPMODE_STANDBY");
    if (opmode == OPMODE_TRANSMITTER)
        ESP_LOGD(TAG, "OPMODE_TRANSMITTER");
    if (opmode == OPMODE_RECEIVER)
        ESP_LOGD(TAG, "OPMODE_RECEIVER");
    if (opmode == OPMODE_SYNTHESIZER_TX)
        ESP_LOGD(TAG, "OPMODE_SYNTHESIZER_TX");
    if (opmode == OPMODE_SYNTHESIZER_RX)
        ESP_LOGD(TAG, "OPMODE_SYNTHESIZER_RX");
    //sx1276_read(REG_IRQFLAGS2, (uint8_t *)&sx1276->RegIrqFlags2);
}

uint8_t sx1276_fsk_process(void)
{
    sx1267_fsk_get_irg_flags();

    rf_process_return_codes_t result = RF_BUSY;

    switch (RFState)
    {
    case RF_STATE_IDLE:
        ESP_LOGD(TAG, "RF_STATE_IDLE");
        break;
        // Tx management
    case RF_STATE_TX_INIT:
        ESP_LOGD(TAG, "RF_STATE_TX_INIT");
        sx1276->RegFifoThresh.bits.TxStartCondition = TXSTARTCONDITION_FIFONOTEMPTY; //TXSTARTCONDITION_FIFOTHRESH; //
        sx1276->RegFifoThresh.bits.FifoThreshold = 0x18;                             // 0x18; // 24 bytes of data
        sx1276_write(REG_FIFOTHRESH, sx1276->RegFifoThresh.reg);
        sx1276_fsk_set_opmode(RF_OPMODE_TRANSMITTER);
        RFState = RF_STATE_TX_READY_WAIT;
        tx_bytes_sent = 0;
        break;
    case RF_STATE_TX_READY_WAIT:
        ESP_LOGD(TAG, "RF_STATE_TX_READY_WAIT");
        if (sx1276->RegIrqFlags1.bits.TxReady) // TxReady
        {
            if (sx1276->RegPacketConfig1.bits.PacketFormat == VARIABLE_LENGTH)
            {
                sx1276_write_fifo((uint8_t *)&tx_packet_size, 1);
            }
            else
            {
                sx1276_write(REG_PAYLOADLENGTH, tx_packet_size);
            }

            if ((tx_packet_size > 0) && (tx_packet_size <= 64))
            {
                data_chunk_size = tx_packet_size;
            }
            else
            {
                data_chunk_size = 32;
            }

            sx1276_write_fifo(RFBuffer, data_chunk_size);
            tx_bytes_sent += data_chunk_size;
            //TxTimeoutTimer = GET_TICK_COUNT();
            RFState = RF_STATE_TX_RUNNING;
        }
        break;
    case RF_STATE_TX_RUNNING:
        ESP_LOGD(TAG, "RF_STATE_TX_RUNNING");

        if (sx1276->RegIrqFlags2.bits.FifoLevel) // FifoLevel below threshold
        {
            if ((tx_packet_size - tx_bytes_sent) > data_chunk_size)
            {
                sx1276_write_fifo((RFBuffer + tx_bytes_sent), data_chunk_size);
                tx_bytes_sent += data_chunk_size;
            }
            else
            {
                // we write the last chunk of data
                sx1276_write_fifo(RFBuffer + tx_bytes_sent, tx_packet_size - tx_bytes_sent);
                tx_bytes_sent += tx_packet_size - tx_bytes_sent;
            }
        }

        if (sx1276->RegIrqFlags2.bits.PacketSent) // PacketSent
        {
            //TxTimeoutTimer = GET_TICK_COUNT();
            sx1276_fsk_set_opmode(RF_OPMODE_STANDBY);
            RFState = RF_STATE_TX_DONE;
        }

        // Packet timeout
        // if ((GET_TICK_COUNT() - TxTimeoutTimer) > TICK_RATE_MS(1000))
        // {
        //     RFState = RF_STATE_TX_TIMEOUT;
        // }
        break;
    case RF_STATE_TX_DONE:
        ESP_LOGD(TAG, "RF_STATE_TX_DONE");
        RFState = RF_STATE_IDLE;
        result = RF_TX_DONE;
        vTaskDelay(5000 / portTICK_PERIOD_MS); //wait for 500 ms
        break;
    default:
        break;
    }
    return result;
}
