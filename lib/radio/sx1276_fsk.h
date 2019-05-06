#ifndef __SX1276_FSK_H__
#define __SX1276_FSK_H__

#include <stdbool.h>
#include <stdint.h>

/*!
 * SX1276 FSK General parameters definition
 */

typedef struct fsk_settings_s
{
    uint32_t RFFrequency;
    uint32_t Bitrate;
    uint32_t Fdev;
    int8_t Power;
    uint32_t RxBw;
    uint32_t RxBwAfc;
    bool CrcOn;
    bool AfcOn;
    uint8_t PayloadLength;
} fsk_settings_t;

typedef struct
{
    bool BitSyncOn;
    uint8_t OokThreshType;
    uint8_t OokPeakTheshStep;
} ook_ext_settings_t;

/*!
 * RF packet definition
 */
#define RF_BUFFER_SIZE_MAX 256
#define RF_BUFFER_SIZE 256

/*!
 * RF state machine
 */
// FSK
typedef enum
{
    RF_STATE_IDLE,
    RF_STATE_RX_INIT,
    RF_STATE_RX_SYNC,
    RF_STATE_RX_RUNNING,
    RF_STATE_RX_DONE,
    RF_STATE_RX_TIMEOUT,
    RF_STATE_RX_LEN_ERROR,
    RF_STATE_TX_INIT,
    RF_STATE_TX_READY_WAIT,
    RF_STATE_TX_RUNNING,
    RF_STATE_TX_DONE,
    RF_STATE_TX_TIMEOUT,
} rf_states_t;

/*!
 * SX1276 definitions
 */
#define XTAL_FREQ 32000000
#define FREQ_STEP 61.03515625

/*!
 * \brief Initializes the SX1276
 */
void sx1276_fsk_init(void);

/*!
 * \brief Sets the SX1276 to datasheet default values
 */
void sx1276_fsk_setDefaults(void);

/*!
 * \brief Resets the SX1276
 */
void sx1276_fsk_reset(void);

/*!
 * \brief Enables/Disables the LoRa modem
 *
 * \param [IN]: enable [true, false]
 */
void sx1276_fsk_set_lora_on(bool enable);

/*!
 * \brief Sets the SX1276 operating mode
 *
 * \param [IN] opMode New operating mode
 */
void sx1276_fsk_set_opmode(uint8_t opMode);

/*!
 * \brief Gets the SX1276 operating mode
 *
 * \retval opMode Current operating mode
 */
uint8_t sx1276_fsk_get_OpMode(void);

/*!
 * \brief Trigs and reads the FEI
 *
 * \retval feiValue Frequency error value.
 */
int32_t sx1276_fsk_read_fei(void);

/*!
 * \brief Reads the current AFC value
 *
 * \retval afcValue Frequency offset value.
 */
int32_t sx1276_fsk_read_afc(void);

/*!
 * \brief Reads the current Rx gain setting
 *
 * \retval rxGain Current gain setting
 */
uint8_t sx1276_fsk_read_rx_gain(void);

/*!
 * \brief Trigs and reads the current RSSI value
 *
 * \retval rssiValue Current RSSI value in [dBm]
 */
double sx1276_fsk_read_rssi(void);

/*!
 * \brief Gets the Rx gain value measured while receiving the packet
 *
 * \retval rxGainValue Current Rx gain value
 */
uint8_t sx1276_fsk_getPacketRxGain(void);

/*!
 * \brief Gets the RSSI value measured while receiving the packet
 *
 * \retval rssiValue Current RSSI value in [dBm]
 */
double sx1276_fsk_getPacketRssi(void);

/*!
 * \brief Gets the AFC value measured while receiving the packet
 *
 * \retval afcValue Current AFC value in [Hz]
 */
uint32_t sx1276_fsk_getPacketAfc(void);

/*!
 * \brief Sets the radio in Rx mode. Waiting for a packet
 */
void SX1276FskStartRx(void);

/*!
 * \brief Gets a copy of the current received buffer
 *
 * \param [IN]: buffer     Buffer pointer
 * \param [IN]: size       Buffer size
 */
void sx1276_fsk_getRxPacket(void *buffer, uint16_t *size);

/*!
 * \brief Sets a copy of the buffer to be transmitted and starts the
 *        transmission
 *
 * \param [IN]: buffer     Buffer pointer
 * \param [IN]: size       Buffer size
 */
void sx1276_fsk_set_tx_packet(const void *buffer, uint16_t size);

/*!
 * \brief Gets the current RFState
 *
 * \retval rfState Current RF state [RF_IDLE, RF_BUSY, 
 *                                   RF_RX_DONE, RF_RX_TIMEOUT,
 *                                   RF_TX_DONE, RF_TX_TIMEOUT]
 */
uint8_t sx1276_fsk_get_rf_state(void);

/*!
 * \brief Sets the new state of the RF state machine
 *
 * \param [IN]: state New RF state machine state
 */
void sx1276_fsk_set_rf_state(uint8_t state);

/*!
 * \brief Process the FSK modem Rx and Tx state machines depending on the
 *       SX1276 operating mode.
 *
 * \retval rfState Current RF state [RF_IDLE, RF_BUSY, 
 *                                   RF_RX_DONE, RF_RX_TIMEOUT,
 *                                   RF_TX_DONE, RF_TX_TIMEOUT]
 */
uint8_t sx1276_fsk_process(void);

#endif