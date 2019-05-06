#ifndef __SX1276_FSK_MISC_H__
#define __SX1276_FSK_MISC_H__

#include <stdint.h>

/*!
 * \brief Writes the new RF frequency value
 *
 * \param [IN] freq New RF frequency value in [Hz]
 */
void sx1276_fsk_set_rf_frequency(uint32_t freq);

/*!
 * \brief Reads the current RF frequency value
 *
 * \retval freq Current RF frequency value in [Hz]
 */
uint32_t sx1276_fsk_get_rf_frequency(void);

/*!
 * \brief Calibrate RSSI and I/Q mismatch for HF 
 *
 * \retval none
 */
void sx1276_fsk_rx_calibrate(void);

/*!
 * \brief Writes the new bitrate value
 *
 * \param [IN] bitrate New bitrate value in [bps]
 */
void sx1276_fsk_set_bitrate(uint32_t bitrate);

/*!
 * \brief Reads the current bitrate value
 *
 * \retval bitrate Current bitrate value in [bps]
 */
uint32_t sx1276_fsk_get_bitrate(void);

/*!
 * \brief Writes the new frequency deviation value
 *
 * \param [IN] fdev New frequency deviation value in [Hz]
 */
void sx1276_fsk_set_fdev(uint32_t fdev);

/*!
 * \brief Reads the current frequency deviation value
 *
 * \retval fdev Current frequency deviation value in [Hz]
 */
uint32_t sx1276_fsk_get_fdev(void);

/*!
 * \brief Writes data processing mode
 *
 * \param [IN] datamode processing mode
 */
void sx1276_fsk_set_datamode(uint8_t datamode);

/*!
 * \brief Writes the new RF output power value
 *
 * \param [IN] power New output power value in [dBm]
 */
void sx1276_fsk_set_rf_power(int8_t power);

// /*!
//  * \brief Reads the current RF output power value
//  *
//  * \retval power Current output power value in [dBm]
//  */
// int8_t SX1276FskGetRFPower(void);

/*!
 * \brief Writes the DC offset canceller and Rx bandwidth values
 *
 * \remark For SX1276 there is no DCC setting. dccValue should be 0
 *         ie: SX1276SetDccBw( &SX1276.RegRxBw, 0, 62500 );
 *
 * \param [IN] reg Register pointer to either SX1231.RegRxBw or SX1231.RegAfcBw
 * \param [IN] dccValue New DC offset canceller value in [Hz] ( SX1231 only )
 * \param [IN] rxBwValue New Rx bandwidth value in [Hz]
 */
void sx1276_fsk_set_dcc_bw(uint8_t *reg, uint32_t dccValue, uint32_t rxBwValue);

// /*!
//  * \brief Reads the current bandwidth setting
//  *
//  * \param [IN] reg Register pointer to either SX1231.RegRxBw or SX1231.RegAfcBw
//  *
//  * \retval bandwidth Bandwidth value
//  */
// uint32_t SX1276FskGetBw(uint8_t *reg);

// /*!
//  * \brief Enables/Disables CRC
//  *
//  * \param [IN] enable CRC enable/disable
//  */
// void SX1276FskSetPacketCrcOn(bool enable);

// /*!
//  * \brief Reads the current CRC Enable/Disbale value
//  *
//  * \retval enable Current CRC Enable/Disbale value
//  */
// bool SX1276FskGetPacketCrcOn(void);

// /*!
//  * \brief Enables/Disables AFC
//  *
//  * \param [IN] enable AFC enable/disable
//  */
// void SX1276FskSetAfcOn(bool enable);

// /*!
//  * \brief Reads the current AFC Enable/Disbale value
//  *
//  * \retval enable Current AFC Enable/Disbale value
//  */
// bool SX1276FskGetAfcOn(void);

// /*!
//  * \brief Writes the new payload length value
//  *
//  * \param [IN] value New payload length value
//  */
// void SX1276FskSetPayloadLength(uint8_t value);

// /*!
//  * \brief Reads the current payload length value
//  *
//  * \retval value Current payload length value
//  */
// uint8_t SX1276FskGetPayloadLength(void);

/*!
 * \brief Enables/Disables the 20 dBm PA
 *
 * \param [IN] enable [true, false]
 */
void sx1276_fsk_set_pa_20dBm(bool enale);

// /*!
//  * \brief Gets the current 20 dBm PA status
//  *
//  * \retval enable [true, false]
//  */
// bool SX1276FskGetPa20dBm(void);

/*!
 * \brief Set the RF Output pin 
 *
 * \param [IN] RF_PACONFIG_PASELECT_PABOOST or RF_PACONFIG_PASELECT_RFO
 */
void sx1276_fsk_set_pa_output(uint8_t outputPin);

// /*!
//  * \brief Gets the used RF Ouptu pin
//  *
//  * \retval RF_PACONFIG_PASELECT_PABOOST or RF_PACONFIG_PASELECT_RFO
//  */
// uint8_t SX1276FskGetPAOutput(void);

// /*!
//  * \brief Writes the new PA rise/fall time of ramp up/down value
//  *
//  * \param [IN] value New PaRamp value
//  */
// void SX1276FskSetPaRamp(uint8_t value);

// /*!
//  * \brief Reads the current PA rise/fall time of ramp up/down value
//  *
//  * \retval value Current PaRamp value
//  */
// uint8_t SX1276FskGetPaRamp(void);

/*!
 * \brief Applies an offset to the RSSI. Compensates board components
 *
 * \param [IN] offset Offset to be applied (+/-)
 */
void sx1276_fsk_set_rssi_offset(int8_t offset);

/*!
 * \brief Gets the current RSSI offset.
 *
 * \retval offset Current offset (+/-)
 */
int8_t sx1276_fsk_get_rssi_offset(void);

// /*!
//  * \brief Writes the new value for the preamble size
//  *
//  * \param [IN] size New value of pramble size
//  */
// void SX1276FskSetPreambleSize(uint16_t size);

// /*!
//  * Reads the raw temperature
//  * \retval temperature New raw temperature reading in 2's complement format
//  */
// int8_t SX1276FskGetRawTemp(void);

// /*!
//  * Computes the temperature compensation factor
//  * \param [IN] actualTemp Actual temperature measured by an external device
//  * \retval compensationFactor Computed compensation factor
//  */
// int8_t SX1276FskCalibreateTemp(int8_t actualTemp);

// /*!
//  * Gets the actual compensated temperature
//  * \param [IN] compensationFactor Return value of the calibration function
//  * \retval New compensated temperature value
//  */
// int8_t SX1276FskGetTemp(int8_t compensationFactor);

#endif //__SX1276_FSK_MISC_H__