#include <stdint.h>
#include <math.h>

#include "driver\spi_common.h"
#include "esp_log.h"

#include "sx1276_fsk_defs.h"
#include "sx1276_fsk_regs.h"
#include "sx1276_fsk_misc.h"
#include "sx1276_fsk.h"
#include "sx1276_hal.h"
#include "sx1276.h"

extern fsk_settings_t fsk_settings;

static const char *TAG = "sx1276_fsk_misc";

void sx1276_fsk_set_rf_frequency(uint32_t freq)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    fsk_settings.RFFrequency = freq;

    freq = (uint32_t)((double)freq / (double)FREQ_STEP);

    sx1276->RegFrf.RegFrfMsb = (uint8_t)((freq >> 16) & 0xFF);
    sx1276->RegFrf.RegFrfMid = (uint8_t)((freq >> 8) & 0xFF);
    sx1276->RegFrf.RegFrfLsb = (uint8_t)(freq & 0xFF);
    sx1276_write_buffer(REG_FRFMSB, (uint8_t *)&sx1276->RegFrf, 3);
}

uint32_t sx1276_fsk_get_rf_frequency(void)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    sx1276_read_buffer(REG_FRFMSB, (uint8_t *)&sx1276->RegFrf, 3);

    fsk_settings.RFFrequency = ((uint32_t)sx1276->RegFrf.RegFrfMsb << 16) | ((uint32_t)sx1276->RegFrf.RegFrfMid << 8) | ((uint32_t)sx1276->RegFrf.RegFrfLsb);
    fsk_settings.RFFrequency = (uint32_t)((double)fsk_settings.RFFrequency * (double)FREQ_STEP);

    return fsk_settings.RFFrequency;
}

void sx1276_fsk_rx_calibrate(void)
{
    // the function RadioRxCalibrate is called just after the reset so all register are at their default values
    uint8_t regPaConfigInitVal;
    uint32_t initialFreq;

    // save register values;
    sx1276_read(REG_PACONFIG, &regPaConfigInitVal);
    initialFreq = sx1276_fsk_get_rf_frequency();

    // Cut the PA just in case
    sx1276->RegPaConfig.reg = 0x00; // RFO output, power = -1 dBm
    sx1276_write(REG_PACONFIG, sx1276->RegPaConfig.reg);

    // Set Frequency in HF band
    sx1276_fsk_set_rf_frequency(860000000);

    // Rx chain re-calibration workaround
    sx1276_read(REG_IMAGECAL, (uint8_t *)&sx1276->RegImageCal);
    sx1276->RegImageCal.bits.ImageCalStart = true;
    sx1276_write(REG_IMAGECAL, sx1276->RegImageCal.reg);

    sx1276_read(REG_IMAGECAL, (uint8_t *)&sx1276->RegImageCal);
    // rx_cal_run goes low when calibration in finished
    while (sx1276->RegImageCal.bits.ImageCalRunning == true)
    {
        sx1276_read(REG_IMAGECAL, (uint8_t *)&sx1276->RegImageCal);
    }

    // reload saved values into the registers
    sx1276->RegPaConfig.reg = regPaConfigInitVal;
    sx1276_write(REG_PACONFIG, sx1276->RegPaConfig.reg);

    sx1276_fsk_set_rf_frequency(initialFreq);
}

void sx1276_fsk_set_bitrate(uint32_t bitrate)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    fsk_settings.Bitrate = bitrate;

    bitrate = (uint16_t)((double)XTAL_FREQ / (double)bitrate);
    sx1276->RegBitrate.RegBitrateMsb = (uint8_t)((bitrate >> 8) & 0xFF);
    sx1276->RegBitrate.RegBitrateLsb = (uint8_t)(bitrate & 0xFF);
    sx1276_write_buffer(REG_BITRATEMSB, (uint8_t *)&sx1276->RegBitrate, 2);
}

uint32_t sx1276_fsk_get_bitrate(void)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    sx1276_read_buffer(REG_BITRATEMSB, (uint8_t *)&sx1276->RegBitrate, 2);
    fsk_settings.Bitrate = (((uint32_t)sx1276->RegBitrate.RegBitrateMsb << 8) | ((uint32_t)sx1276->RegBitrate.RegBitrateLsb));
    fsk_settings.Bitrate = (uint16_t)((double)XTAL_FREQ / (double)fsk_settings.Bitrate);

    return fsk_settings.Bitrate;
}

void sx1276_fsk_set_fdev(uint32_t fdev)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    fsk_settings.Fdev = fdev;

    sx1276_read_buffer(REG_FDEVMSB, (uint8_t *)&sx1276->RegFdev, 2);
    fdev = (uint16_t)((double)fdev / (double)FREQ_STEP);
    sx1276->RegFdev.RegFdevMsb.bits.Fdev_13_8 = (uint8_t)((fdev >> 8) & 0xFF);
    sx1276->RegFdev.RegFdevLsb.Fdev_7_0 = (uint8_t)(fdev & 0xFF);
    sx1276_write_buffer(REG_FDEVMSB, (uint8_t *)&sx1276->RegFdev, 2);
}

uint32_t sx1276_fsk_get_fdev(void)
{
    sx1276_read_buffer(REG_FDEVMSB, (uint8_t *)&sx1276->RegFdev, 2);
    fsk_settings.Fdev = (((uint32_t)(sx1276->RegFdev.RegFdevMsb.bits.Fdev_13_8 << 8)) | ((uint32_t)sx1276->RegFdev.RegFdevLsb.Fdev_7_0));
    fsk_settings.Fdev = (uint16_t)((double)fsk_settings.Fdev * (double)FREQ_STEP);

    return fsk_settings.Fdev;
}

void sx1276_fsk_set_datamode(uint8_t datamode)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);
    sx1276->RegPacketConfig2.bits.DataMode = CONTINUOUS_MODE;
    sx1276_write(REG_FDEVMSB, sx1276->RegPacketConfig2.reg);
}

void sx1276_fsk_set_rf_power(int8_t power)
{
    sx1276_read(REG_PACONFIG, (uint8_t *)&sx1276->RegPaConfig);
    sx1276_read(REG_PADAC, (uint8_t *)&sx1276->RegPaDac);

    if (sx1276->RegPaConfig.bits.PaSelect == MAX_POWER_20_DBM)
    {
        if (sx1276->RegPaDac.bits.PaDac == 0x07)
        {
            if (power < 5)
            {
                power = 5;
            }
            if (power > 20)
            {
                power = 20;
            }
            sx1276->RegPaConfig.bits.MaxPower = 0b00000111;
            sx1276->RegPaConfig.bits.OutputPower = power - 5;
        }
        else
        {
            if (power < 2)
            {
                power = 2;
            }
            if (power > 17)
            {
                power = 17;
            }
            sx1276->RegPaConfig.bits.MaxPower = 0b00000111;
            sx1276->RegPaConfig.bits.OutputPower = power - 2;
        }
    }
    else
    {
        if (power < -1)
        {
            power = -1;
        }
        if (power > 14)
        {
            power = 14;
        }
        sx1276->RegPaConfig.bits.MaxPower = 0b00000111;
        sx1276->RegPaConfig.bits.OutputPower = power + 1;
    }
    sx1276_write(REG_PACONFIG, sx1276->RegPaConfig.reg);
    fsk_settings.Power = power;
}

int8_t sx1276_fsk_get_rf_power(void)
{
    sx1276_read(REG_PACONFIG, (uint8_t *)&sx1276->RegPaConfig);
    sx1276_read(REG_PADAC, (uint8_t *)&sx1276->RegPaDac);

    if (sx1276->RegPaConfig.bits.PaSelect == MAX_POWER_20_DBM)
    {
        if (sx1276->RegPaDac.bits.PaDac == 0b0000111)
        {
            fsk_settings.Power = 5 + sx1276->RegPaConfig.bits.OutputPower;
        }
        else
        {
            fsk_settings.Power = 2 + sx1276->RegPaConfig.bits.OutputPower;
        }
    }
    else
    {
        fsk_settings.Power = -1 + sx1276->RegPaConfig.bits.OutputPower;
    }
    return fsk_settings.Power;
}

// /*!
//  * \brief Computes the Rx bandwidth with the mantisse and exponent
//  *
//  * \param [IN] mantisse Mantisse of the bandwidth value
//  * \param [IN] exponent Exponent of the bandwidth value
//  * \retval bandwidth Computed bandwidth
//  */
// static uint32_t SX1276FskComputeRxBw(uint8_t mantisse, uint8_t exponent)
// {
//     // rxBw
//     if ((SX1276->RegOpMode & RF_OPMODE_MODULATIONTYPE_FSK) == RF_OPMODE_MODULATIONTYPE_FSK)
//     {
//         return (uint32_t)((double)XTAL_FREQ / (mantisse * (double)pow(2, exponent + 2)));
//     }
//     else
//     {
//         return (uint32_t)((double)XTAL_FREQ / (mantisse * (double)pow(2, exponent + 3)));
//     }
// }

/*!
 * \brief Computes the mantisse and exponent from the bandwitdh value
 *
 * \param [IN] rxBwValue Bandwidth value
 * \param [OUT] mantisse Mantisse of the bandwidth value
 * \param [OUT] exponent Exponent of the bandwidth value
 */
static void sx1276_fsk_compute_rx_bw_mant_exp(uint32_t rxBwValue, uint8_t *mantisse, uint8_t *exponent)
{
    uint8_t tmpExp = 0;
    uint8_t tmpMant = 0;

    double tmpRxBw = 0;
    double rxBwMin = 10e6;

    for (tmpExp = 0; tmpExp < 8; tmpExp++)
    {
        for (tmpMant = 16; tmpMant <= 24; tmpMant += 4)
        {
            if (sx1276->RegOpMode.bits.ModulationType == MODULATIONTYPE_FSK)
            {
                tmpRxBw = (double)XTAL_FREQ / (tmpMant * (double)pow(2, tmpExp + 2));
            }
            else
            {
                tmpRxBw = (double)XTAL_FREQ / (tmpMant * (double)pow(2, tmpExp + 3));
            }
            if (fabs(tmpRxBw - rxBwValue) < rxBwMin)
            {
                rxBwMin = fabs(tmpRxBw - rxBwValue);
                *mantisse = tmpMant;
                *exponent = tmpExp;
            }
        }
    }
}

void sx1276_fsk_set_dcc_bw(uint8_t *reg, uint32_t dccValue, uint32_t rxBwValue)
{
    uint8_t mantisse = 0;
    uint8_t exponent = 0;

    if (reg == (uint8_t *)&sx1276->RegRxBw)
    {
        *reg = (uint8_t)dccValue & 0x60;
    }
    else
    {
        *reg = 0;
    }

    sx1276_fsk_compute_rx_bw_mant_exp(rxBwValue, &mantisse, &exponent);

    switch (mantisse)
    {
    case 16:
        *reg |= (uint8_t)(0x00 | (exponent & 0x07));
        break;
    case 20:
        *reg |= (uint8_t)(0x08 | (exponent & 0x07));
        break;
    case 24:
        *reg |= (uint8_t)(0x10 | (exponent & 0x07));
        break;
    default:
        // Something went terribely wrong
        break;
    }

    if (reg == (uint8_t *)&sx1276->RegRxBw)
    {
        sx1276_write(REG_RXBW, *reg);
        fsk_settings.RxBw = rxBwValue;
    }
    else
    {
        sx1276_write(REG_AFCBW, *reg);
        fsk_settings.RxBwAfc = rxBwValue;
    }
}

// uint32_t SX1276FskGetBw(uint8_t *reg)
// {
//     uint32_t rxBwValue = 0;
//     uint8_t mantisse = 0;
//     switch ((*reg & 0x18) >> 3)
//     {
//     case 0:
//         mantisse = 16;
//         break;
//     case 1:
//         mantisse = 20;
//         break;
//     case 2:
//         mantisse = 24;
//         break;
//     default:
//         break;
//     }
//     rxBwValue = SX1276FskComputeRxBw(mantisse, (uint8_t)*reg & 0x07);
//     if (reg == &SX1276->RegRxBw)
//     {
//         return fsk_settings.RxBw = rxBwValue;
//     }
//     else
//     {
//         return fsk_settings.RxBwAfc = rxBwValue;
//     }
// }

void sx1276_fsk_set_packet_crc_on(bool enable)
{
    sx1276_read(REG_PACKETCONFIG1, (uint8_t *)&sx1276->RegPacketConfig1);
    sx1276->RegPacketConfig1.bits.CrcOn = enable;
    sx1276_write(REG_PACKETCONFIG1, sx1276->RegPacketConfig1.reg);
    fsk_settings.CrcOn = enable;
}

bool sx1276_fsk_get_packet_crc_on(void)
{
    sx1276_read(REG_PACKETCONFIG1, (uint8_t *)&sx1276->RegPacketConfig1);
    fsk_settings.CrcOn = sx1276->RegPacketConfig1.bits.CrcOn;
    return fsk_settings.CrcOn;
}

void sx1276_fsk_set_afc_on(bool enable)
{
    sx1276_read(REG_RXCONFIG, (uint8_t *)&sx1276->RegRxConfig);
    sx1276->RegRxConfig.bits.AfcAutoOn = enable;
    sx1276_write(REG_RXCONFIG, sx1276->RegRxConfig.reg);
    fsk_settings.AfcOn = enable;
}

bool sx1276_fsk_get_afc_on(void)
{
    sx1276_read(REG_RXCONFIG, (uint8_t *)&sx1276->RegRxConfig);
    fsk_settings.AfcOn = sx1276->RegRxConfig.bits.AfcAutoOn;
    return fsk_settings.AfcOn;
}

void sx1276_fsk_set_payload_length(uint8_t value)
{
    sx1276->RegPayloadLength.reg = value;
    sx1276_write(REG_PAYLOADLENGTH, sx1276->RegPayloadLength.reg);
    fsk_settings.PayloadLength = value;
}

uint8_t sx1276_fsk_get_payload_length(void)
{
    sx1276_read(REG_PAYLOADLENGTH, (uint8_t *)&sx1276->RegPayloadLength);
    fsk_settings.PayloadLength = sx1276->RegPayloadLength.reg;
    return fsk_settings.PayloadLength;
}

void sx1276_fsk_set_pa_20dBm(bool enale)
{
    sx1276_read(REG_PADAC, (uint8_t *)&sx1276->RegPaDac);
    sx1276_read(REG_PACONFIG, (uint8_t *)&sx1276->RegPaConfig);

    if (sx1276->RegPaConfig.bits.PaSelect == MAX_POWER_20_DBM)
    {
        if (enale == true)
        {
            sx1276->RegPaDac.bits.PaDac = 0b00000111;
        }
    }
    else
    {
        sx1276->RegPaDac.bits.PaDac = 0b00000100;
    }
    sx1276_write(REG_PADAC, sx1276->RegPaDac.reg);
}

bool sx1276_fsk_get_pa_20dBm(void)
{
    sx1276_read(REG_PADAC, (uint8_t *)&sx1276->RegPaDac);

    return (sx1276->RegPaDac.bits.PaDac == 0b00000111) ? true : false;
}

void sx1276_fsk_set_pa_output(uint8_t outputPin)
{
    sx1276_read(REG_PACONFIG, (uint8_t *)&sx1276->RegPaConfig);
    sx1276->RegPaConfig.bits.PaSelect = outputPin;
    sx1276_write(REG_PACONFIG, sx1276->RegPaConfig.reg);
}

uint8_t sx1276_fsk_get_pa_output(void)
{
    sx1276_read(REG_PACONFIG, (uint8_t *)&sx1276->RegPaConfig);
    return sx1276->RegPaConfig.bits.PaSelect;
}

void sx1276_fsk_set_pa_ramp(uint8_t value)
{
    sx1276_read(REG_PARAMP, (uint8_t *)&sx1276->RegPaRamp);
    sx1276->RegPaRamp.bits.PaRamp = value;
    sx1276_write(REG_PARAMP, sx1276->RegPaRamp.reg);
}

uint8_t sx1276_fsk_get_pa_ramp(void)
{
    sx1276_read(REG_PARAMP, (uint8_t *)&sx1276->RegPaRamp);
    return sx1276->RegPaRamp.bits.PaRamp;
}

void sx1276_fsk_set_rssi_offset(int8_t offset)
{
    sx1276_read(REG_RSSICONFIG, (uint8_t *)&sx1276->RegRssiConfig);
    if (offset < 0)
    {
        offset = (~offset & 0x1F);
        offset += 1;
        offset = -offset;
    }
    sx1276->RegRssiConfig.bits.RssiOffset = offset;
    sx1276_write(REG_RSSICONFIG, sx1276->RegRssiConfig.reg);
}

int8_t sx1276_fsk_get_rssi_offset(void)
{
    sx1276_read(REG_RSSICONFIG, (uint8_t *)&sx1276->RegRssiConfig);
    int8_t offset = sx1276->RegRssiConfig.bits.RssiOffset;
    if ((offset & 0x10) == 0x10)
    {
        offset = (~offset & 0x1F);
        offset += 1;
        offset = -offset;
    }
    return offset;
}

// int8_t SX1276FskGetRawTemp(void)
// {
//     int8_t temp = 0;
//     uint8_t previousOpMode;
//     uint32_t startTick;

//     // Enable Temperature reading
//     sx1276_read(REG_IMAGECAL, &SX1276->RegImageCal);
//     SX1276->RegImageCal = (SX1276->RegImageCal & RF_IMAGECAL_TEMPMONITOR_MASK) | RF_IMAGECAL_TEMPMONITOR_ON;
//     SX1276Write(REG_IMAGECAL, SX1276->RegImageCal);

//     // save current Op Mode
//     sx1276_read(REG_OPMODE, &SX1276->RegOpMode);
//     previousOpMode = SX1276->RegOpMode;

//     // put device in FSK RxSynth
//     SX1276->RegOpMode = RF_OPMODE_SYNTHESIZER_RX;
//     SX1276Write(REG_OPMODE, SX1276->RegOpMode);

//     // Wait 1ms
//     startTick = GET_TICK_COUNT();
//     while ((GET_TICK_COUNT() - startTick) < TICK_RATE_MS(1))
//         ;

//     // Disable Temperature reading
//     sx1276_read(REG_IMAGECAL, &SX1276->RegImageCal);
//     SX1276->RegImageCal = (SX1276->RegImageCal & RF_IMAGECAL_TEMPMONITOR_MASK) | RF_IMAGECAL_TEMPMONITOR_OFF;
//     SX1276Write(REG_IMAGECAL, SX1276->RegImageCal);

//     // Read temperature
//     sx1276_read(REG_TEMP, &SX1276->RegTemp);

//     temp = SX1276->RegTemp & 0x7F;

//     if ((SX1276->RegTemp & 0x80) == 0x80)
//     {
//         temp *= -1;
//     }

//     // Reload previous Op Mode
//     SX1276Write(REG_OPMODE, previousOpMode);

//     return temp;
// }

// int8_t SX1276FskCalibreateTemp(int8_t actualTemp)
// {
//     return actualTemp - SX1276FskGetRawTemp();
// }

// int8_t SX1276FskGetTemp(int8_t compensationFactor)
// {
//     return SX1276FskGetRawTemp() + compensationFactor;
// }
