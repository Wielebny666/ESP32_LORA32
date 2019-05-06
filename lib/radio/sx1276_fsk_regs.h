#ifndef __SX1276_FSK_REGS_H__
#define __SX1276_FSK_REGS_H__

#include <stdint.h>

typedef enum
{
    MODULATIONTYPE_FSK,
    MODULATIONTYPE_OOK
} modulation_type_t;

typedef enum
{
    FSK_OOK,
    LORA
} long_range_mode_t;

typedef struct
{
    uint8_t Fifo;
} RegFifo_t;

typedef union {
    struct __attribute__((__packed__, aligned(1)))
    {
        enum
        {
            OPMODE_SLEEP = 0x00,
            OPMODE_STANDBY = 0x01,
            OPMODE_SYNTHESIZER_TX = 0x02,
            OPMODE_TRANSMITTER = 0x03,
            OPMODE_SYNTHESIZER_RX = 0x04,
            OPMODE_RECEIVER = 0x05
        } Mode : 3;
        uint8_t LowFrequencyModeOn : 1;
        uint8_t reserved_4 : 1;
        modulation_type_t ModulationType : 2;
        long_range_mode_t LongRangeMode : 1;
    } bits;
    uint8_t reg;
} RegOpMode_t;

typedef union __attribute__((__packed__, aligned(1))) {
    struct
    {
        uint8_t RegBitrateMsb;
        uint8_t RegBitrateLsb;
    };
    uint16_t reg;
} RegBitrate_t;

typedef union {
    uint8_t reg;
    struct
    {
        uint8_t Fdev_13_8 : 6;
        uint8_t reserved_7_6 : 2;
    } bits;
} RegFdevMsb_t;

typedef struct
{
    uint8_t Fdev_7_0;
} RegFdevLsb_t;

typedef union __attribute__((__packed__, aligned(1))) {
    uint16_t reg;
    struct
    {
        RegFdevMsb_t RegFdevMsb;
        RegFdevLsb_t RegFdevLsb;
    };
} RegFdev_t;

// typedef struct
// {
//     uint8_t Frf_23_16;
// } RegFrfMsb_t;

// typedef struct
// {
//     uint8_t Frf_15_8;
// } RegFrfMid_t;

// typedef struct
// {
//     uint8_t Frf_7_0;
// } RegFrfLsb_t;

typedef union __attribute__((__packed__, aligned(1))) {
    struct
    {
        uint8_t RegFrfMsb : 8;
        uint8_t RegFrfMid : 8;
        uint8_t RegFrfLsb : 8;
    };
    uint32_t reg : 24;
} RegFrf_t;

/*!
* Registers for the Transmitter
*/
typedef union {
    struct __attribute__((__packed__, aligned(1)))
    {
        uint8_t OutputPower : 4;
        uint8_t MaxPower : 3;
        enum
        {
            MAX_POWER_14_DBM = 0,
            MAX_POWER_20_DBM
        } PaSelect : 1;
    } bits;
    uint8_t reg;
} RegPaConfig_t;

//OOK
#define NO_SHAPING 0
#define GAUSSIAN_FILTER_BT_1_0 1
#define GAUSSIAN_FILTER_BT_0_5 2
#define GAUSSIAN_FILTER_BT_0_3 3
//ASK
#define F_CUTOFF_BIT_RATE 1
#define F_CUTOFF_2X_BIT_RATE 2
#define RESERVED 3

typedef union {
    struct __attribute__((__packed__, aligned(1)))
    {
        enum
        {
            PARAMP_3400_US = 0x00,
            PARAMP_2000_US,
            PARAMP_1000_US,
            PARAMP_0500_US,
            PARAMP_0250_US,
            PARAMP_0125_US,
            PARAMP_0100_US,
            PARAMP_0062_US,
            PARAMP_0050_US,
            PARAMP_0040_US, // Default
            PARAMP_0031_US,
            PARAMP_0025_US,
            PARAMP_0020_US,
            PARAMP_0015_US,
            PARAMP_0012_US,
            PARAMP_0010_US
        } PaRamp : 4;
        uint8_t reserved : 1;
        uint8_t ModulationShaping : 2;
        uint8_t unused : 1;
    } bits;
    uint8_t reg;
} RegPaRamp_t;

typedef union {
    struct
    {
        uint8_t unused : 2;
        uint8_t OcpOn : 1;
        uint8_t OcpTrim : 5;
    } bits;
    uint8_t reg;
} RegOcp_t;

/*!
* Registers for the Receiver
*/
typedef union {
    struct __attribute__((__packed__, aligned(1)))
    {
        uint8_t LnaBoostHf : 2;
        uint8_t reserved : 1;
        uint8_t LnaBoostLf : 2;
        enum
        {
            LNA_GAIN_G1 = 1,
            LNA_GAIN_G2,
            LNA_GAIN_G3,
            LNA_GAIN_G4,
            LNA_GAIN_G5,
            LNA_GAIN_G6,
        } LnaGain : 3;
    } bits;
    uint8_t reg;
} RegLna_t;

typedef union {
    struct __attribute__((__packed__, aligned(1)))
    {
        enum
        {
            RXTRIGER_OFF = 0x00,
            RXTRIGER_RSSI = 0x01,
            RXTRIGER_PREAMBLEDETECT = 0x06, // Default
            RXTRIGER_RSSI_PREAMBLEDETECT = 0x07
        } RxTrigger : 3;
        uint8_t AgcAutoOn : 1;
        uint8_t AfcAutoOn : 1;
        uint8_t RestartRxWithPllLock : 1;
        uint8_t RestartRxWithoutPllLock : 1;
        uint8_t RestartRxOnCollision : 1;
    } bits;
    uint8_t reg;
} RegRxConfig_t;

typedef union {
    struct
    {
        uint8_t RssiSmoothing : 3;
        uint8_t RssiOffset : 5;
    } bits;
    uint8_t reg;
} RegRssiConfig_t;

typedef struct
{
    uint8_t reg;
} RegRssiCollision_t;

typedef struct
{
    uint8_t reg;
} RegRssiThresh_t;

typedef struct
{
    uint8_t reg;
} RegRssiValue_t;

typedef union {
    struct
    {
        uint8_t RxBwExp : 3;
        uint8_t RxBwMant : 2;
        uint8_t reserved_6_5 : 2;
        uint8_t unused_7 : 1;
    } bits;
    uint8_t reg;
} RegRxBw_t;

typedef union {
    struct
    {
        uint8_t RxBwExpAfc : 3;
        uint8_t RxBwMantAfc : 2;
        uint8_t reserved_7_5 : 3;
    } bits;
    uint8_t reg;
} RegAfcBw_t;

typedef union {
    struct
    {
        uint8_t OokPeakTheshStep : 3;
        uint8_t OokThreshType : 2;
        uint8_t BitSyncOn : 1;
        uint8_t reserved_7_6 : 2;
    } bits;
    uint8_t reg;
} RegOokPeak_t;

typedef struct
{
    uint8_t OokFixedThreshold;
} RegOokFix_t;

typedef union {
    struct
    {
        uint8_t OokAverageThreshFilt : 2;
        uint8_t OokAverageOffset : 2;
        uint8_t reserved : 1;
        uint8_t OokPeakThreshDec : 3;
    } bits;
    uint8_t reg;
} RegOokAvg_t;

typedef union {
    struct
    {
        uint8_t AfcAutoClearOn : 1;
        uint8_t AfcClear : 1;
        uint8_t unused_2 : 1;
        uint8_t reserved_3 : 1;
        uint8_t AgcStart : 1;
        uint8_t unused_7_5 : 3;
    } bits;
    uint8_t reg;
} RegAfcFei_t;

typedef union {
    struct
    {
        uint8_t RegAfcMsb;
        uint8_t RegAfcLsb;
    };
    uint16_t reg;
} RegAfc_t;

typedef union {
    struct
    {
        uint8_t RegFeiMsb;
        uint8_t RegFeiLsb;
    };
    uint16_t reg;
} RegFei_t;

typedef union {
    struct __attribute__((__packed__, aligned(1)))
    {
        uint8_t PreambleDetectorTol : 5;
        enum
        {
            DETECTORSIZE_1,
            DETECTORSIZE_2,
            DETECTORSIZE_3
        } PreambleDetectorSize : 2;
        uint8_t PreambleDetectorOn : 1;
    } bits;
    uint8_t reg;
} RegPreambleDetect_t;

typedef struct
{
    uint8_t TimeoutRxRssi;
} RegRxTimeout1_t;

typedef struct
{
    uint8_t TimeoutRxPreamble;
} RegRxTimeout2_t;

typedef struct
{
    uint8_t TimeoutSignalSync;
} RegRxTimeout3_t;

typedef struct
{
    uint8_t InterPacketRxDelay;
} RegRxDelay_t;

typedef union {
    struct
    {
        uint8_t ClkOut : 3;
        uint8_t RcCalStart : 1;
        uint8_t unused_7_4 : 4;
    } bits;
    uint8_t reg;
} RegOsc_t;

typedef union {
    struct
    {
        uint8_t RegPreambleMsb;
        uint8_t RegPreambleLsb;
    };
    uint16_t RegPreamble;
} RegPreamble_t;

typedef union {
    struct __attribute__((__packed__, aligned(1)))
    {
        enum
        {
            SYNCSIZE_1 = 0,
            SYNCSIZE_2,
            SYNCSIZE_3,
            SYNCSIZE_4,
            SYNCSIZE_5,
            SYNCSIZE_6,
            SYNCSIZE_7,
            SYNCSIZE_8
        } SyncSize : 3;
        uint8_t reserved_3 : 1;
        uint8_t SyncOn : 1;
        enum
        {
            PREAMBLE_0xAA = 0,
            PREAMBLE_0x55
        } PreamblePolarity : 1;
        uint8_t AutoRestartRxMode : 2;
    } bits;
    uint8_t reg;
} RegSyncConfig_t;

typedef struct
{
    uint8_t SyncValue;
} RegSyncValue1_t;

typedef struct
{
    uint8_t SyncValue;
} RegSyncValue2_t;

typedef struct
{
    uint8_t SyncValue;
} RegSyncValue3_t;

typedef struct
{
    uint8_t SyncValue;
} RegSyncValue4_t;

typedef struct
{
    uint8_t SyncValue;
} RegSyncValue5_t;

typedef struct
{
    uint8_t SyncValue;
} RegSyncValue6_t;

typedef struct
{
    uint8_t SyncValue;
} RegSyncValue7_t;

typedef struct
{
    uint8_t SyncValue;
} RegSyncValue8_t;

typedef union {
    struct
    {
        RegSyncValue1_t SyncValue1;
        RegSyncValue2_t SyncValue2;
        RegSyncValue3_t SyncValue3;
        RegSyncValue4_t SyncValue4;
        RegSyncValue5_t SyncValue5;
        RegSyncValue6_t SyncValue6;
        RegSyncValue7_t SyncValue7;
        RegSyncValue8_t SyncValue8;
    };
    uint64_t SyncValue;
} RegSyncValue_t;

typedef union {
    struct __attribute__((__packed__, aligned(1)))
    {
        uint8_t CrcWhiteningType : 1;
        uint8_t AddressFiltering : 2;
        uint8_t CrcAutoClearOff : 1;
        uint8_t CrcOn : 1;
        enum
        {
            NONE = 0,
            MANCHASTER,
            WHITENING
        } DcFree : 2;
        enum
        {
            FIXED_LENGTH = 0,
            VARIABLE_LENGTH
        } PacketFormat : 1;
    } bits;
    uint8_t reg;
} RegPacketConfig1_t;

typedef union {
    struct __attribute__((__packed__, aligned(1)))
    {
        uint8_t unused_7 : 1;
        enum
        {
            CONTINUOUS_MODE,
            PACKET_MODE
        } DataMode : 1;
        uint8_t IoHomeOn : 1;
        uint8_t IoHomePowerFrame : 1;
        uint8_t BeaconOn : 1;
        uint8_t PayloadLength_10_8 : 3;
    } bits;
    uint8_t reg;
} RegPacketConfig2_t;

typedef struct
{
    uint8_t reg;
} RegPayloadLength_t;

typedef struct
{
    uint8_t NodeAddress;
} RegNodeAdrs_t;

typedef struct
{
    uint8_t BroadcastAddress;
} RegBroadcastAdrs_t;

typedef union {
    struct __attribute__((__packed__, aligned(1)))
    {
        uint8_t FifoThreshold : 6;
        uint8_t unused_6 : 1;
        enum
        {
            TXSTARTCONDITION_FIFOTHRESH = 0x00, // Default
            TXSTARTCONDITION_FIFONOTEMPTY
        } TxStartCondition : 1;
    } bits;
    uint8_t reg;
} RegFifoThresh_t;

typedef union {
    struct
    {
        uint8_t FromTransmit : 1;
        uint8_t FromIdle : 1;
        uint8_t LowPowerSelection : 1;
        uint8_t FromStart : 2;
        uint8_t IdleMode : 1;
        uint8_t SequencerStop : 1;
        uint8_t SequencerStart : 1;
    } bits;
    uint8_t reg;
} RegSeqConfig1_t;

typedef union {
    struct
    {
        uint8_t FromPacketReceived : 3;
        uint8_t FromRxTimeout : 2;
        uint8_t FromReceive : 3;
    } bits;
    uint8_t reg;
} RegSeqConfig2_t;

typedef union {
    struct
    {
        uint8_t unused_7_4 : 4;
        uint8_t Timer1Resolution : 2;
        uint8_t Timer2Resolution : 2;
    } bits;
    uint8_t reg;
} RegTimerResol_t;

typedef struct
{
    uint8_t Timer1Coefficient;
} RegTimer1Coef_t;

typedef struct
{
    uint8_t Timer2Coefficient;
} RegTimer2Coef_t;

typedef union {
    struct
    {
        uint8_t TempMonitorOff : 1;
        uint8_t TempThreshold : 2;
        uint8_t TempChange : 1;
        uint8_t unused_4 : 1;
        uint8_t ImageCalRunning : 1;
        uint8_t ImageCalStart : 1;
        uint8_t AutoImageCalOn : 1;
    } bits;
    uint8_t reg;
} RegImageCal_t;

typedef struct
{
    uint8_t TempValue;
} RegTemp_t;

typedef union {
    struct
    {
        uint8_t LowBatTrim : 3;
        uint8_t LowBatOn : 1;
        uint8_t unused_7_4 : 4;
    } bits;
    uint8_t reg;
} RegLowBat_t;

/*!
* Status registers
*/
typedef union {
    struct
    {
        uint8_t SyncAddressMatch : 1;
        uint8_t PreambleDetect : 1;
        uint8_t Timeout : 1;
        uint8_t Rssi : 1;
        uint8_t PllLock : 1;
        uint8_t TxReady : 1;
        uint8_t RxReady : 1;
        uint8_t ModeReady : 1;
    } bits;
    uint8_t reg;
} RegIrqFlags1_t;

typedef union {
    struct
    {
        uint8_t LowBat : 1;
        uint8_t CrcOk : 1;
        uint8_t PayloadReady : 1;
        uint8_t PacketSent : 1;
        uint8_t FifoOverrun : 1;
        uint8_t FifoLevel : 1;
        uint8_t FifoEmpty : 1;
        uint8_t FifoFull : 1;
    } bits;
    uint8_t reg;
} RegIrqFlags2_t;

typedef union {
    struct
    {
        uint8_t Dio3Mapping : 2;
        uint8_t Dio2Mapping : 2;
        uint8_t Dio1Mapping : 2;
        uint8_t Dio0Mapping : 2;
    } bits;
    uint8_t reg;
} RegDioMapping1_t;

typedef union {
    struct
    {
        uint8_t MapPreambleDetect : 1;
        uint8_t reserved_3_1 : 3;
        uint8_t Dio5Mapping : 2;
        uint8_t Dio4Mapping : 2;
    } bits;
    uint8_t reg;
} RegDioMapping2_t;

typedef struct
{
    uint8_t Version;
} RegVersion_t;

typedef union {
    struct
    {
        uint8_t reserved_6_0 : 7;
        uint8_t FastHopOn : 1;
    } bits;
    uint8_t reg;
} RegPllHop_t;

typedef union {
    struct
    {
        uint8_t reserved_3_0 : 4;
        uint8_t TcxoInputOn : 1;
        uint8_t reserved_7_5 : 3;
    } bits;
    uint8_t reg;
} RegTcxo_t;

typedef union {
    struct
    {
        uint8_t PaDac : 3;
        uint8_t reserved_7_3 : 5;
    } bits;
    uint8_t reg;
} RegPaDac_t;

typedef struct
{
    uint8_t FormerTemp;
} RegFormerTemp_t;

typedef union {
    struct
    {
        uint8_t BitRateFrac : 4;
        uint8_t unused_7_4 : 4;
    } bits;
    uint8_t reg;
} RegBitRateFrac_t;

typedef union {
    struct
    {
        uint8_t AgcReferenceLevel : 6;
        uint8_t unused_7_6 : 2;
    } bits;
    uint8_t reg;
} RegAgcRef_t;

typedef union {
    struct
    {
        uint8_t AgcStep1 : 5;
        uint8_t unused_7_5 : 3;
    } bits;
    uint8_t reg;
} RegAgcThresh1_t;

typedef union {
    struct
    {
        uint8_t AgcStep3 : 4;
        uint8_t AgcStep2 : 4;
    } bits;
    uint8_t reg;
} RegAgcThresh2_t;

typedef union {
    struct
    {
        uint8_t AgcStep5 : 4;
        uint8_t AgcStep4 : 4;
    } bits;
    uint8_t reg;
} RegAgcThresh3_t;

typedef struct
{
    uint8_t RegPll;
} RegPll_t;

typedef struct __attribute__((__packed__, aligned(1)))
{
    RegFifo_t RegFifo; //0x00
        // Common settings
    RegOpMode_t RegOpMode;   //0x01
    RegBitrate_t RegBitrate; //0x02
    RegFdev_t RegFdev;       //0x04
    RegFrf_t RegFrf;         //0x06
        // Tx settings
    RegPaConfig_t RegPaConfig; //0x09
    RegPaRamp_t RegPaRamp;     //0x0A
    RegOcp_t RegOcp;           //0x0B
        // Rx settings
    RegLna_t RegLna;                       //0x0C
    RegRxConfig_t RegRxConfig;             //0x0D
    RegRssiConfig_t RegRssiConfig;         //0x0E
    RegRssiCollision_t RegRssiCollision;   //0x0F
    RegRssiThresh_t RegRssiThresh;         //0x10
    RegRssiValue_t RegRssiValue;           //0x11
    RegRxBw_t RegRxBw;                     //0x12
    RegAfcBw_t RegAfcBw;                   //0x13
    RegOokPeak_t RegOokPeak;               //0x14
    RegOokFix_t RegOokFix;                 //0x15
    RegOokAvg_t RegOokAvg;                 //0x16
    uint8_t Reserved17;                    //0x17
    uint8_t Reserved18;                    //0x18
    uint8_t Reserved19;                    //0x19
    RegAfcFei_t RegAfcFei;                 //0x1A
    RegAfc_t RegAfc;                       //0x1B
    RegFei_t RegFei;                       //0x1D
    RegPreambleDetect_t RegPreambleDetect; //0x1F
    RegRxTimeout1_t RegRxTimeout1;         //0x20
    RegRxTimeout2_t RegRxTimeout2;         //0x21
    RegRxTimeout3_t RegRxTimeout3;         //0x22
    RegRxDelay_t RegRxDelay;               //0x23
        // Oscillator settings
    RegOsc_t RegOsc; //0x24
        // Packet handler settings
    RegPreamble_t RegPreamble;           //0x25
    RegSyncConfig_t RegSyncConfig;       //0x27
    RegSyncValue_t RegSyncValue;         //0x28
    RegPacketConfig1_t RegPacketConfig1; //0x30
    RegPacketConfig2_t RegPacketConfig2; //0x31
    RegPayloadLength_t RegPayloadLength; //0x32
    RegNodeAdrs_t RegNodeAdrs;           //0x33
    RegBroadcastAdrs_t RegBroadcastAdrs; //0x34
    RegFifoThresh_t RegFifoThresh;       //0x35
        // Sequencer settings
    RegSeqConfig1_t RegSeqConfig1; //0x36
    RegSeqConfig2_t RegSeqConfig2; //0x37
    RegTimerResol_t RegTimerResol; //0x38
    RegTimer1Coef_t RegTimer1Coef; //0x39
    RegTimer2Coef_t RegTimer2Coef; //0x3A
        // Service settings
    RegImageCal_t RegImageCal; //0x3B
    RegTemp_t RegTemp;         //0x3C
    RegLowBat_t RegLowBat;     //0x3D
        // Status
    RegIrqFlags1_t RegIrqFlags1; //0x3E
    RegIrqFlags2_t RegIrqFlags2; //0x3F
        // I/O settings
    RegDioMapping1_t RegDioMapping1; //0x40
    RegDioMapping2_t RegDioMapping2; //0x41
                                     // Version
    RegVersion_t RegVersion;         //0x42
        // Additional settings
    RegPllHop_t RegPllHop;              //0x44
                                        // Test
    uint8_t reserved_0x45[0x4B - 0x45]; //0x45 - 0x4A
    RegTcxo_t RegTcxo;                  //0x4B
    uint8_t reserved_0x4C;              //0x4C
    RegPaDac_t RegPaDac;                //0x4D
        // Test
    uint8_t reserved_0x4E[0x5B - 0x4E]; //0x4E - 0x5A
    RegFormerTemp_t RegFormerTemp;      //0x5B
        // Test
    uint8_t reserved_0x5C;           //0x5C
    RegBitRateFrac_t RegBitRateFrac; //0x5D
        // Test
    uint8_t reserved_0x5E[0x61 - 0x5E]; //0x5E - 0x60
    RegAgcRef_t RegAgcRef;              //0x61
    RegAgcThresh1_t RegAgcThresh1;      //0x62
    RegAgcThresh2_t RegAgcThresh2;      //0x63
    RegAgcThresh3_t RegAgcThresh3;      //0x64
        // Test
    uint8_t reserved_0x65[0x70 - 0x65]; //0x65 - 0x6F
    RegPll_t RegPll;                    //0x70
} sx1276_reg_t;

extern sx1276_reg_t *sx1276;

#endif