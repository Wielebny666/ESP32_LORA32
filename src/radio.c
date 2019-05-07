#include <stdio.h>
#include <string.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esp_log.h"

#include "board.h"
#include "radio.h"

static const char *TAG = "radio";

#define RF_FREQUENCY 915000000 // Hz

#define TX_OUTPUT_POWER 14 // dBm

#if defined(USE_MODEM_LORA)

#define LORA_BANDWIDTH 0        // [0: 125 kHz,   1: 250 kHz,   2: 500 kHz,   3: Reserved]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12]
#define LORA_CODINGRATE 1       // [1: 4/5,  2: 4/6,  3: 4/7,  4: 4/8]
#define LORA_PREAMBLE_LENGTH 8  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 5   // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#elif defined(USE_MODEM_FSK)

#define FSK_FDEV 25e3              // Hz
#define FSK_DATARATE 50e3          // bps
#define FSK_BANDWIDTH 50e3         // Hz
#define FSK_AFC_BANDWIDTH 83.333e3 // Hz
#define FSK_PREAMBLE_LENGTH 5      // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON false

#else
#error "Please define a modem in the compiler options."
#endif

typedef enum
{
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT,
} States_t;

#define RX_TIMEOUT_VALUE 1000
#define BUFFER_SIZE 64 // Define the payload size here

const uint8_t PingMsg[] = "PING";
const uint8_t PongMsg[] = "PONG";

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

States_t State = LOWPOWER;

int8_t RssiValue = 0;
int8_t SnrValue = 0;

/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone(void);

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout(void);

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout(void);

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError(void);

void task_radio(void *pvParameter)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    bool isMaster = true;
    uint8_t i;

    // Hardware initialisation
    SpiInit();

    // Radio initialization
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;

    Radio.Init(&RadioEvents);

    Radio.SetChannel(RF_FREQUENCY);

#if defined(USE_MODEM_LORA)

    Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                      LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                      LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

    Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                      LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                      LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                      0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

#elif defined(USE_MODEM_FSK)

    Radio.SetTxConfig(MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                      FSK_DATARATE, 0,
                      FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                      true, 0, 0, 0, 3000);

    Radio.SetRxConfig(MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                      0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                      0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                      0, 0, false, true);

#else
#error "Please define a frequency band in the compiler options."
#endif

    Radio.Rx(RX_TIMEOUT_VALUE);

    while (1)
    {
        switch (State)
        {
        case RX:
            ESP_LOGD(TAG, "State RX");
            if (isMaster == true)
            {
                if (BufferSize > 0)
                {
                    if (strncmp((const char *)Buffer, (const char *)PongMsg, 4) == 0)
                    {
                        // Send the next PING frame
                        Buffer[0] = 'P';
                        Buffer[1] = 'I';
                        Buffer[2] = 'N';
                        Buffer[3] = 'G';
                        // We fill the buffer with numbers for the payload
                        for (i = 4; i < BufferSize; i++)
                        {
                            Buffer[i] = i - 4;
                        }
                        vTaskDelay(1 / portTICK_PERIOD_MS);
                        Radio.Send(Buffer, BufferSize);
                    }
                    else if (strncmp((const char *)Buffer, (const char *)PingMsg, 4) == 0)
                    { // A master already exists then become a slave
                        isMaster = false;
                        Radio.Rx(RX_TIMEOUT_VALUE);
                    }
                    else // valid reception but neither a PING or a PONG message
                    {    // Set device as master ans start again
                        isMaster = true;
                        Radio.Rx(RX_TIMEOUT_VALUE);
                    }
                }
            }
            else
            {
                if (BufferSize > 0)
                {
                    if (strncmp((const char *)Buffer, (const char *)PingMsg, 4) == 0)
                    {
                        // Send the reply to the PONG string
                        Buffer[0] = 'P';
                        Buffer[1] = 'O';
                        Buffer[2] = 'N';
                        Buffer[3] = 'G';
                        // We fill the buffer with numbers for the payload
                        for (i = 4; i < BufferSize; i++)
                        {
                            Buffer[i] = i - 4;
                        }
                        vTaskDelay(1 / portTICK_PERIOD_MS);
                        Radio.Send(Buffer, BufferSize);
                    }
                    else // valid reception but not a PING as expected
                    {    // Set device as master and start again
                        isMaster = true;
                        Radio.Rx(RX_TIMEOUT_VALUE);
                    }
                }
            }
            State = LOWPOWER;
            break;
        case TX:
            ESP_LOGD(TAG, "State TX");

            Radio.Rx(RX_TIMEOUT_VALUE);
            State = LOWPOWER;
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            ESP_LOGD(TAG, "State ERROR/TIOMOUT");

            if (isMaster == true)
            {
                // Send the next PING frame
                Buffer[0] = 'P';
                Buffer[1] = 'I';
                Buffer[2] = 'N';
                Buffer[3] = 'G';
                for (i = 4; i < BufferSize; i++)
                {
                    Buffer[i] = i - 4;
                }
                vTaskDelay(1 / portTICK_PERIOD_MS);
                Radio.Send(Buffer, BufferSize);
            }
            else
            {
                Radio.Rx(RX_TIMEOUT_VALUE);
            }
            State = LOWPOWER;
            break;
        case TX_TIMEOUT:
            ESP_LOGD(TAG, "State TX_TIMEOUT");

            Radio.Rx(RX_TIMEOUT_VALUE);
            State = LOWPOWER;
            break;
        case LOWPOWER:
        default:
            ESP_LOGD(TAG, "State LOWPOWER/DEFAULT");
            // Set low power
            break;
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);

        //TimerLowPowerHandler();
    }
    vTaskDelete(NULL);
}

void OnTxDone(void)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    Radio.Sleep();
    State = TX;
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    Radio.Sleep();
    BufferSize = size;
    memcpy(Buffer, payload, BufferSize);
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;
}

void OnTxTimeout(void)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    Radio.Sleep();
    State = TX_TIMEOUT;
}

void OnRxTimeout(void)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    Radio.Sleep();
    State = RX_TIMEOUT;
}

void OnRxError(void)
{
    ESP_LOGD(TAG, "%s", __FUNCTION__);

    Radio.Sleep();
    State = RX_ERROR;
}