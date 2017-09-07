/*
 * File Name: trf797x.h
 *
 * Description: Headers and Defines for TRF797x Driver Functions
 *
 * Copyright (C) 2016 Texas Instruments Incorporated - http://www.ti.com/
 *
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _TRF79xxa_H_
#define _TRF79xxa_H_

//=====Driver Configuration======================================

/*
 * Please define the version of the TRF79xxA chip being used (Use
 * decimal, not hex for the numbers)
 *
 *  Current Support:
 *  - TRF7970A, TRF7964A* ==> 70
 *  - TRF7960A ==> 60
 *
 *  Eg:
 *  #define TRF79xxA_VERSION 70 // Set version to TRF7970A
 *
 *  * The TRF7964A is a derivative of the TRF7970A and thus uses
 *  all the same settings as the TRF7970A.
 */
#ifndef TRF79xxA_VERSION
#error "Please define the TRF79xxA version to be used!"
#elif (TRF79xxA_VERSION != 60) && (TRF79xxA_VERSION != 70)
#error "Invalid TRF79xxA version specified!"
#endif

/*
 * This defines the largest FIFO size of the TRF79xxA chip
 * TRF7970A and TRF7964A - 127 bytes
 * TRF7960A, TRF7961A, TRF7962A, and TRF7963A - 12 bytes
 */
#if (TRF79xxA_VERSION == 70)
#define TRF79xxA_MAX_FIFO_SIZE          127
#elif (TRF79xxA_VERSION == 60)
#define TRF79xxA_MAX_FIFO_SIZE          12
#endif

/*
 * This defines the size of the FIFO buffer generated. This size
 * comes out of RAM memory. For example, setting this to 100 takes
 * up 100 bytes of RAM. Recommendation is to reduce this if the
 * full buffer would not be used in certain applications.
 *
 * Eg:
 * #define NFC_FIFO_SIZE 100
 *
 */
#ifndef NFC_FIFO_SIZE
#error "Please define the size of the NFC FIFO to store in RAM!"
#endif

//===============================================================

//=====TRF79xxA API Types========================================

/*
 * Power Settings for TRF79xxA.
 *
 * Each definition is based of the setting which would required
 * in the Chip Status Control Register. These settings do not
 * differ across the TRF79xxA family.
 *
 */
typedef enum
{
    TRF79xxA_3V_FULL_POWER = 0x00,			// Full Power, 3V
    TRF79xxA_5V_FULL_POWER = 0x01,			// Full Power, 5V
    TRF79xxA_3V_HALF_POWER = 0x10,			// Half Power, 3V
    TRF79xxA_5V_HALF_POWER = 0x11,			// Half Power, 5V
} tTrfPowerOptions;

// Structure for various TRF79xxA Settings
typedef struct
{
    uint8_t ui8IsoControl;
    bool bRfFieldOn; // RF Field Status
    tTrfPowerOptions eTrfPowerSetting;
} tTrfSettings;

// TRF79xxA status
typedef enum
{
    TRF_IDLE,
    TX_COMPLETE,
    RX_COMPLETE,
    TX_ERROR,
    RX_WAIT,
    RX_WAIT_EXTENSION,
    TX_WAIT,
    PROTOCOL_ERROR,
    COLLISION_ERROR,
    NO_RESPONSE_RECEIVED,
    NO_RESPONSE_RECEIVED_15693
} tTrfStatus;

//===============================================================

//===============================================================
//==== Version Control ==========================================

#define RFID_DRIVER_VERSION     "0.1.0"
#define RFID_DRIVER_DATE        "Sep 6th, 2017"

//===============================================================

//==== TRF797x Definitions ======================================

//---- Direct commands ------------------------------------------
#define TRF79XXA_IDLE_CMD                               0x00
#define TRF79XXA_SOFT_INIT_CMD                          0x03
#if (TRF79xxA_VERSION == 70)
#define TRF79XXA_INITIAL_RF_COLLISION_AVOID_CMD         0x04
#define TRF79XXA_PERFORM_RES_RF_COLLISION_AVOID_CMD     0x05
#define TRF79XXA_PERFORM_RES_RF_COLLISION_AVOID_N0_CMD  0x06
#endif
#define TRF79XXA_RESET_FIFO_CMD                         0x0F
#define TRF79XXA_TRANSMIT_NO_CRC_CMD                    0x10
#define TRF79XXA_TRANSMIT_CRC_CMD                       0x11
#define TRF79XXA_DELAY_TRANSMIT_NO_CRC_CMD              0x12
#define TRF79XXA_DELAY_TRANSMIT_CRC_CMD                 0x13
#define TRF79XXA_TRANSMIT_NEXT_SLOT_CMD                 0x14
#if (TRF79xxA_VERSION == 70)
#define TRF79XXA_CLOSE_SLOT_SEQUENCE_CMD                0x15
#endif
#define TRF79XXA_STOP_DECODERS_CMD                      0x16
#define TRF79XXA_RUN_DECODERS_CMD                       0x17
#define TRF79XXA_TEST_INTERNAL_RF_CMD                   0x18
#define TRF79XXA_TEST_EXTERNAL_RF_CMD                   0x19

//---- Registers ------------------------------------------------
#define TRF79XXA_CHIP_STATUS_CONTROL                    0x00
#define TRF79XXA_ISO_CONTROL                            0x01
#define TRF79XXA_ISO14443B_TX_OPTIONS                   0x02
#define TRF79XXA_ISO14443A_BITRATE_OPTIONS              0x03
#define TRF79XXA_TX_TIMER_EPC_HIGH                      0x04
#define TRF79XXA_TX_TIMER_EPC_LOW                       0x05
#define TRF79XXA_TX_PULSE_LENGTH_CONTROL                0x06
#define TRF79XXA_RX_NO_RESPONSE_WAIT_TIME               0x07
#define TRF79XXA_RX_WAIT_TIME                           0x08
#define TRF79XXA_MODULATOR_CONTROL                      0x09
#define TRF79XXA_RX_SPECIAL_SETTINGS                    0x0A
#define TRF79XXA_REGULATOR_CONTROL                      0x0B
#define TRF79XXA_IRQ_STATUS                             0x0C
#define TRF79XXA_IRQ_MASK                               0x0D
#define	TRF79XXA_COLLISION_POSITION                     0x0E
#define TRF79XXA_RSSI_LEVELS                            0x0F
// Special functions register not documented on TRF7960A
// datasheet. See sloa155 Section 7.8
#define TRF79XXA_SPECIAL_FUNCTION_1                     0x10
#if (TRF79xxA_VERSION == 70)
#define TRF79XXA_SPECIAL_FUNCTION_2                     0x11
#define TRF79XXA_FIFO_IRQ_LEVELS                        0x14
#define TRF79XXA_NFC_LOW_DETECTION_LEVEL                0x16
#define TRF79XXA_NFC_ID_REG                             0x17
#define TRF79XXA_NFC_TARGET_LEVEL                       0x18
#define TRF79XXA_NFC_TARGET_PROTOCOL                    0x19
#endif
#define TRF79XXA_TEST_SETTINGS_1                        0x1A
#define TRF79XXA_TEST_SETTINGS_2                        0x1B
#define TRF79XXA_FIFO_STATUS                            0x1C
#define TRF79XXA_TX_LENGTH_BYTE_1                       0x1D
#define TRF79XXA_TX_LENGTH_BYTE_2                       0x1E
#define TRF79XXA_FIFO                                   0x1F

//---- IRQ STATUS -----------------------------------------------
#define TRF79XXA_IRQ_STATUS_IDLE                        0x00
#define TRF79XXA_IRQ_STATUS_NO_RESPONSE                 0x01
#define TRF79XXA_IRQ_STATUS_COLLISION_ERROR             0x02
#define TRF79XXA_IRQ_STATUS_FRAMING_ERROR               0x04
#define TRF79XXA_IRQ_STATUS_PARITY_ERROR                0x08
#define TRF79XXA_IRQ_STATUS_CRC_ERROR                   0x10
#define TRF79XXA_IRQ_STATUS_FIFO_HIGH_OR_LOW            0x20
#define TRF79XXA_IRQ_STATUS_RX_COMPLETE                 0x40
#define TRF79XXA_IRQ_STATUS_TX_COMPLETE                 0x80

//===============================================================

//=====TRF79xxA API Functions====================================

void TRF79xxA_disableSlotCounter(void);
void TRF79xxA_enableSlotCounter(void);
void TRF79xxA_initialSettings(void);
void TRF79xxA_writeRaw(uint8_t * pui8Payload, uint8_t ui8Length);
void TRF79xxA_readContinuous(uint8_t * pui8Payload,
                             uint8_t ui8Length);
uint8_t TRF79xxA_readRegister(uint8_t ui8TrfRegister);
void TRF79xxA_reset(void);
void TRF79xxA_resetFIFO(void);
void TRF79xxA_resetIrqStatus(void);
void TRF79xxA_sendDirectCommand(uint8_t ui8Value);
void TRF79xxA_setupInitiator(uint8_t ui8IsoControl);
void TRF79xxA_turnRfOff(void);
void TRF79xxA_turnRfOn(void);
void TRF79xxA_writeContinuous(uint8_t * pui8Payload,
                              uint8_t ui8Length);
void TRF79xxA_writeRegister(uint8_t ui8TrfRegister,
                            uint8_t ui8Value);
void TRF79xxA_waitTxIRQ(uint8_t ui8TxTimeout);
void TRF79xxA_waitRxIRQ(uint8_t ui8RxTimeout);
tTrfStatus TRF79xxA_waitRxData(uint8_t ui8TxTimeout,
                               uint8_t ui8RxTimeout);
tTrfStatus TRF79xxA_getTrfStatus(void);
uint8_t TRF79xxA_getCollisionPosition(void);
void TRF79xxA_setCollisionPosition(uint8_t ui8ColPos);
void TRF79xxA_setTrfStatus(tTrfStatus sTrfStatus);
void TRF79xxA_setTrfPowerSetting(tTrfPowerOptions sTrfPowerSetting);
uint8_t TRF79xxA_getRxBytesReceived(void);
uint8_t * TRF79xxA_getTrfBuffer(void);
uint8_t TRF79xxA_getIsoControlValue(void);
bool TRF79xxA_checkExternalRfField(void);

//===============================================================

//=====TRF79xxA API Externs======================================

extern void trf79xxaSpi_directCommand(uint8_t cmd);
extern void trf79xxaSpi_setup();
extern void trf79xxaSpi_rawWrite(uint8_t *payload,
                                 uint8_t len,
                                 bool continuedSend);
extern void trf79xxaSpi_writeSingle(uint8_t *value);
extern void trf79xxaSpi_writeCont(uint8_t *payload, uint8_t len);
extern void trf79xxaSpi_readSingle(uint8_t *value);
extern void trf79xxaSpi_readCont(uint8_t *payload, uint8_t len);

//===============================================================

#endif // _TRF79xxa_H_
