/*
 * orbis.h
 * Orbis rotary encoder driver for TI StarterWare
 *
 * The functions and global data structures are documented
 * in the source code file to avoid saying the same thing twice.
 * Use the source code file as a primary reference.
 *
 *  Created on: 22 Apr 2019
 *      Author: Oliver Frolovs
 */

#ifndef ORBIS_H_
#define ORBIS_H_

#define MCSPI_IN_CLK                  48000000u
#define MCSPI_ORBIS_OUT_FREQ           3000000u
#define ORBIS_SPI_CHANNEL                    0u

#define ORBIS_BITS_PER_WORD                  8u
#define ORBIS_WORD_COUNT                     5u
#define ORBIS_BIT_MASK                    0xFFu

#define ORBIS_CRC_OK    0u
#define ORBIS_CRC_FAIL  1u

// TODO something is wrong with the timer as I can see on the scope; this results in about 8 microsec
#define ORBIS_DELAY_SINGLE            TIMER_1US
#define ORBIS_DELAY_MULTI       (2 * TIMER_1US)

//
// Orbis command set
//
#define ORBIS_CMD_NONE          0x00u
#define ORBIS_CMD_SERIAL        0x76u
#define ORBIS_CMD_SPEED         0x73u
#define ORBIS_CMD_TEMPERATURE   0x74u
#define ORBIS_CMD_STATUS        0x64u

//
// The sizes of parts of responses supported by Orbis,
// both single- and multi- turn. In bytes. Each byte is 8 bits.
// This information closely follows page 14 of Orbis datasheet.
//
#define ORBIS_SIZE_MULTITURN     2
#define ORBIS_SIZE_POSITION      2
#define ORBIS_SIZE_SERIAL        6
#define ORBIS_SIZE_SPEED         2
#define ORBIS_SIZE_TEMPERATURE   2
#define ORBIS_SIZE_STATUS        1
#define ORBIS_SIZE_CRC           1
#define ORBIS_SIZE_BUFFER        (ORBIS_SIZE_MULTITURN + ORBIS_SIZE_POSITION + ORBIS_SIZE_SERIAL + ORBIS_SIZE_CRC)

extern volatile uint8_t orbisDataRx[ORBIS_SIZE_BUFFER];
extern volatile uint32_t orbisDataRxLength;
extern volatile uint32_t orbisReady;

extern uint8_t orbisReceivedCRC;
extern uint8_t orbisCalculatedCRC;
extern uint8_t orbisCRCErrorFlag;

void OrbisSetup(void);
void orbisMcSPIIsr(void);
uint8_t OrbisCaptureGet(void);
uint8_t OrbisValidateCRC(void);
uint8_t OrbisCRC_Buffer(volatile uint8_t* buffer, uint32_t numOfBytes);

// TODO maybe OrbisPrintDataFrame() for debug?

#endif /* ORBIS_H_ */
