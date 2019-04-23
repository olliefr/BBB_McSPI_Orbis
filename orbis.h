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

#define ORBIS_BUFFER_SIZE    (ORBIS_WORD_COUNT + 5)

#define ORBIS_CRC_OK    0u
#define ORBIS_CRC_FAIL  1u

extern uint8_t orbisDataRx[ORBIS_BUFFER_SIZE];
extern uint32_t orbisDataRxLength;

extern uint8_t orbisReceivedCRC;
extern uint8_t orbisCalculatedCRC;
extern uint8_t orbisCRCErrorFlag;

void OrbisSetup(void);
uint8_t OrbisCaptureGet(void);
uint8_t OrbisValidateCRC(void);
uint8_t OrbisCRC_Buffer(uint8_t* buffer, uint32_t numOfBytes);

#endif /* ORBIS_H_ */
