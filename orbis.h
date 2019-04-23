/*
 * orbis.h
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

// Initialise McSPI0 module and set-up channel 0 for Orbis
void OrbisSetup(void);

// This function works by polling --
// Send a dummy request and read Orbis response.
// Return ORBIS_CRC_OK if CRC computes, ORBIS_FAIL otherwise
uint8_t OrbisCaptureGet(void);

// Computes the CRC for the data packet in orbisDataRx
// Updates globals orbisReceivedCRC, orbisCalculatedCRC, and orbisCRCErrorFlag
// Returns ORBIS_CRC_OK or ORBIS_CRC_FAIL
uint8_t OrbisValidateCRC(void);

//
// Calculate CRC from fixed length buffer with 0x97 polynome.
// Adapted from the Appendix 1 of the Orbis data sheet.
// Input: pointer to the buffer, and how many bytes from
// the buffer to use to calculate CRC
//
uint8_t OrbisCRC_Buffer(uint8_t* buffer, uint32_t numOfBytes);

#endif /* ORBIS_H_ */
