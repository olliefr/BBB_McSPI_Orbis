/*
 * orbis.c
 * Orbis Rotary Encoder driver for TI StarterWare
 *
 * This driver uses McSPI0 module channel 0 in single-channel, four-pin,
 * FIFO-disabled, Tx/Rx mode. The data is read by polling the channel
 * status register MCSPI_CHxSTAT. "GetSpeed" command is continuously
 * transmitted to the encoder, thus it returns position, and speed.
 * The exception is made for the first data frame which contains
 * the command to be sent, the rest of the frames are processed inside the loop.
 *
 *  Created on: 22 Apr 2019
 *      Author: Oliver Frolovs
 */
#include <stdint.h>
#include "hw_types.h"
#include "soc_AM335x.h"
#include "beaglebone.h"
#include "pin_mux.h"
#include "gpio_v2.h"
#include "dmtimer.h"
#include "hw_mcspi.h"
#include "mcspi.h"
#include "mcspi_beaglebone.h"
#include "orbis.h"
#include "util.h"

// The buffer for data read from Orbis, including the CRC. The size of response depends on the command
// transmitted. Allocate enough memory to store the longest possible response.
uint8_t orbisDataRx[ORBIS_BUFFER_SIZE];

// The length of the response to the last transmitted command, including the CRC.
uint32_t orbisDataRxLength;

// The CRC as it has been read from SPI. The CRC is the last byte transmitted. So the value
// comes from orbisDataRx[orbisDataRxLength-1].
uint8_t orbisReceivedCRC;

// The CRC as it has been calculated from the contents of orbisDataRx buffer.
uint8_t orbisCalculatedCRC;

// Sticky flag to indicate that there was a CRC error. Takes values from {ORBIS_CRC_OK, ORBIS_CRC_FAIL}.
uint8_t orbisCRCErrorFlag;

//
// Orbis CRC calculation table representing 0x97 polynome. Adapted from the Appendix 1 of the Orbis datasheet.
//
static uint8_t orbisTableCRC[256] = {
    0x00, 0x97, 0xB9, 0x2E, 0xE5, 0x72, 0x5C, 0xCB, 0x5D, 0xCA, 0xE4, 0x73, 0xB8, 0x2F, 0x01, 0x96,
    0xBA, 0x2D, 0x03, 0x94, 0x5F, 0xC8, 0xE6, 0x71, 0xE7, 0x70, 0x5E, 0xC9, 0x02, 0x95, 0xBB, 0x2C,
    0xE3, 0x74, 0x5A, 0xCD, 0x06, 0x91, 0xBF, 0x28, 0xBE, 0x29, 0x07, 0x90, 0x5B, 0xCC, 0xE2, 0x75,
    0x59, 0xCE, 0xE0, 0x77, 0xBC, 0x2B, 0x05, 0x92, 0x04, 0x93, 0xBD, 0x2A, 0xE1, 0x76, 0x58, 0xCF,
    0x51, 0xC6, 0xE8, 0x7F, 0xB4, 0x23, 0x0D, 0x9A, 0x0C, 0x9B, 0xB5, 0x22, 0xE9, 0x7E, 0x50, 0xC7,
    0xEB, 0x7C, 0x52, 0xC5, 0x0E, 0x99, 0xB7, 0x20, 0xB6, 0x21, 0x0F, 0x98, 0x53, 0xC4, 0xEA, 0x7D,
    0xB2, 0x25, 0x0B, 0x9C, 0x57, 0xC0, 0xEE, 0x79, 0xEF, 0x78, 0x56, 0xC1, 0x0A, 0x9D, 0xB3, 0x24,
    0x08, 0x9F, 0xB1, 0x26, 0xED, 0x7A, 0x54, 0xC3, 0x55, 0xC2, 0xEC, 0x7B, 0xB0, 0x27, 0x09, 0x9E,
    0xA2, 0x35, 0x1B, 0x8C, 0x47, 0xD0, 0xFE, 0x69, 0xFF, 0x68, 0x46, 0xD1, 0x1A, 0x8D, 0xA3, 0x34,
    0x18, 0x8F, 0xA1, 0x36, 0xFD, 0x6A, 0x44, 0xD3, 0x45, 0xD2, 0xFC, 0x6B, 0xA0, 0x37, 0x19, 0x8E,
    0x41, 0xD6, 0xF8, 0x6F, 0xA4, 0x33, 0x1D, 0x8A, 0x1C, 0x8B, 0xA5, 0x32, 0xF9, 0x6E, 0x40, 0xD7,
    0xFB, 0x6C, 0x42, 0xD5, 0x1E, 0x89, 0xA7, 0x30, 0xA6, 0x31, 0x1F, 0x88, 0x43, 0xD4, 0xFA, 0x6D,
    0xF3, 0x64, 0x4A, 0xDD, 0x16, 0x81, 0xAF, 0x38, 0xAE, 0x39, 0x17, 0x80, 0x4B, 0xDC, 0xF2, 0x65,
    0x49, 0xDE, 0xF0, 0x67, 0xAC, 0x3B, 0x15, 0x82, 0x14, 0x83, 0xAD, 0x3A, 0xF1, 0x66, 0x48, 0xDF,
    0x10, 0x87, 0xA9, 0x3E, 0xF5, 0x62, 0x4C, 0xDB, 0x4D, 0xDA, 0xF4, 0x63, 0xA8, 0x3F, 0x11, 0x86,
    0xAA, 0x3D, 0x13, 0x84, 0x4F, 0xD8, 0xF6, 0x61, 0xF7, 0x60, 0x4E, 0xD9, 0x12, 0x85, 0xAB, 0x3C
};

// Configure McSPI0 controller and channel for communication with Orbis rotary encoder.
void OrbisSetup(void)
{
    // Surrounding modules and other global initialisation...

    // Pin muxing
    GpioPinMuxSetup(CONTROL_CONF_SPI0_SCLK, PAD_FS_RXE_NA_PUPDD(0));
    GpioPinMuxSetup(CONTROL_CONF_SPI0_D0,   PAD_FS_RXD_NA_PUPDD(0));
    GpioPinMuxSetup(CONTROL_CONF_SPI0_D1,   PAD_FS_RXE_PU_PUPDE(0));
    GpioPinMuxSetup(CONTROL_CONF_SPI0_CS0,  PAD_FS_RXD_NA_PUPDD(0));

    // Enable clock to the module
    McSPI0ModuleClkConfig();

    // Soft reset (waiting included)
    McSPIReset(SOC_SPI_0_REGS);

    // Module-wide options...

    // We are going to use CS pins, enable four-pin mode
    McSPICSEnable(SOC_SPI_0_REGS);

    // Put the module into master mode as it is in slave mode after soft reset
    McSPIMasterModeEnable(SOC_SPI_0_REGS);

    // Channel options...

    // MCSPI_DATA_LINE_COMM_MODE_6 = D0 enabled for output; receive on D1
    McSPIMasterModeConfig(SOC_SPI_0_REGS, MCSPI_SINGLE_CH,
                          MCSPI_TX_RX_MODE, MCSPI_DATA_LINE_COMM_MODE_6,
                          ORBIS_SPI_CHANNEL);

    // Set D1 to be an input at module level. Why doesn't StarterWare do that? I checked the source!
    HWREG(SOC_SPI_0_REGS + MCSPI_SYST) |= (1 << 9);

    // Orbis loads data on rising clk edge, read it on the falling edge. Idle clock is low.
    McSPIClkConfig(SOC_SPI_0_REGS, MCSPI_IN_CLK, MCSPI_ORBIS_OUT_FREQ, ORBIS_SPI_CHANNEL,
                   MCSPI_CLK_MODE_1);
    McSPICSPolarityConfig(SOC_SPI_0_REGS, MCSPI_CS_POL_LOW, ORBIS_SPI_CHANNEL);

    // Set SPI word length
    McSPIWordLengthSet(SOC_SPI_0_REGS, MCSPI_WORD_LENGTH(ORBIS_BITS_PER_WORD), ORBIS_SPI_CHANNEL);

    // No FIFO
    McSPIRxFIFOConfig(SOC_SPI_0_REGS, MCSPI_RX_FIFO_DISABLE, ORBIS_SPI_CHANNEL);
    McSPITxFIFOConfig(SOC_SPI_0_REGS, MCSPI_TX_FIFO_DISABLE, ORBIS_SPI_CHANNEL);

    // Reset all interrupt status bits for channel 0 (although interrupt status register is not used in this example)
    McSPIIntStatusClear(SOC_SPI_0_REGS, MCSPI_INT_TX_EMPTY(ORBIS_SPI_CHANNEL) | MCSPI_INT_RX_FULL(ORBIS_SPI_CHANNEL) | MCSPI_INT_EOWKE);
}

// Returns ORBIS_CRC_OK or ORBIS_CRC_FAIL
uint8_t OrbisCaptureGet(void)
{
    // Just an ordinary null request for now as we don't yet have TX capability. Orbis will
    // respond with position information (16 bit single-turn, 32 bit multi-turn) and CRC (8 bit) only.
    orbisDataRxLength = ORBIS_SIZE_POSITION + ORBIS_SIZE_SPEED + ORBIS_SIZE_CRC;

    // We are the only device on this SPI bus, so can enable the channel without checking
    // if there is any activity on the bus. The AM335x TRM (24.4.1.9) claims that this action
    // sets MCSPI_CHxSTAT[TXS] bit to indicate that the channel's Tx register is empty, but
    // this does not happen.
    McSPIChannelEnable(SOC_SPI_0_REGS, ORBIS_SPI_CHANNEL);

    // Assert CS manually as we are in four-pin mode. This will set MCSPI_CHxSTAT[TXS] bit,
    // to indicate that the channel's Tx register is empty. This behaviour is a deviation
    // from the AM335x TRM.
    McSPICSAssert(SOC_SPI_0_REGS, ORBIS_SPI_CHANNEL);

    // Wait for Orbis to prepare the transmission. Multi-turn Oribs needs at least 7.2 micro-seconds,
    // and for single turn Orbis 1.25 micro-seconds is enough.
    // TODO something is wrong with the timer as I can see on the scope; this results in about 8 microsec
    waitfor(2 * TIMER_1US);

    // Now the command can be transmitted...

    // The channel is in the Tx/Rx mode, so need to keep refilling the Tx register.
    McSPITransmitData(SOC_SPI_0_REGS, ORBIS_CMD_SPEED, ORBIS_SPI_CHANNEL);

    // Wait until the transfer is complete, otherwise the register contains rubbish
    while (MCSPI_CH_STAT_EOT != (MCSPI_CH_STAT_EOT & McSPIChannelStatusGet(SOC_SPI_0_REGS, 0)));

    // Read the response from Rx register
    orbisDataRx[0] = McSPIReceiveData(SOC_SPI_0_REGS, ORBIS_SPI_CHANNEL) & ORBIS_BIT_MASK;

    // Continue transmitting and receiving until the full response is not read...

    // Transmit dummy frames and read the remaining response frames. The controller in Tx/Rx mode
    // will not clock out an extra frame at the end.
    for (uint32_t i = 1; i < orbisDataRxLength; i++) {

        // The channel is in the Tx/Rx mode, so need to keep refilling the Tx register.
        McSPITransmitData(SOC_SPI_0_REGS, ORBIS_CMD_NONE, ORBIS_SPI_CHANNEL);

        // Wait until the transfer is complete, otherwise the register contains rubbish
        while (MCSPI_CH_STAT_EOT != (MCSPI_CH_STAT_EOT & McSPIChannelStatusGet(SOC_SPI_0_REGS, 0)));

        // Read the response from Rx register
        orbisDataRx[i] = McSPIReceiveData(SOC_SPI_0_REGS, ORBIS_SPI_CHANNEL) & ORBIS_BIT_MASK;
    }

    // We are done transmitting the data, deassert CS
    McSPICSDeAssert(SOC_SPI_0_REGS, ORBIS_SPI_CHANNEL);

    // Disable the channel
    McSPIChannelDisable(SOC_SPI_0_REGS, ORBIS_SPI_CHANNEL);

    // Validate CRC and report back
    uint8_t isValidCRC = OrbisValidateCRC();

    return isValidCRC;
}

//
// Calculate and store CRC for Orbis response. Set a global flag, if there is a CRC error.
//
// Sets global variables orbisReceivedCRC, orbisCalculatedCRC, and orbisCRCErrorFlag.
// Returns ORBIS_CRC_OK if validation is successful, ORBIS_CRC_FAIL otherwise.
//
uint8_t OrbisValidateCRC(void)
{
    uint8_t isValidCRC = ORBIS_CRC_FAIL;

    orbisReceivedCRC = (uint8_t) ~orbisDataRx[orbisDataRxLength - 1];
    orbisCalculatedCRC = OrbisCRC_Buffer(orbisDataRx, orbisDataRxLength - 1);

    isValidCRC = (orbisReceivedCRC == orbisCalculatedCRC) ? ORBIS_CRC_OK : ORBIS_CRC_FAIL;

    // The CRC error flag is sticky
    if (ORBIS_CRC_FAIL == isValidCRC)
        orbisCRCErrorFlag = ORBIS_CRC_FAIL;

    return isValidCRC;
}

//
// Calculate CRC from fixed length buffer with 0x97 polynome.
// Adapted from the Appendix 1 of the Orbis data sheet.
// Input: pointer to the buffer, and how many bytes from
// the buffer to use to calculate CRC. The CRC is 8 bit and the length
// of the preceding data depends on the request/response type.
//
uint8_t OrbisCRC_Buffer(uint8_t* buffer, uint32_t numOfBytes)
{
    uint32_t t;
    uint8_t icrc;

    numOfBytes -= 1;
    icrc = 1;
    t = buffer[0];
    while (numOfBytes--)
    {
        // TODO is the way ^ is done and assigned compatible with SEI CERT C Coding Standard section INT02-C?
        t = buffer[icrc++] ^ orbisTableCRC[t];
    }

    return orbisTableCRC[t];
}
