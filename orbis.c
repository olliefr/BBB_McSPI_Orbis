/*
 * orbis.c
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

#include "consoleUtils.h"

uint8_t orbisDataRx[ORBIS_BUFFER_SIZE];
uint32_t orbisDataRxLength;

uint8_t orbisReceivedCRC;
uint8_t orbisCalculatedCRC;
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
    0xAA, 0x3D, 0x13, 0x84, 0x4F, 0xD8, 0xF6, 0x61, 0xF7, 0x60, 0x4E, 0xD9, 0x12, 0x85, 0xAB, 0x3C};


void OrbisSetup(void)
{
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

    // MCSPI_DATA_LINE_COMM_MODE_7 = D0 & D1 disabled for output; receive on D1
    McSPIMasterModeConfig(SOC_SPI_0_REGS, MCSPI_SINGLE_CH,
                          MCSPI_RX_ONLY_MODE, MCSPI_DATA_LINE_COMM_MODE_7,
                          ORBIS_SPI_CHANNEL);

    // Set D1 to be an input at module level. Why doesn't StarterWare do that? I checked the source!
    HWREG(SOC_SPI_0_REGS + MCSPI_SYST) |= (1 << 9);


    // Orbis loads data on rising clk edge, read it on the falling, idle low clock edge.
    // The first bit is loaded when CS goes low, thus it can be read when the clock starts
    McSPIClkConfig(SOC_SPI_0_REGS, MCSPI_IN_CLK, MCSPI_ORBIS_OUT_FREQ, ORBIS_SPI_CHANNEL,
                   MCSPI_CLK_MODE_1);
    McSPICSPolarityConfig(SOC_SPI_0_REGS, MCSPI_CS_POL_LOW, ORBIS_SPI_CHANNEL);

    // Set SPI word length
    McSPIWordLengthSet(SOC_SPI_0_REGS, MCSPI_WORD_LENGTH(ORBIS_BITS_PER_WORD), ORBIS_SPI_CHANNEL);

    // No FIFO to begin with
    McSPIRxFIFOConfig(SOC_SPI_0_REGS, MCSPI_RX_FIFO_DISABLE, ORBIS_SPI_CHANNEL);
    McSPITxFIFOConfig(SOC_SPI_0_REGS, MCSPI_TX_FIFO_DISABLE, ORBIS_SPI_CHANNEL);

    // Reset all interrupt status bits for channel 0
    McSPIIntStatusClear(SOC_SPI_0_REGS, 0xFFFFFFFFu);
}

// Returns ORBIS_CRC_OK or ORBIS_CRC_FAIL
uint8_t OrbisCaptureGet(void)
{
    // Just an ordinary null request for now as we don't yet have TX capability
    orbisDataRxLength = 3;

    // We are the only device on the SPI bus, so enable the channel!
    McSPIChannelEnable(SOC_SPI_0_REGS, ORBIS_SPI_CHANNEL);

    // Assert CS, we are the only users of SPI for now
    McSPICSAssert(SOC_SPI_0_REGS, ORBIS_SPI_CHANNEL);

    // Wait for at least 7.2 micro-seconds
    waitfor(TIMER_1US);

    //uint32_t intCode = McSPIIntStatusGet(SOC_SPI_0_REGS);

    // Read one less word than necessary because in single-channel, four-pin, FIFO-disabled, RX-only mode
    // the controller does not like Rx register empty, so once the contents of the register is read, it
    // clocks out and reads another (extra) frame. Therefore at the last iteration of this loop,
    // the last data frame has already been transmitted into the controller and is waiting in the Rx
    // register. After the loop, the channel is switched off to prevent the transmission of an extra
    // data frame when the Rx register is read.
    for (uint32_t i = 0; i < orbisDataRxLength - 1; i++) {

        // The channel is in Rx mode, so only need to fill the Tx register with dummy data once
        // Alas, cannot do it before CS is asserted, this is not mentioned in the documentation,
        // instead it says that this bit is sent when the channel is "enabled", which is not the case,
        // as I have verified [see LOG 23/04/19]
        //if (i == 0) {
            McSPITransmitData(SOC_SPI_0_REGS, 0, ORBIS_SPI_CHANNEL);
            //while (MCSPI_CH1STAT_EOT != (MCSPI_CH1STAT_EOT & McSPIChannelStatusGet(SOC_SPI_0_REGS, 0)));
         //   orbisDataRx[i] = McSPIReceiveData(SOC_SPI_0_REGS, ORBIS_SPI_CHANNEL) & ORBIS_BIT_MASK;
        ///}
        // Wait until the transfer is complete, otherwise garbage is read
        while (MCSPI_CH1STAT_EOT != (MCSPI_CH1STAT_EOT & McSPIChannelStatusGet(SOC_SPI_0_REGS, 0)));

        // Read the response from Rx register
        orbisDataRx[i] = McSPIReceiveData(SOC_SPI_0_REGS, ORBIS_SPI_CHANNEL) & ORBIS_BIT_MASK;
    }
    // We are done transmitting the data, deassert CS
    McSPICSDeAssert(SOC_SPI_0_REGS, ORBIS_SPI_CHANNEL);

    // Disable the channel
    McSPIChannelDisable(SOC_SPI_0_REGS, ORBIS_SPI_CHANNEL);

    // Workaround: read the final byte
    while (MCSPI_CH1STAT_EOT != (MCSPI_CH1STAT_EOT & McSPIChannelStatusGet(SOC_SPI_0_REGS, 0)));
    orbisDataRx[orbisDataRxLength - 1] = McSPIReceiveData(SOC_SPI_0_REGS, ORBIS_SPI_CHANNEL) & ORBIS_BIT_MASK;

    /*
    // Fill the Tx register & clear its interrupt signal
    if (MCSPI_INT_TX_EMPTY(ORBIS_SPI_CHANNEL) == (intCode & MCSPI_INT_TX_EMPTY(ORBIS_SPI_CHANNEL))) {
        McSPITransmitData(SOC_SPI_0_REGS, 0, ORBIS_SPI_CHANNEL);
        McSPIIntStatusClear(SOC_SPI_0_REGS, MCSPI_INT_TX_EMPTY(ORBIS_SPI_CHANNEL));
    }

    MCSPI_CH_STAT_EOT == McSPIChannelStatusGet(SOC_SPI_0_REGS, 0);

    // Read the response
    for (uint32_t i = 0; i < orbisDataRxLength; i++) {
        // Wait until the receiver register is ready
        while (!(MCSPI_INT_RX_FULL(ORBIS_SPI_CHANNEL) & McSPIIntStatusGet(SOC_SPI_0_REGS)));
        // Read the value from the register and store in main memory
        orbisDataRx[i] = McSPIReceiveData(SOC_SPI_0_REGS, ORBIS_SPI_CHANNEL) & ORBIS_BIT_MASK;
        // Clear the interrupt signal
        McSPIIntStatusClear(SOC_SPI_0_REGS, MCSPI_INT_RX_FULL(ORBIS_SPI_CHANNEL));
    }
    */

    //McSPIChannelDisable(SOC_SPI_0_REGS, ORBIS_SPI_CHANNEL);


    // Validate CRC and report back
    return OrbisValidateCRC();
}

//
// Calculate and store CRC for received Orbis data packet.
//
// Sets global variables orbisReceivedCRC, orbisCalculatedCRC, and orbisCRCErrorFlag.
// Returns ORBIS_CRC_OK if validation is successful, ORBIS_CRC_FAIL otherwise.
//
uint8_t OrbisValidateCRC(void)
{
    uint8_t r;

    orbisReceivedCRC = (uint8_t) ~orbisDataRx[orbisDataRxLength - 1];
    orbisCalculatedCRC = OrbisCRC_Buffer(orbisDataRx, orbisDataRxLength - 1);

    r = (orbisReceivedCRC == orbisCalculatedCRC) ? ORBIS_CRC_OK : ORBIS_CRC_FAIL;

    // The CRC failure flag is sticky
    if (ORBIS_CRC_FAIL == r)
        orbisCRCErrorFlag = ORBIS_CRC_FAIL;

    return r;
}


//
// Calculate CRC from fixed length buffer with 0x97 polynome.
// Adapted from the Appendix 1 of the Orbis data sheet.
// Input: pointer to the buffer, and how many bytes from
// the buffer to use to calculate CRC
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
