/**
 * \file   main.c
 *
 * BeagleBone Black bare metal (StarterWare) and Orbis encoder
 * SPI communications example.
 *
 * Based on the StarterWare GPIO Example (Blinky)
 *
 * TODO The timer over-waits by a factor of x2 to x4, investigate
 *
 * Oliver Frolovs, 2019
 */
/*
* Copyright (C) 2010 Texas Instruments Incorporated - http://www.ti.com/
*/
/*
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

#include <stdint.h>
#include "hw_types.h"
#include "soc_AM335x.h"
#include "beaglebone.h"
#include "gpio_v2.h"
#include "pin_mux.h"
#include "delay.h"
#include "consoleUtils.h"
#include "orbis.h"
#include "util.h"

/*****************************************************************************
**                INTERNAL MACRO DEFINITIONS
*****************************************************************************/
#define GPIO_INSTANCE_ADDRESS           (SOC_GPIO_1_REGS)
#define GPIO_INSTANCE_PIN_NUMBER        (23)

#define LED_DELAY (0x122222)

/*****************************************************************************
**                INTERNAL FUNCTION PROTOTYPES
*****************************************************************************/
static void Delay(unsigned int count);
static void TimerSetup(void);
static void LEDGPIOSetup(void);
static void ConsoleUARTSetup(void);

/*****************************************************************************
**                GLOBAL VARIABLES
*****************************************************************************/

/*****************************************************************************
**                INTERNAL FUNCTION DEFINITIONS
*****************************************************************************/
/*
** The main function. Application starts here.
*/
int main()
{
    GPIO0ModuleClkConfig();
    GPIOModuleEnable(SOC_GPIO_0_REGS);
    GPIOModuleReset(SOC_GPIO_0_REGS);

    ConsoleUARTSetup();
    ConsoleUtilsPrintf("\n\n-==[ BBB_McSPI_Orbis ]==-\n");
    ConsoleUtilsPrintf("Initialising hardware:\n");

    TimerSetup();
    ConsoleUtilsPrintf("\t+ Delay timer...\n");

    OrbisSetup();
    ConsoleUtilsPrintf("\t+ Orbis rotary encoder...\n");

    LEDGPIOSetup();
    ConsoleUtilsPrintf("\t+ LEDs...\n");

    ConsoleUtilsPrintf("Entering the main loop...\n");
    while(1)
    {
        /* Driving a logic HIGH on the GPIO pin. */
        GPIOPinWrite(GPIO_INSTANCE_ADDRESS,
                     GPIO_INSTANCE_PIN_NUMBER,
                     GPIO_PIN_HIGH);

        Delay(LED_DELAY);

        /* Get data from Orbis */
        if (OrbisCaptureGet() != ORBIS_CRC_OK) {
            ConsoleUtilsPrintf("VAL: %x\t\tCRC_RX: %x\t\tCRC_CP: %x\n",
                               (orbisDataRx[0] << 16) | (orbisDataRx[1] << 8) | orbisDataRx[2],
                               (uint8_t) ~orbisDataRx[orbisDataRxLength - 1],
                               OrbisCRC_Buffer(orbisDataRx, orbisDataRxLength - 1));
        }

        /* Driving a logic LOW on the GPIO pin. */
        GPIOPinWrite(GPIO_INSTANCE_ADDRESS,
                     GPIO_INSTANCE_PIN_NUMBER,
                     GPIO_PIN_LOW);

        Delay(LED_DELAY);
    }

}

/*
** A function which is used to generate a delay.
*/
static void Delay(volatile unsigned int count)
{
    while(count--);
}

static void TimerSetup(void)
{
    DMTimer4ModuleClkConfig();
    DMTimerPreScalerClkDisable(SOC_DMTIMER_4_REGS);
    DMTimerCounterSet(SOC_DMTIMER_4_REGS, 0);
    DMTimerReloadSet(SOC_DMTIMER_4_REGS, 0);
    DMTimerModeConfigure(SOC_DMTIMER_4_REGS, DMTIMER_AUTORLD_NOCMP_ENABLE);
    DMTimerEnable(SOC_DMTIMER_4_REGS);
}

static void LEDGPIOSetup(void)
{
    /* Enabling functional clocks for GPIO1 instance. */
    GPIO1ModuleClkConfig();

    /* Selecting GPIO1[23] pin for use. */
    GPIO1Pin23PinMuxSetup();

    /* Enabling the GPIO module. */
    GPIOModuleEnable(GPIO_INSTANCE_ADDRESS);

    /* Resetting the GPIO module. */
    GPIOModuleReset(GPIO_INSTANCE_ADDRESS);

    /* Setting the GPIO pin as an output pin. */
    GPIODirModeSet(GPIO_INSTANCE_ADDRESS,
                   GPIO_INSTANCE_PIN_NUMBER,
                   GPIO_DIR_OUTPUT);
}

static void ConsoleUARTSetup(void)
{
    ConsoleUtilsInit();
    ConsoleUtilsSetType(CONSOLE_UART);
}
/******************************* End of file *********************************/
