/*
 * util.c
 * Timer-related utilities from David Barton's RTC code.
 *
 *  Created on: 22 Apr 2019
 *      Author: Oliver Frolovs
 */

#include "util.h"

// Wait for a certain number of counter ticks
void waitfor(uint32_t duration)
{
    uint32_t t0 = TIME;
    while ((TIME - t0) < duration);
}
