/*
 * util.h
 * Timer-related utilities from David Barton's RTC code.
 *
 * Assumes Timer4 has been initialised.
 *
 *  Created on: 22 Apr 2019
 *      Author: Oliver Frolovs
 */

#ifndef UTIL_H_
#define UTIL_H_

#include <stdint.h>
#include "soc_AM335x.h"
#include "dmtimer.h"

#define TIMER_MASTER_FREQ               (24000000) /* main clock at 24 MHz */
#define TIMER_1US                       (0x18)
#define TIMER_10US                      (0xf0)
#define TIMER_100US                     (0x960)
#define TIMER_1MS                       (0x5DC0)
#define TIMER_OVERFLOW                  (0xFFFFFFFFu)

#define TIME DMTimerCounterGet(SOC_DMTIMER_4_REGS)

// Wait for a certain number of counter ticks
void waitfor(uint32_t duration);

#endif /* UTIL_H_ */
