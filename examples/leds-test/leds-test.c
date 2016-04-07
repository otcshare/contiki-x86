/*
 * Copyright (C) 2016, Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdio.h>

#include "contiki.h"
#include "sys/clock.h"
#include "sys/ctimer.h"
#include "dev/leds.h"

static struct ctimer timer;

/*---------------------------------------------------------------------------*/
PROCESS(leds_process, "LEDs-test process");
AUTOSTART_PROCESSES(&leds_process);
/*---------------------------------------------------------------------------*/
static void timeout(void *data)
{
  printf("Toggling LEDs\n");

  leds_toggle(LEDS_GREEN);
  leds_toggle(LEDS_YELLOW);

  ctimer_reset(&timer);
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(leds_process, ev, data)
{
  PROCESS_BEGIN();

  printf("Start LEDs sample application\n");

  /* Turn all LEDs on */
  leds_set(LEDS_ALL);
  clock_wait(CLOCK_SECOND);

  /* Turn all LEDs off */
  leds_set(~LEDS_ALL);
  clock_wait(CLOCK_SECOND);

  leds_on(LEDS_GREEN);
  clock_wait(CLOCK_SECOND);
  leds_off(LEDS_GREEN);
  clock_wait(CLOCK_SECOND);

  leds_on(LEDS_YELLOW);
  clock_wait(CLOCK_SECOND);
  leds_off(LEDS_YELLOW);
  clock_wait(CLOCK_SECOND);

  leds_blink();

  /* Check if both LEDs are turned off */
  if (leds_get() & LEDS_GREEN) {
    printf("Error: Green LED is on!\n");
  }
  if (leds_get() & LEDS_YELLOW) {
    printf("Error: Yellow LED is on!\n");
  }

  ctimer_set(&timer, CLOCK_SECOND * 2, timeout, NULL);
  PROCESS_YIELD();

  PROCESS_END();
}
