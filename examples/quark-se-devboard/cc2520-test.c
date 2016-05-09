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

#include "contiki.h"
#include <stdio.h>
#include <string.h>
#include "cc2520.h"
#ifndef PERIOD
#define PERIOD 60
#endif

/*---------------------------------------------------------------------------*/
PROCESS(test_process, "CC2520 Test process");
AUTOSTART_PROCESSES(&test_process);
/*---------------------------------------------------------------------------*/
static uint8_t
getreg(uint16_t regname)
{
  uint8_t reg;
  CC2520_READ_REG(regname, reg);
  return reg;
}
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(test_process, ev, data)
{
  static int ch = 11;
  static struct etimer periodic;
  int tmp;

  PROCESS_BEGIN();
  PROCESS_PAUSE();
  printf("CC2520 Radio Test started\n");
  ch = 11;
  etimer_set(&periodic, PERIOD);
  while(1) {
    PROCESS_YIELD();
    if(etimer_expired(&periodic)) {
      etimer_reset(&periodic);
      ch++;
      if(ch > 26) {
        ch = 11;
      }
      printf("Radio test --\n");
      tmp = cc2520_get_channel();
      printf(" Channel read: %d (1)\n", tmp);
      cc2520_set_channel(ch);
      printf(" Channel set to %d \n", ch);
      tmp = cc2520_get_channel();
      printf(" Channel read: %d (2)\n", tmp);
      /* Define radio channel (between 11 and 25) */
      printf(" Resulting frq: %d\n", getreg(CC2520_FREQCTRL));
      printf(" RSSI: %d\n", getreg(CC2520_RSSI));
    }
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
