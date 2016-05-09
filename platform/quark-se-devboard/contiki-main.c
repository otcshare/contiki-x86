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
#include "dev/watchdog.h"
#include "dev/leds.h"
#include "dev/button-sensor.h"
#include "cc2520.h"

#include "net/netstack.h"
#include "net/mac/frame802154.h"
#include "net/queuebuf.h"

#include "net/ipv6/uip-ds6.h"

SENSORS(&button_sensor, &button_sensor2);

/*
 * FIXME(jeez): move all network related code out of this .c file and protect it
 * with a -DQUARK_SE_DEVBOARD_NETWORK_ENABLED of some sort.
 */
static void
set_node_addr(void)
{
  linkaddr_t n_addr;
  int i;

  memset(&n_addr, 0, sizeof(linkaddr_t));

  n_addr.u8[7] = NODE_ID & 0xff;
  n_addr.u8[6] = NODE_ID >> 8;

  linkaddr_set_node_addr(&n_addr);
  printf("Network started with address ");
  for(i = 0; i < sizeof(n_addr.u8) - 1; i++) {
    printf("%d.", n_addr.u8[i]);
  }
  printf("%d\n", n_addr.u8[i]);
}
/*---------------------------------------------------------------------------*/
int
main(void)
{
  clock_init();
  rtimer_init();
  watchdog_init();
  leds_init();
  process_init();

  process_start(&sensors_process, NULL);
  process_start(&etimer_process, NULL);
  ctimer_init();

  watchdog_start();

  /* Networking Init Code.
   *
   * FIXME(jeez): move this out of this .c file and protect it with a
   * -DQUARK_SE_DEVBOARD_NETWORK_ENABLED of some sort.
   */
  set_node_addr();

  cc2520_init();
  {
    uint8_t longaddr[8];
    uint16_t shortaddr;

    shortaddr = (linkaddr_node_addr.u8[0] << 8) +
      linkaddr_node_addr.u8[1];
    memset(longaddr, 0, sizeof(longaddr));
    linkaddr_copy((linkaddr_t *)&longaddr, &linkaddr_node_addr);

    printf("MAC %x:%x:%x:%x:%x:%x:%x:%x ",
           longaddr[0], longaddr[1], longaddr[2], longaddr[3],
           longaddr[4], longaddr[5], longaddr[6], longaddr[7]);

    cc2520_set_pan_addr(IEEE802154_PANID, shortaddr, longaddr);
  }
  cc2520_set_channel(RF_CHANNEL);

  /* memcpy(&uip_lladdr.addr, ds2411_id, sizeof(uip_lladdr.addr)); */
  memcpy(&uip_lladdr.addr, linkaddr_node_addr.u8,
         UIP_LLADDR_LEN > LINKADDR_SIZE ? LINKADDR_SIZE : UIP_LLADDR_LEN);

  queuebuf_init();
  NETSTACK_RDC.init();
  NETSTACK_MAC.init();
  NETSTACK_NETWORK.init();

  printf("%s %s, channel check rate %lu Hz, radio channel %u\n",
         NETSTACK_MAC.name, NETSTACK_RDC.name,
         CLOCK_SECOND / (unsigned long)(NETSTACK_RDC.channel_check_interval() == 0 ? 1 :
                                        NETSTACK_RDC.channel_check_interval()),
         RF_CHANNEL);

  process_start(&tcpip_process, NULL);

  printf("Tentative link-local IPv6 address ");
  {
    uip_ds6_addr_t *lladdr;
    int i;
    lladdr = uip_ds6_get_link_local(-1);
    for(i = 0; i < 7; ++i) {
      printf("%x%x:", lladdr->ipaddr.u8[i * 2],
             lladdr->ipaddr.u8[i * 2 + 1]);
    }
    printf("%x%x\n", lladdr->ipaddr.u8[14], lladdr->ipaddr.u8[15]);
  }

  autostart_start(autostart_processes);

  while(1) {
    watchdog_periodic();

    process_run();
  }

  return 0;
}
