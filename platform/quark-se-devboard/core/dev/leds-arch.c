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

#include "dev/leds.h"

#include "qm_gpio.h"
#include "qm_pinmux.h"

#define GPIO_PIN_GREEN_LED  25
#define GPIO_PIN_YELLOW_LED 26

static unsigned char leds_state;

void
leds_arch_init(void)
{
  qm_gpio_port_config_t cfg = { 0 };

  leds_state = 0;

  /* Select GPIO function */
  qm_pmux_select(QM_PIN_ID_59, QM_PMUX_FN_0);
  qm_pmux_select(QM_PIN_ID_60, QM_PMUX_FN_0);

  /* Set output direction for LED pins */
  cfg.direction = BIT(GPIO_PIN_GREEN_LED) | BIT(GPIO_PIN_YELLOW_LED);
  qm_gpio_set_config(QM_GPIO_0, &cfg);

  /* Turn both LEDs off */
  qm_gpio_set_pin(QM_GPIO_0, GPIO_PIN_GREEN_LED);
  qm_gpio_set_pin(QM_GPIO_0, GPIO_PIN_YELLOW_LED);
}
/*---------------------------------------------------------------------------*/
unsigned char
leds_arch_get(void)
{
  return leds_state;
}
/*---------------------------------------------------------------------------*/
void
leds_arch_set(unsigned char leds)
{
  leds_state = leds;

  if(leds & LEDS_GREEN) {
    qm_gpio_clear_pin(QM_GPIO_0, GPIO_PIN_GREEN_LED);
  } else {
    qm_gpio_set_pin(QM_GPIO_0, GPIO_PIN_GREEN_LED);
  }

  if(leds & LEDS_YELLOW) {
    qm_gpio_clear_pin(QM_GPIO_0, GPIO_PIN_YELLOW_LED);
  } else {
    qm_gpio_set_pin(QM_GPIO_0, GPIO_PIN_YELLOW_LED);
  }
}
