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

#include "dev/button-sensor.h"

#include "qm_gpio.h"
#include "qm_interrupt.h"

#define BUTTON1_PIN 4
#define BUTTON2_PIN 5

static void button_handler(uint32_t status);

static void
init(int pin_num)
{
  qm_gpio_port_config_t cfg;

  qm_gpio_get_config(QM_AON_GPIO_0, &cfg);
  cfg.direction &= ~BIT(pin_num); /* Set as input pin. */
  qm_gpio_set_config(QM_AON_GPIO_0, &cfg);
}
/*---------------------------------------------------------------------------*/
static void
activate(int pin_num)
{
  qm_gpio_port_config_t cfg;

  qm_irq_request(QM_IRQ_AONGPIO_0, qm_aon_gpio_isr_0);

  qm_gpio_get_config(QM_AON_GPIO_0, &cfg);
  cfg.int_en |= BIT(pin_num); /* Enable interrupt. */
  cfg.int_type |= BIT(pin_num); /* Edge trigger. */
  cfg.int_polarity &= ~BIT(pin_num); /* Low. */
  cfg.int_debounce |= BIT(pin_num);  /* Enable debounce. */
  cfg.int_bothedge &= ~BIT(pin_num); /* Disable both edges. */
  cfg.callback = button_handler;
  qm_gpio_set_config(QM_AON_GPIO_0, &cfg);
}
/*---------------------------------------------------------------------------*/
static int
configure(int type, int pin_num)
{
  switch (type) {
  case SENSORS_HW_INIT:
    init(pin_num);
    break;
  case SENSORS_ACTIVE:
    activate(pin_num);
    break;
  case SENSORS_READY:
    /* Nothing to do here. */
    break;
  default:
    return 0;
  }

  return 1;
}
/*---------------------------------------------------------------------------*/
static int
button1_value(int type)
{
  /* Function not implemented. */
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
button1_configure(int type, int value)
{
  return configure(type, BUTTON1_PIN);
}
/*---------------------------------------------------------------------------*/
static int
button1_status(int type)
{
  /* Function not implemented. */
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
button2_value(int type)
{
  /* Function not implemented. */
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
button2_configure(int type, int value)
{
  return configure(type, BUTTON2_PIN);
}
/*---------------------------------------------------------------------------*/
static int
button2_status(int type)
{
  /* Function not implemented. */
  return 0;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(button_sensor, BUTTON_SENSOR, button1_value, button1_configure,
	       button1_status);
SENSORS_SENSOR(button_sensor2, BUTTON_SENSOR, button2_value, button2_configure,
	       button2_status);
/*---------------------------------------------------------------------------*/
static void
button_handler(uint32_t status)
{
  sensors_changed(&button_sensor2);
}
