/*
 * Copyright (C) 2015, Intel Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
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

#include "gpio.h"
#include "helpers.h"

/* GPIO Controler Registers */
#define SWPORTA_DR    0x00
#define SWPORTA_DDR   0x04
#define INTEN         0x30
#define INTMASK       0x34
#define INTTYPE_LEVEL 0x38
#define INT_POLARITY  0x3c
#define INTSTATUS     0x40
#define RAW_INTSTATUS 0x44
#define DEBOUNCE      0x48
#define PORTA_EOI     0x4c
#define EXT_PORTA     0x50
#define LS_SYNC       0x60

#define PINS 8

static inline uint32_t
read(uint32_t base_addr, uint32_t offest)
{
  return *(uint32_t*)(base_addr + offest);
}

static inline void
write(uint32_t base_addr, uint32_t offest, uint32_t val)
{
  *(uint32_t*)(base_addr + offest) = val;
}

/* value must be 0x00 or 0x01 */
static void
set_bit(uint32_t base_addr, uint32_t offset, uint8_t bit, uint8_t value)
{
  uint32_t reg = read(base_addr, offset);

  reg &= ~BIT(bit);
  reg |= value << bit;

  write(base_addr, offset, reg);
}

static void
gpio_interrupt_config(struct quarkX1000_gpio_config *config, uint8_t pin, int flags)
{
  /* set as input */
  set_bit(config->pci.mmio, SWPORTA_DDR, pin, 0);

  /* set interrupt enabled */
  set_bit(config->pci.mmio, INTEN, pin, 1);

  /* unmask interrupt */
  set_bit(config->pci.mmio, INTMASK, pin, 0);

  /* set active high/low */
  set_bit(config->pci.mmio, INT_POLARITY, pin, !!(flags & QUARKX1000_GPIO_ACTIVE_HIGH));

  /* set level/edge */
  set_bit(config->pci.mmio, INTTYPE_LEVEL, pin, !!(flags & QUARKX1000_GPIO_EDGE));

  /* set debounce */
  set_bit(config->pci.mmio, DEBOUNCE, pin, !!(flags & QUARKX1000_GPIO_DEBOUNCE));

  /* set clock synchronous */
  set_bit(config->pci.mmio, LS_SYNC, 0, !!(flags & QUARKX1000_GPIO_CLOCK_SYNC));
}

int
quarkX1000_gpio_config(struct quarkX1000_gpio_config *config, uint8_t pin, int flags)
{
  if (((flags & QUARKX1000_GPIO_IN) && (flags & QUARKX1000_GPIO_OUT)) ||
    ((flags & QUARKX1000_GPIO_INT) && (flags & QUARKX1000_GPIO_OUT))) {
  	return -1;
  }

  if (flags & QUARKX1000_GPIO_INT) {
    gpio_interrupt_config(config, pin, flags);
  } else {
    /* set direction */
    set_bit(config->pci.mmio, SWPORTA_DDR, pin, !!(flags & QUARKX1000_GPIO_OUT));

    /* set interrupt disabled */
    set_bit(config->pci.mmio, INTEN, pin, 0);
  }

  return 0;
}

int
quarkX1000_gpio_config_port(struct quarkX1000_gpio_config *config, int flags)
{
  uint8_t i;

  for (i = 0; i < PINS; i++) {
    if (quarkX1000_gpio_config(config, i, flags) < 0) {
      return -1;
    }
  }

  return 0;
}

int
quarkX1000_gpio_read(struct quarkX1000_gpio_config *config, uint8_t pin, uint8_t *value)
{
  uint32_t value32 = read(config->pci.mmio, EXT_PORTA);
  *value = !!(value32 & BIT(pin));

  return 0;
}

int
quarkX1000_gpio_write(struct quarkX1000_gpio_config *config, uint8_t pin, uint8_t value)
{
  set_bit(config->pci.mmio, SWPORTA_DR, pin, !!value);
  return 0;
}

int
quarkX1000_gpio_read_port(struct quarkX1000_gpio_config *config, uint8_t *value)
{
  uint32_t value32 = read(config->pci.mmio, EXT_PORTA);
  *value = value32 & ~0xFFFFFF00;

  return 0;
}

int
quarkX1000_gpio_write_port(struct quarkX1000_gpio_config *config, uint8_t value)
{
  write(config->pci.mmio, SWPORTA_DR, value);
  return 0;
}

void
quarkX1000_gpio_clock_enable(struct quarkX1000_gpio_config *config)
{
  set_bit(config->pci.mmio, LS_SYNC, 0, 1);
}

void
quarkX1000_gpio_clock_disable(struct quarkX1000_gpio_config *config)
{
  set_bit(config->pci.mmio, LS_SYNC, 0, 0);
}

int
quarkX1000_gpio_init(struct quarkX1000_gpio_config *config)
{
  pci_config_addr_t pci_addr;

  pci_addr.raw = 0;
  pci_addr.bus = 0;
  pci_addr.dev = 21;
  pci_addr.func = 2;

  pci_command_enable(pci_addr, PCI_CMD_MEM_ENABLE);

  pci_init(&config->pci, pci_addr, PCI_CONFIG_REG_BAR1);

  quarkX1000_gpio_clock_enable(config);

  /* clear registers */
  write(config->pci.mmio, INTEN, 0);
  write(config->pci.mmio, INTMASK, 0);
  write(config->pci.mmio, PORTA_EOI, 0);

  return 0;
}
