/*
 * Copyright (c) 2011, Swedish Institute of Computer Science
 * Copyright (c) 2015 Intel Corporation.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <string.h>

#include "cc2520-arch.h"
#include "cc2520.h"

#include "qm_gpio.h"
#include "qm_interrupt.h"
#include "qm_spi.h"
#include "qm_pinmux.h"
#include "qm_scss.h"

#ifdef CC2520_CONF_SFD_TIMESTAMPS
#define CONF_SFD_TIMESTAMPS CC2520_CONF_SFD_TIMESTAMPS
#endif /* CC2520_CONF_SFD_TIMESTAMPS */

#ifndef CONF_SFD_TIMESTAMPS
#define CONF_SFD_TIMESTAMPS 0
#endif /* CONF_SFD_TIMESTAMPS */

/* These are on AON GPIO pins */
#define GPIO_VREG_EN 0
#define GPIO_RESET_N 1
/* These are on regular GPIO pins */
#define GPIO_CS          11
#define GPIO_FIFO        4
#define GPIO_FIFOP       5
#define GPIO_CCA         6
#define GPIO_SFD         29
#define GPIO_CS_PAD      45
#define GPIO_FIFO_PAD    4
#define GPIO_FIFOP_PAD   5
#define GPIO_CCA_PAD     6
#define GPIO_SFD_PAD     33

static void
fifop_callback(uint32_t status)
{
  /* call to the driver */
  cc2520_interrupt();
}
/*---------------------------------------------------------------------------*/
static void
spi_transfer(uint8_t *data_out, uint8_t *data_in, int len)
{
  qm_spi_transfer_t transfer;
  transfer.tx = data_out;
  transfer.rx = data_in;
  transfer.rx_len = len;
  transfer.tx_len = len;

  qm_gpio_clear_pin(QM_GPIO_0, GPIO_CS);
  qm_spi_transfer(QM_SPI_MST_1, &transfer);
  qm_gpio_set_pin(QM_GPIO_0, GPIO_CS);
}
/*---------------------------------------------------------------------------*/
void
cc2520_enable_fifop_int(int enable)
{
  qm_gpio_port_config_t gpintr_cfg;
  qm_gpio_get_config(QM_GPIO_0, &gpintr_cfg);

  gpintr_cfg.direction &= ~BIT(GPIO_FIFOP);
  gpintr_cfg.int_type |= BIT(GPIO_FIFOP);
  gpintr_cfg.int_polarity |= BIT(GPIO_FIFOP);
  gpintr_cfg.int_debounce &= ~BIT(GPIO_FIFOP);
  gpintr_cfg.callback = fifop_callback;

  if(enable) {
    gpintr_cfg.int_en |= BIT(GPIO_FIFOP);
  } else {
    gpintr_cfg.int_en &= ~BIT(GPIO_FIFOP);
  }

  qm_gpio_set_config(QM_GPIO_0, &gpintr_cfg);

  qm_irq_request(QM_IRQ_GPIO_0, qm_gpio_isr_0);
}
void
cc2520_init_fifop_int(void)
{
  /* nothing ? */
}
void
cc2520_clear_fifop(void)
{
  /* nothing ? */
}
void
cc2520_clear_fifop_int(void)
{
  /* printf("*** Should clear FIFOP \n"); */
}
uint8_t
cc2520_get_fifop()
{
  return (uint8_t)qm_gpio_read_pin(QM_GPIO_0, GPIO_FIFOP);
}
uint8_t
cc2520_get_fifo()
{
  return (uint8_t)qm_gpio_read_pin(QM_GPIO_0, GPIO_FIFO);
}
uint8_t
cc2520_get_sfd()
{
  return (uint8_t)qm_gpio_read_pin(QM_GPIO_0, GPIO_SFD);
}
uint8_t
cc2520_get_cca()
{
  return (uint8_t)qm_gpio_read_pin(QM_GPIO_0, GPIO_CCA);
}
void
cc2520_set_vreg(int enable)
{
  if(enable) {
    qm_gpio_set_pin(QM_AON_GPIO_0, GPIO_VREG_EN);
  } else {
    qm_gpio_clear_pin(QM_AON_GPIO_0, GPIO_VREG_EN);
  }
}
void
cc2520_set_reset(int enable)
{
  if(enable) {
    qm_gpio_set_pin(QM_AON_GPIO_0, GPIO_RESET_N);
  } else {
    qm_gpio_clear_pin(QM_AON_GPIO_0, GPIO_RESET_N);
  }
}
void
cc2520_strobe(uint8_t strobe)
{
  uint8_t data_in = 0;
  spi_transfer(&strobe, &data_in, 1);
}
void
cc2520_strobe_plus_nop(uint8_t strobe)
{
  uint8_t data_out[2];
  uint8_t data_in[2] = { 0, 0 };
  data_out[0] = strobe;
  data_out[1] = CC2520_INS_SNOP;
  spi_transfer(data_out, data_in, 2);
}
void
cc2520_write_reg(uint16_t adr, uint8_t data)
{
  uint8_t data_in[3] = { 0, 0, 0 };
  uint8_t data_out[3];
  data_out[0] = (CC2520_INS_MEMWR | ((adr >> 8) & 0x0f)) & 0xff;
  data_out[1] = adr & 0xff;
  data_out[2] = data & 0xff;

  spi_transfer(data_out, data_in, 3);
}
uint8_t
cc2520_read_reg(uint16_t adr)
{
  uint8_t data_out[3];
  uint8_t data_in[3] = { 0, 0, 0 };

  data_out[0] = (CC2520_INS_MEMRD | ((adr >> 8) & 0x0f)) & 0xff;
  data_out[1] = adr & 0xff;
  data_out[2] = 0;

  spi_transfer(data_out, data_in, 3);
  /* assume success and - TODO: check this!! */
  return data_in[2];
}
uint8_t
cc2520_read_fifo_byte(void)
{
  uint8_t data_out[2];
  uint8_t data_in[2] = { 0, 0 };
  data_out[0] = CC2520_INS_RXBUF;
  data_out[1] = 0; /* for the read */
  spi_transfer(data_out, data_in, 2);
  /* assume success and - TODO: check this!! */
  return data_in[1];
}
void
cc2520_read_fifo_buf(uint8_t *buffer, int count)
{
  /* FIXME(jeez): Improve this function by removing
   * data_in and using buffer instead, using count
   * to allocate data_out, use count instead of
   * sizeof() and check if we need the extra byte for
   * the transfer as they say. Most likely we don't.
   */

  /* this is a bit of an over-allocation... */
  uint8_t data_out[128 + 1];
  uint8_t data_in[128 + 1];
  memset(data_out, 0xff, sizeof(data_out));
  data_out[0] = CC2520_INS_RXBUF;
  /* do a read with one more than count since we know that the first read will
     not give the payload and needs to be ignored */
  spi_transfer(data_out, data_in, count + 1);
  /* assume success and - TODO: check this!! */
  /* copy the data into the buffer */
  memcpy(buffer, &data_in[1], count);
}
void
cc2520_write_fifo_buf(uint8_t *buffer, int count)
{
  /* FIXME(jeez): Improve this function by removing
   * data_in and using buffer instead, using count
   * to allocate data_out, use count instead of
   * sizeof() and check if we need the extra byte for
   * the transfer as they say. Most likely we don't.
   */

  uint8_t data_out[128 + 1];
  uint8_t data_in[128 + 1];
  data_out[0] = CC2520_INS_TXBUF;
  memcpy(&data_out[1], buffer, count);
  /* write the command + buffer */
  spi_transfer(data_out, data_in, count + 1);
}
void
cc2520_write_ram(uint8_t *buffer, uint16_t adr, int count)
{
  /* FIXME(jeez): Improve this function by removing
   * data_in and using buffer instead, using count
   * to allocate data_out, use count instead of
   * sizeof() and check if we need the extra byte for
   * the transfer as they say. Most likely we don't.
   */

  /* assume that 128 bytes is ok */
  uint8_t data_out[128 + 2];
  uint8_t data_in[128 + 2];
  data_out[0] = CC2520_INS_MEMWR | (((adr) >> 8) & 0xFF);
  data_out[1] = ((adr) & 0xFF);
  memcpy(&data_out[2], buffer, count);
  /* write the command + buffer */
  spi_transfer(data_out, data_in, count + 2);
}
void
cc2520_read_ram(uint8_t *buffer, uint16_t adr, int count)
{
  /* FIXME(jeez): Improve this function by removing
   * data_in and using buffer instead, using count
   * to allocate data_out, use count instead of
   * sizeof() and check if we need the extra byte for
   * the transfer as they say. Most likely we don't.
   */

  /* assume that 128 bytes is ok */
  uint8_t data_out[128 + 2] = { 0, };
  uint8_t data_in[128 + 2];
  data_out[0] = CC2520_INS_MEMRD | (((adr) >> 8) & 0xFF);
  data_out[1] = ((adr) & 0xFF);
  /* write the command + buffer */
  spi_transfer(data_out, data_in, count + 2);
  memcpy(buffer, &data_in[2], count);
}
uint8_t
cc2520_get_status()
{
  uint8_t data_out = CC2520_INS_SNOP;
  uint8_t data_in;
  spi_transfer(&data_out, &data_in, 1);
  /* assume success and - TODO: check this!! */
  return data_in;
}
/*---------------------------------------------------------------------------*/
void
cc2520_arch_init(void)
{
  /* Configure FIFOP and CCA pins for GPIO */
  qm_pmux_select(GPIO_FIFOP_PAD, QM_PMUX_FN_0);
  qm_pmux_select(GPIO_FIFO_PAD, QM_PMUX_FN_0);
  qm_pmux_select(GPIO_CCA_PAD, QM_PMUX_FN_0);
  qm_pmux_select(GPIO_SFD_PAD, QM_PMUX_FN_1);
  qm_pmux_select(QM_PIN_ID_42, QM_PMUX_FN_1);
  qm_pmux_select(QM_PIN_ID_43, QM_PMUX_FN_1);
  qm_pmux_select(QM_PIN_ID_44, QM_PMUX_FN_1);
  qm_pmux_select(QM_PIN_ID_45, QM_PMUX_FN_0);

  qm_pmux_input_en(QM_PIN_ID_43, true);
  qm_pmux_input_en(GPIO_CCA_PAD, true);
  qm_pmux_input_en(GPIO_SFD_PAD, true);
  qm_pmux_input_en(GPIO_FIFO, true);

  qm_gpio_port_config_t gpin_cfg;
  qm_gpio_get_config(QM_GPIO_0, &gpin_cfg);
  gpin_cfg.direction &= ~(BIT(GPIO_CCA) | BIT(GPIO_SFD) | BIT(GPIO_FIFO));
  gpin_cfg.direction |= BIT(GPIO_CS);
  qm_gpio_set_config(QM_GPIO_0, &gpin_cfg);

  qm_gpio_set_pin(QM_GPIO_0, GPIO_CS);

  /* Configure AON output pins reset and vreg. */
  qm_gpio_port_config_t gpout_cfg;
  qm_gpio_get_config(QM_AON_GPIO_0, &gpout_cfg);
  gpout_cfg.direction |= (BIT(GPIO_RESET_N) | BIT(GPIO_VREG_EN));
  qm_gpio_set_config(QM_AON_GPIO_0, &gpout_cfg);

  clk_periph_enable(CLK_PERIPH_CLK | CLK_PERIPH_SPI_M1_REGISTER);

  qm_spi_config_t spi_cfg;
  spi_cfg.clk_divider = 32;
  spi_cfg.transfer_mode = QM_SPI_TMOD_TX_RX;
  spi_cfg.frame_size = QM_SPI_FRAME_SIZE_8_BIT;
  spi_cfg.bus_mode = QM_SPI_BMODE_0;

  qm_spi_set_config(QM_SPI_MST_1, &spi_cfg);
  qm_spi_slave_select(QM_SPI_MST_1, QM_SPI_SS_0);
}
/*---------------------------------------------------------------------------*/
