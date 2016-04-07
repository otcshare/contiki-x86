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

#ifndef CC2520_H_
#define CC2520_H_

#include "radio.h"
#include "dev/cc2520/cc2520_const.h"
#include "cc2520-arch.h"

#define CC2520_MAX_PACKET_LEN      127

/************************************************************************/
/* Contiki CC2520 API from dev/cc2520/cc2520.h                          */
/************************************************************************/
int cc2520_init(void);
radio_result_t cc2520_set_channel(int channel);
int cc2520_get_channel(void);
void cc2520_set_pan_addr(unsigned pan,
                         unsigned addr,
                         const uint8_t *ieee_addr);
int cc2520_rssi(void);

extern signed char cc2520_last_rssi;
extern uint8_t cc2520_last_correlation;
extern const struct radio_driver cc2520_driver;

void cc2520_set_txpower(uint8_t power);
int cc2520_get_txpower(void);
#define CC2520_TXPOWER_MAX  31
#define CC2520_TXPOWER_MIN   0

int cc2520_interrupt(void);
int cc2520_on(void);
int cc2520_off(void);
void cc2520_set_cca_threshold(int value);

#define CC2520_STROBE(s) cc2520_strobe(s)
#define CC2520_STROBE_PLUS_NOP(s) cc2520_strobe_plus_nop(s)
#define CC2520_WRITE_REG(adr, data) cc2520_write_reg(adr, data)
#define CC2520_READ_REG(adr, data) (data = cc2520_read_reg(adr))
#define CC2520_READ_FIFO_BYTE(data) (data = cc2520_read_fifo_byte())
#define CC2520_READ_FIFO_BUF(buffer, count) cc2520_read_fifo_buf(buffer, count)
#define CC2520_WRITE_FIFO_BUF(buffer, count) cc2520_write_fifo_buf((uint8_t *)buffer, count)
#define CC2520_WRITE_RAM(buffer, adr, count) cc2520_write_ram(buffer, adr, count)
#define CC2520_READ_RAM(buffer, adr, count) cc2520_read_ram(buffer, adr, count)
#define CC2520_GET_STATUS(s) (s = cc2520_get_status())
#define CC2520_ENABLE_FIFOP_INT() cc2520_enable_fifop_int(1)
#define CC2520_DISABLE_FIFOP_INT() cc2520_enable_fifop_int(0)
#define CC2520_FIFOP_INT_INIT() cc2520_init_fifop_int()
#define CC2520_CLEAR_FIFOP_INT() cc2520_clear_fifop_int()
#define CC2520_FIFOP_IS_1 (cc2520_get_fifop() != 0)
#define CC2520_FIFO_IS_1 (cc2520_get_fifo() != 0)
#define CC2520_SFD_IS_1 (cc2520_get_sfd() != 0)
#define CC2520_CCA_IS_1 (cc2520_get_cca() != 0)
#define SET_VREG_ACTIVE() cc2520_set_vreg(1)
#define SET_VREG_INACTIVE() cc2520_set_vreg(0)
#define SET_RESET_ACTIVE() cc2520_set_reset(0)
#define SET_RESET_INACTIVE() cc2520_set_reset(1)

#endif /* CC2520_H_ */
