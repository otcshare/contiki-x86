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

#ifndef CC2520_ARCH_H_
#define CC2520_ARCH_H_

#include <stdint.h>

/* FIXME: Understand and implement these defines correctly. */
#define splhigh() 0
#define splx(s) (s++)
#define CC2520_CONF_SYMBOL_LOOP_COUNT 10

void cc2520_arch_init();

void cc2520_enable_fifop_int(int enable);
void cc2520_init_fifop_int(void);
void cc2520_clear_fifop(void);
void cc2520_clear_fifop_int(void);
uint8_t cc2520_get_fifop();
uint8_t cc2520_get_fifo();
uint8_t cc2520_get_sfd();
uint8_t cc2520_get_cca();
void cc2520_set_vreg(int enable);
void cc2520_set_reset(int enable);
void cc2520_strobe(uint8_t);
void cc2520_strobe_plus_nop(uint8_t);
void cc2520_write_reg(uint16_t adr, uint8_t data);
uint8_t cc2520_read_reg(uint16_t adr);
uint8_t cc2520_read_fifo_byte(void);
uint8_t cc2520_get_status(void);
void cc2520_read_fifo_buf(uint8_t *buffer, int count);
void cc2520_write_fifo_buf(uint8_t *buffer, int count);
void cc2520_write_ram(uint8_t *buffer, uint16_t adr, int count);
void cc2520_read_ram(uint8_t *buffer, uint16_t adr, int count);

#endif /* CC2520_ARCH_H_ */
