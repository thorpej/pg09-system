;
; Copyright (c) 2023 Jason R. Thorpe.
; All rights reserved.
;
; Redistribution and use in source and binary forms, with or without
; modification, are permitted provided that the following conditions
; are met:
; 1. Redistributions of source code must retain the above copyright
;    notice, this list of conditions and the following disclaimer.
; 2. Redistributions in binary form must reproduce the above copyright
;    notice, this list of conditions and the following disclaimer in the
;    documentation and/or other materials provided with the distribution.
;
; THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
; IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
; OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
; IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
; INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
; BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
; AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
; OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
; OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
; SUCH DAMAGE.
;

;
; Memory map and register definitions for the 6809 Playround.
;
; $0000 - $7FFF		Low Banked RAM (32K)
; $8000 - $9DFF		Fixed RAM (7.5K)
; $9E00 - $9FFF		I/O region (0.5K -- 16 x 32 bytes)
; $A000 - $BFFF		High Banked RAM (8K - 512K .. 2MB)
; $C000 - $DFFF		Banked ROM (8K - 128K .. 512K)
; $E000 - $FFFF		Fixed ROM (8K)
;

LBRAM_START		equ	$0000
LBRAM_SIZE		equ	$8000

FRAM_START		equ	$8000
FRAM_SIZE		equ	$1E00

IO_START		equ	$9E00
IOSEL_SIZE		equ	32
IO_SIZE			equ	IOSEL_SIZE*16

IO0_START		equ	(IO_START + (0 * IOSEL_SIZE))
IO1_START		equ	(IO_START + (1 * IOSEL_SIZE))
IO2_START		equ	(IO_START + (2 * IOSEL_SIZE))
IO3_START		equ	(IO_START + (3 * IOSEL_SIZE))
IO4_START		equ	(IO_START + (4 * IOSEL_SIZE))
IO5_START		equ	(IO_START + (5 * IOSEL_SIZE))
IO6_START		equ	(IO_START + (6 * IOSEL_SIZE))
IO7_START		equ	(IO_START + (7 * IOSEL_SIZE))
IO8_START		equ	(IO_START + (8 * IOSEL_SIZE))
IO9_START		equ	(IO_START + (9 * IOSEL_SIZE))
IO10_START		equ	(IO_START + (10 * IOSEL_SIZE))
IO11_START		equ	(IO_START + (11 * IOSEL_SIZE))
IO12_START		equ	(IO_START + (12 * IOSEL_SIZE))
IO13_START		equ	(IO_START + (13 * IOSEL_SIZE))
IO14_START		equ	(IO_START + (14 * IOSEL_SIZE))
IO15_START		equ	(IO_START + (15 * IOSEL_SIZE))

HBRAM_START		equ	$A000
HBRAM_SIZE		equ	$2000

BROM_START		equ	$C000
BROM_SIZE		equ	$2000

FROM_START		equ	$E000
FROM_SIZE		equ	$2000

;
; System registers at IO0SEL
;
ROM_BANKER_PIA		equ	IO0_START+0
ROM_BANK_REG		equ	ROM_BANKER_PIA+0	/* PIA */
PSU_CTRL_REG		equ	ROM_BANKER_PIA+2	/* PIB */
RAM_BANKER_PIA		equ	IO0_START+4
LBRAM_BANK_REG		equ	RAM_BANKER_PIA+0	/* PIA */
HBRAM_BANK_REG		equ	RAM_BANKER_PIA+2	/* PIB */
IRQ_ENABLE_PIA		equ	IO0_START+8
IRQ_ENABLE_L_REG	equ	IRQ_ENABLE_PIA+0	/* PIA */
IRQ_ENABLE_H_REG	equ	IRQ_ENABLE_PIA+2	/* PIB */
IRQ_VECBASE_REG		equ	IO0_START+12
CLOCK_SPEED_REG		equ	IO0_START+16

;
; PSU_CTRL_REG values
;
PSU_CTRL_POWEROFF	equ	0
PSU_CTRL_RESET		equ	1

;
; UARTs at IO1SEL
;
UART0_BASE		equ	IO1_START+0	; W65C51 ACIA
UART1_BASE		equ	IO1_START+8	; TL16550

;
; VIA for SD card interface at IO2SEL
;
SDVIA_BASE		equ	IO2_START
