;
; Copyright (c) 2022 Jason R. Thorpe.
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
; Definitions for the W65C21 Peripheral Interface Adapter (PIA).
;

PIA_REG_PIA	equ	0		; Peripheral Interface A
PIA_REG_DDRA	equ	0		; Data Direction Register A
PIA_REG_CRA	equ	1		; Control Register A
PIA_REG_PIB	equ	2		; Peripheral Interface B
PIA_REG_DDRB	equ	2		; Data Direction Register B
PIA_REG_CRB	equ	3		; Control Register B

; Data Direction Register -- 0 -> input, 1 -> output

; Control Register definitions
; IRQ1 --> CA1/CB1 pins
; IRQ2 --> CA2/CB2 pins
PIA_CRx_Cx1_NE_DIS	equ	(0 << 0) ; IRQ1 negative edge (disabled)
PIA_CRx_Cx1_NE		equ	(1 << 0) ; IRQ1 negative edge (enabled)
PIA_CRx_Cx1_PE_DIS	equ	(2 << 0) ; IRQ1 positive edge (disabled)
PIA_CRx_Cx1_PE		equ	(3 << 0) ; IRQ1 positive edge (enabled)
PIA_CRx_DDR_PI		equ	(1 << 2) ; 0 -> DDR access, 1 -> Peripheral
PIA_CRx_Cx2_NE_DIS	equ	(0 << 3) ; IRQ2 negative edge (disabled)
PIA_CRx_Cx2_NE		equ	(1 << 3) ; IRQ2 negative edge (enabled)
PIA_CRx_Cx2_PE_DIS	equ	(2 << 3) ; IRQ2 positive edge (disabled)
PIA_CRx_Cx2_PE		equ	(3 << 3) ; IRQ2 positive edge (enabled)
PIA_CRx_Cx2_HS_READ	equ	(4 << 3) ; Cx2 handshake on read
PIA_CRx_Cx2_PO		equ	(5 << 3) ; Cx2 pulse output
PIA_CRx_Cx2_MAN_LOW	equ	(6 << 3) ; Cx2 manual output low
PIA_CRx_Cx2_MAN_HIGH	equ	(7 << 3) ; Cx2 manual output high
PIA_CRx_IRQ2		equ	(1 << 6) ; IRQ2 active
PIA_CRx_IRQ1		equ	(1 << 7) ; IRQ1 active
