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
; Definitions for the W65C22 Versatile Interface Adapter (VIA).
;

VIA_REG_ORB	equ	0	; Output Register B
VIA_REG_IRB	equ	0	; Input Register B
VIA_REG_ORA	equ	1	; Output Register A
VIA_REG_IRA	equ	1	; Input Register A
VIA_REG_DDRB	equ	2	; Data Direction Register B
VIA_REG_DDRA	equ	3	; Data Direction Register A
VIA_REG_T1C_L	equ	4	; Timer 1 low-order latches/counter
VIA_REG_T1C_H	equ	5	; Timer 1 high-order counter
VIA_REG_T1L_L	equ	6	; Timer 1 low-order latches
VIA_REG_T1L_H	equ	7	; Timer 1 high-order latches
VIA_REG_T2C_L	equ	8	; Timer 2 low-order latches/counter
VIA_REG_T2C_H	equ	9	; Timer 2 high-order counter
VIA_REG_SR	equ	10	; Shift Register
VIA_REG_ACR	equ	11	; Auxliliary Control Register
VIA_REG_PCR	equ	12	; Peripheral Control Register
VIA_REG_IFR	equ	13	; Interrupt Flag Register
VIA_REG_IER	equ	14	; Interrupt Enable Register
VIA_REG_ORA_NH	equ	15	; ORA "no handshake"
VIA_REG_IRA_NH	equ	15	; IRA "no handshake"

; Data Direction Register -- 0 -> input, 1 -> output

; Auxiliary Control Register
VIA_ACR_PA_LATCH_EN	equ	$01	; PA latch enable
VIA_ACR_PB_LATCH_EN	equ	$02	; PB latch enable
VIA_ACR_SR_DISABLE	equ	$00	; Shift Register disabled
VIA_ACR_SR_IN_T2	equ	$04	; Shift in under T2 control
VIA_ACR_SR_IN_PHI2	equ	$08	; Shift in under PHI2 control
VIA_ACR_SR_IN_EXT	equ	$0c	; Shift in under external control
VIA_ACR_SR_OUT_T2_FREE	equ	$10	; Shift out free running at T2 rate
VIA_ACR_SR_OUT_T2	equ	$14	; Shift out under T2 control
VIA_ACR_SR_OUT_PHI2	equ	$18	; Shift out under PHI2 control
VIA_ACR_SR_OUT_EXT	equ	$1c	; Shift out under external control
VIA_ACR_T2_CNTDN_PB6	equ	$20	; T2 counts down with pulses on PB6
VIA_ACR_T1_CONTINUOUS	equ	$40	; T1 continuous interrupts
VIA_ACR_T1_PB7_EN	equ	$80	; T1 enable PB7 output

; Peripheral Control Register
VIA_PCR_CA1_NE		equ	$00	; CA1 IRQ neg edge
VIA_PCR_CA1_PE		equ	$01	; CA1 IRQ pos edge
VIA_PCR_CA2_NE		equ	$00	; CA2 IRQ neg edge
VIA_PCR_CA2_NE_IND	equ	$02	; CA2 independent IRQ neg edge
VIA_PCR_CA2_PE		equ	$04	; CA2 IRQ pos edge
VIA_PCR_CA2_PE_IND	equ	$06	; CA2 independent IRQ pos edge
VIA_PCR_CA2_HS_OUT	equ	$08	; CA2 handshake output
VIA_PCR_CA2_PULSE_OUT	equ	$0a	; CA2 pulse output
VIA_PCR_CA2_LOW_OUT	equ	$0c	; CA2 low output
VIA_PCR_CA2_HIGH_OUT	equ	$0e	; CA2 high output
VIA_PCR_CB1_NE		equ	$00	; CB1 IRQ neg edge
VIA_PCR_CB1_PE		equ	$10	; CB1 IRQ pos edge
VIA_PCR_CB2_NE		equ	$00	; CB2 IRQ neg edge
VIA_PCR_CB2_NE_IND	equ	$20	; CB2 independent IRQ neg edge
VIA_PCR_CB2_PE		equ	$40	; CB2 IRQ pos edge
VIA_PCR_CB2_PE_IND	equ	$60	; CB2 independent IRQ pos edge
VIA_PCR_CB2_HS_OUT	equ	$80	; CB2 handshake output
VIA_PCR_CB2_PULSE_OUT	equ	$a0	; CB2 pulse output
VIA_PCR_CB2_LOW_OUT	equ	$c0	; CB2 low output
VIA_PCR_CB2_HIGH_OUT	equ	$e0	; CB2 high output

; Interrupt Flag Register / Interrupt Enable Register
VIA_INT_CA2		equ	$01	; CA2 active
VIA_INT_CA1		equ	$02	; CA1 active
VIA_INT_SR		equ	$04	; Shift register finished byte
VIA_INT_CB2		equ	$08	; CB2 active
VIA_INT_CB1		equ	$10	; CB1 active
VIA_INT_T2		equ	$20	; T2 expired
VIA_INT_T1		equ	$40	; T1 expired
VIA_IFR_ANY		equ	$80	; (IFR) any interrupt is active
VIA_IER_ALL		equ	$80	; (IER) enable/disable all
