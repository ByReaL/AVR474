;******************************************************************************
;*
;* SmartBattery assembly source file.
;*
;*      This file contains the low-level implementations for the
;*      intrinsics functions. It is written for the IAR Assembler.
;*
;*      Note on IAR calling convention:
;*         Scratch registers:   R0-R3, R16-R23, R30-R31
;*         Preserved registers: R4-R15, R24-R27
;*         Parameter registers: R16-R23 (1-, 2- or 4-byte alignment)
;*         Return registers:    R16-R19
;*
;* Documentation
;*      For comprehensive code documentation, supported compilers, compiler
;*      settings and supported devices see readme.html
;*
;*      Atmel Corporation: http:;www.atmel.com \n
;*      Support email: avr@atmel.com
;*
;* $Revision: 2211 $
;* $Date: 2009-03-12 10:42:14 +0100 (to, 12 mar 2009) $
;*
;* Copyright (c) 2009, Atmel Corporation All rights reserved.
;*
;* Redistribution and use in source and binary forms, with or without
;* modification, are permitted provided that the following conditions are met:
;*
;* 1. Redistributions of source code must retain the above copyright notice,
;* this list of conditions and the following disclaimer.
;*
;* 2. Redistributions in binary form must reproduce the above copyright notice,
;* this list of conditions and the following disclaimer in the documentation
;* and/or other materials provided with the distribution.
;*
;* 3. The name of ATMEL may not be used to endorse or promote products derived
;* from this software without specific prior written permission.
;*
;* THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
;* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
;* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
;* SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
;* INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
;* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
;* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
;* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
;* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
;* THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;******************************************************************************

; ---
; This routine reads a byte from signature row
; R17:R16.
;
; Input:
;     R17:R16.
;
; Returns:
;     R16 - Read byte.
; ---
#include <avr/io.h>
	.global READ_SIGNATUREBYTE
READ_SIGNATUREBYTE:
	LDI	r17, 0
	MOV	ZH, r17                       ; Move the address to the Z pointer
	MOV ZL, r16
	LDI	r17, ((1 << SIGRD) | (1 << SPMEN))   ; Bits for SPMCR register
	OUT	_SFR_IO_ADDR(SPMCSR), r17                          ; Set bits in SPMCR register
	LPM	r16, Z                               ; Load byte from address pointed to by Z.
	RET

; ---
; This routine reads a word from signature row
; R17:R16.
;
; Input:
;     R17:R16.
;
; Returns:
;     R17:R16 - Read word.
; ---
	.global READ_SIGNATUREWORD
READ_SIGNATUREWORD:
	LDI	r17, 0
	MOV	ZH,r17                       ; Move the address to the Z pointer
	mov zl, r16
	LDI	r17, ((1 << SIGRD) | (1 << SPMEN))   ; Bits for SPMCR register
	OUT	_SFR_IO_ADDR(SPMCSR), r17                          ; Set bits in SPMCR register
	LPM	r16, Z+                              ; Load byte from address pointed to by Z., Wait 3 cycles
	RJMP	wait_1                               ; Wait 2 extra cycles
wait_1:
	LDI	r17, ((1 << SIGRD) | (1 << SPMEN))   ; Bits for SPMCR register
	OUT	_SFR_IO_ADDR(SPMCSR), r17                          ; Set bits in SPMCR register
	LPM	r17, Z                               ; Load byte from address pointed to by Z.
	RET


; ---
; CRC-16-Atmel calculation
;
; Polynomial: x^16 + x^15 + x^2 + 1
; (0x8005 / 0xA001 / 0xC002)
;
; Input:
;     R17:R16 - CRC checksum to update
;     R19:R18 - Data to update CRC with
;
; Returns:
;     R17:R16 - The new checksum
; ---
	.global CRC_16_Atmel
;uint16_t CRC_16_Atmel(uint16_t crc, uint16_t data)
;{
;	for (uint8_t i = 0; i < 16; ++i){
;		if (crc & 0x8000) {
;			crc = ((crc << 1)|((data>>15)&0x0001)) ^ 0x8005;
;		} else {
;			crc = ((crc << 1)|((data>>15)&0x0001));
;		}
;		data <<= 1;
;	}
;	return crc;
;}
CRC_16_Atmel:
	LDI     R20, 5          ;Load low byte of poly (0x05)
	LDI     R21, 128        ;Load high byte of poly (ox80)
	LDI     R22, 16         ;Load counter
CRC_16_Atmel_0:
	LSL     R18             ;Leftshift low byte, 0 -> LSB, MSB -> Carry
	ROL     R19             ;Leftrotate high byte, Carry -> LSB, MSB -> Carry
	ROL     R16             ;Leftrotate low byte, Carry -> LSB, MSB -> Carry
	ROL     R17             ;Leftrotate high byte, Carry -> LSB, MSB -> Carry
	BRCC    CRC_16_Atmel_1  ;Branch if Carry is 0 else goto XOR of polynom
	EOR     R16, R20        ;XOR low bytes
	EOR     R17, R21        ;XOR high bytes
CRC_16_Atmel_1:
	DEC     R22             ;Decrease counter
	BRNE    CRC_16_Atmel_0  ;If counter is not 0, continue for-loop
	RET


; ---
; SMBusb PEC calculation
;
; This function calculates the PEC value of one byte, using the
; PEC calculated so far as a starting point.
;
; Polynomial: x^8 + x^2 + x^1 + 1
; (0x07 / 0xE0 / 0x83)
;
; Input:
;     R16 - The current PEC
;     R17 - Value to calculate PEC for
;
; Returns:
;     R16 - The new checksum
; ---
	.global SMBus_PEC
;uint8_t SMB_PEC(uint8_t lastCRC, uint8_t newByte)
;{
;	lastCRC ^= newByte;
;	
;	for(uint8_t i = 0; i < 8; i++){
;		if (lastCRC & 0x80){
;			lastCRC = (lastCRC << 1) ^ SMB_PEC_CRC_POLYNOME;
;		}else{
;			lastCRC <<= 1;
;		}
;	}
;	return lastCRC;
;}
SMBus_PEC:
	EOR     R16, R17      ;XOR last CRC and new byte
	LDI     R18, 7        ;Load poly byte (0x07)
	LDI     R17, 8        ;Load counter
SMBus_PEC_0:
	LSL     R16           ;Leftshift byte, 0 -> LSB, MSB -> Carry
	BRCC    SMBus_PEC_1   ;Branch if Carry is 0 else goto XOR of polynom
	EOR     R16, R18      ;XOR bytes and polynom
SMBus_PEC_1:
	DEC     R17           ;Decrease counter
	BRNE    SMBus_PEC_0   ;If counter is not 0, continue for-loop
	RET

; ---
; HMAC Leftrotate function
;
; This function rotates the double word input a given number
; of times in a barrel roll to the left.
;
; Input:
;     R19:R18:R17:R16 - 32-Bit value to rotate
;     R20             - Steps to rotate
;
; Returns:
;     R19:R18:R17:R16 - 32-Bit rotated value
; ---
	.global LEFTROTATE
LEFTROTATE:
LEFTROTATE_0:
	LSL     R16
	ROL     R17
	ROL     R18
	ROL     R19
	BRCC	LEFTROTATE_1
	ORI	R16,0x01
LEFTROTATE_1:
	DEC     R20
	BRNE    LEFTROTATE_0
	RET

; ---
; HMAC Rightrotate function
;
; This function rotates the double word input a given number
; of times in a barrel roll to the right.
;
; Input:
;     R19:R18:R17:R16 - 32-Bit value to rotate
;     R20             - Steps to rotate
;
; Returns:
;     R19:R18:R17:R16 - 32-Bit rotated value
; ---
	.global RIGHTROTATE
RIGHTROTATE:
RIGHTROTATE_0:
	LSR     R19
	ROR     R18
	ROR     R17
	ROR     R16
	BRCC	RIGHTROTATE_1
	ORI	R19,0x80
RIGHTROTATE_1:
	DEC     R20
	BRNE    RIGHTROTATE_0
	RET


; ---
;  Update the CCITT-CRC16
;
;  Update the CRC for transmitted and received data using the CCITT 16bit
;  algorithm (X^16 + X^12 + X^5 + 1).
;
;  Input:
;     R17:R16 - Currrent CRC value
;     R18     - Next byte that should be included into the crc16
;
; Returns:
;     R17:R16 - Updated crc16
; ---
	.global crc_ccitt_update
crc_ccitt_update:
	EOR	R18, R16
	MOV	R19, R18
	SWAP	R19
	ANDI	R19, 0xF0
	EOR	R18, R19
	MOV	R22, R18
	SWAP	R22
	MOV	r16, r22
	ANDI	R22, 0x0F
	EOR	R22, R17
	MOV	R17, r16
	ANDI	r17, 0x0F
	ANDI	r16, 0xF0
	LSR	r17
	ROR	r16
	EOR     R16, R22
	EOR     R17, R18
	RET

; END OF FILE
