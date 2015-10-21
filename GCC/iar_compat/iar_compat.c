#include "iar_compat.h"
#include <avr/interrupt.h>

uint8_t __save_interrupt(void)
{
	return SREG;
}

void __enable_interrupt(void)
{
	sei();
}

void __disable_interrupt(void)
{
	cli();
}

void __restore_interrupt(uint8_t sreg_state)
{
	SREG = sreg_state;
}
