
#include <stdint.h>
#ifndef __IAR_COMPAT_H__
#define __IAR_COMPAT_H__

uint8_t __save_interrupt(void);
void __disable_interrupt(void);
void __restore_interrupt(uint8_t sreg_state);
void __enable_interrupt(void);
void __sleep(void);
#endif