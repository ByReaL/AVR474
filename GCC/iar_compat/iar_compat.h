
#include <stdint.h>

#if defined(__GNUC__) // GNU Compiler interrupt names

uint8_t __save_interrupt(void);
void __disable_interrupt(void);
void __restore_interrupt(uint8_t sreg_state);
void __enable_interrupt(void);
void __sleep(void);
#endif