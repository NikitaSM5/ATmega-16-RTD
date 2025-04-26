#ifndef SEVSEG_H
#define SEVSEG_H

#include <avr/io.h>

void sevseg_init(void);
void sevseg_bin2bcd(uint16_t number, uint8_t *digits);
void sevseg_display_process(const uint8_t *digits);
void sevseg_off(void);

#endif

/* End File */