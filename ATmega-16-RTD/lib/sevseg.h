#ifndef SEVSEG_H
#define SEVSEG_H

#include <avr/io.h>

void sevseg_init(void);
void sevseg_bin2bcd(uint16_t number, uint8_t *digits);
void sevseg_display_process(const volatile uint8_t *digits);
void sevseg_off(void);
void sevseg_set_dot(uint8_t position);
void sevseg_blank_leading(uint8_t on);


#endif

/* End File */