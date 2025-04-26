#include <avr/io.h>
#include <avr/interrupt.h>

#include "main.h"
#include "sevseg.h"

ISR(TIMER0_OVF_vect)
{
    uint8_t digits[4];
    uint16_t v = (uint16_t)(temp/10);
    sevseg_bin2bcd(v, digits);
    sevseg_display_process(digits);
}

