#include "sevseg.h"
#include "max31865.h"        
#include "main.h"

#define F_CPU 8000000UL
#include <util/delay.h>
#include <util/atomic.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/io.h>

volatile uint8_t disp_digits[4];

static void temperature_to_digits_tenth(float t)
{
	uint8_t d[4];

	uint8_t negative = 0;
	if (t < 0) { negative = 1;  t = -t; }

	uint16_t tenths = (uint16_t)(t * 10.0f + 0.5f);
	if (negative && tenths > 999) tenths = 999;
	else          if (tenths > 9999) tenths = 9999;

	sevseg_bin2bcd(tenths, d);             

	if (negative)          d[3] = 0x0A;    
	else if (d[3] == 0)    d[3] = 0;       

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		for (uint8_t i = 0; i < 4; i++) disp_digits[i] = d[i];
	}
	sevseg_set_dot(1);                     /* 12.3 -> 1 2·3 */
}

int main(void)
{
	sevseg_init();
	sevseg_blank_leading(1);                
	max31865_init(0, 0);
	DDRB |= (1 << PB4);
	TCCR0 = (1 << CS01) | (1 << CS00);
	TIMSK |= (1 << TOIE0);
	sei();
	_delay_ms(10000); 
	while (1) {
		float temp_c = max31865_read_temperature();
		temperature_to_digits_tenth(temp_c);
		_delay_ms(1000);                      /* раз в секунду */
	}
}

ISR(TIMER0_OVF_vect)
{
	sevseg_display_process(disp_digits);      /* быстрая мультиплексия */
}
