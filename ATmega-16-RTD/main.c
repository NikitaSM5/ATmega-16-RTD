/*
 * ATmega-16-RTD.c
 *
 * Created: 26.04.2025 14:51:58
 * Author : Никитосик
 */ 
#include "sevseg.h"
#include "main.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#define F_CPU 8000000UL
#include <util/delay.h>



uint16_t temp = 0;

int main(void)
{
	TIMSK |= (1 << TOIE0);
	TCCR0 = (1 << CS01) | (1 << CS00);
	
	sevseg_init();
	max31865_init(0,0);
	sei();
	
	while (1)
	{
		float tc = max31865_read_temperature();
		temp = tc - 7265;
		_delay_ms(1000);
	}
	
	
}

ISR(TIMER0_OVF_vect)
{
	uint8_t digits[4];
	uint16_t v = (uint16_t)(temp);
	sevseg_bin2bcd(v, digits);
	sevseg_display_process(digits);
}

