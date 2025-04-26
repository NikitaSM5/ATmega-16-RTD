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


volatile long temp = 0;

int main(void)
{
	TIMSK |= (1 << TOIE0);
	TCCR0 = (1 << CS01) | (1 << CS00);
	sei();
	sevseg_init();
	max_init_port(); 
	if (!init_max())
	{
		while (1)
		{
			sevseg_off();
		}
	}
	
	
    while (1) 
    {
		 temp = max_get_data('t');
		 _delay_ms(100);
    }
}

