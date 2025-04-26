#include "sevseg.h"
#include "max31865.h"        
#include "main.h"
#include "twi.h"
#include "pcf8583.h"

#define F_CPU 8000000UL
#include <util/delay.h>
#include <util/atomic.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#define BTN_MASK (1<<PC3)


volatile uint8_t disp_digits[4];
static inline uint8_t bcd2dec(uint8_t v) { return ((v >> 4) * 10) + (v & 0x0F); }

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
	sevseg_set_dot(1);                   
}

static void time_to_digits_mmss(void)
{
	struct pcf_time t;
	pcf_read_time(&t);

	while (twi_status != TWI_STATUS_RX_COMPLETE) ;
	twi_status = TWI_STATUS_READY;

	uint8_t sec = bcd2dec(t.seconds);
	uint8_t min = bcd2dec(t.minutes);

	uint8_t d[4];
	d[0] = sec % 10;
	d[1] = sec / 10;
	d[2] = min % 10;
	d[3] = min / 10;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		for (uint8_t i = 0; i < 4; i++) disp_digits[i] = d[i];
	}
	sevseg_set_dot(2);
}

int main(void)
{
	sevseg_init();
	twi_init();
	sevseg_blank_leading(1);                
	max31865_init(0, 0);
	DDRB |= (1 << PB4);
	TCCR0 = (1 << CS01) | (1 << CS00);
	TIMSK |= (1 << TOIE0);
	
	DDRC &= ~BTN_MASK;
	PORTC |= BTN_MASK;
	sei();
	while (1) {
		if (!(PINC & BTN_MASK)) {
			time_to_digits_mmss();
			} else {
			float temp_c = max31865_read_temperature();
			temperature_to_digits_tenth(temp_c);
		}
		_delay_ms(500);
	}
}

ISR(TIMER0_OVF_vect)
{
	sevseg_display_process(disp_digits);      
}
