#include "sevseg.h"
#include "max31865.h"          /*  ← вернули прототипы */
#include "main.h"

#define F_CPU 8000000UL
#include <util/delay.h>
#include <util/atomic.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/io.h>

volatile uint8_t disp_digits[4];

/*---------------------------------------------------------------*/
static void temperature_to_digits_tenth(float t)
{
	/* 0.1-градусная точность:  –––99.9…999.9 °C → 0…9999 */
	uint16_t tenths = (uint16_t)(t * 10.0f + 0.5f);
	if (tenths > 9999) tenths = 9999;

	uint8_t d[4];
	sevseg_bin2bcd(tenths, d);                /* ← передаём 16-битное число! */

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		for (uint8_t i = 0; i < 4; i++) disp_digits[i] = d[i];
	}
	sevseg_set_dot(1);                        /* точка между десятками и единицами */
}
/*---------------------------------------------------------------*/

int main(void)
{
	sevseg_init();
	sevseg_blank_leading(1);                  /* гасим ведущие нули */

	max31865_init(0, 0);

	/* PB4/SS в выход, чтобы SPI не ушёл в slave */
	DDRB |= (1 << PB4);

	/* Timer0: F_CPU / 64 ≈ 122 кГц → на 256 тик. = 488 Гц */
	TCCR0 = (1 << CS01) | (1 << CS00);
	TIMSK |= (1 << TOIE0);
	sei();

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
