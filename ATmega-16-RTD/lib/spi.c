/* -----------------------------  spi.c  ------------------------------ */
#include "spi.h"
#include <avr/io.h>

void spi_init(void)
{
	/* MOSI – PB5, MISO – PB6, SCK – PB7 */
	DDRB |= (1<<PB5)|(1<<PB7);  /* outputs */
	DDRB &= ~(1<<PB6);          /* MISO input */

	SPCR = (1<<SPE)|(1<<MSTR)|(1<<CPOL)|(1<<CPHA)|(1<<SPR0); /* fosc/16, mode 1 */
}

uint8_t spi_transfer(uint8_t data)
{
	SPDR = data;
	while(!(SPSR & (1<<SPIF)));
	return SPDR;
}
//