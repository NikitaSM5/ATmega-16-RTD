#include "sevseg.h"
#include "max31865.h"        
#include "main.h"
#include "twi.h"
#include "pcf8583.h"
#include <util/delay.h>
#include <util/atomic.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <lcd16x2.h>

#define F_CPU 8000000UL
#define BTN_MASK (1<<PC3)
#define LED_MASK (1<<PD7)


struct pcf_time t;
static uint8_t lcd_buf[3 * sizeof(t) - 1] = {0};
volatile uint8_t disp_digits[4];
static inline uint8_t bcd2dec(uint8_t v) { return ((v >> 4) * 10) + (v & 0x0F); }
	
	
static inline void time2str(struct pcf_time *time, uint8_t *buf)
{
    *buf++ = (time->hours    >> 4) + '0';
    *buf++ = (time->hours    & 0x0F) + '0';
    *buf++ = ':';
    *buf++ = (time->minutes  >> 4) + '0';
    *buf++ = (time->minutes  & 0x0F) + '0';
    *buf++ = ':';
    *buf++ = (time->seconds  >> 4) + '0';
    *buf++ = (time->seconds  & 0x0F) + '0';
}


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

static void minute_led_task(void)
{
    static uint8_t prev_min = 0xFF;
    uint8_t cur_min = t.minutes;
    if (cur_min != prev_min) {
        //PORTD ^= LED_MASK; 
        prev_min = cur_min;
    }
}

static void print_time(void){
		 pcf_read_time(&t);                    
		 while (twi_status != TWI_STATUS_RX_COMPLETE);
		 twi_status = TWI_STATUS_READY;

		 time2str(&t, lcd_buf);
		 lcd_mov_cursor(6);                   
		 lcd_disp_buf(lcd_buf, 8);
}


int main(void)
{
	sevseg_init();
	twi_init();
	sevseg_blank_leading(1); 
	DDRD = 0xFF;
	lcd_init();             
	max31865_init(0, 0);
	
           
	lcd_disp_str((uint8_t *)"Time:");     

	DDRB |= (1 << PB4);
	TCCR0 = (1 << CS01) | (1 << CS00);
	TIMSK |= (1 << TOIE0);
	
	DDRC &= ~BTN_MASK;
	PORTC |= BTN_MASK;
	sei();
	
	while (1) {
		print_time();
		if (!(PINC & BTN_MASK)) {
			time_to_digits_mmss();
			} else {
			float temp_c = max31865_read_temperature();
			temperature_to_digits_tenth(temp_c);
		}
		
		_delay_ms(500);
		minute_led_task();
		
	}
}

ISR(TIMER0_OVF_vect)
{
	sevseg_display_process(disp_digits);      
}
