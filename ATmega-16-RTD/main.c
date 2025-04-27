#include "sevseg.h"
#include "max31865.h"        
#include "main.h"
#include "twi.h"
#include "pcf8583.h"
#include "sdcard.h"
#define F_CPU 8000000UL
#include <util/delay.h>
#include <util/atomic.h>
#include <math.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <lcd16x2.h>


#define BTN_MASK (1<<PC3)
#define LED_MASK (1<<PD7)
#define BTN_FWD   PC5          /* ЂвперЄдї  Ц следующа€ запись */
#define BTN_BWD   PC6          /* Ђназадї   Ц предыдуща€ запись */
#define BTN_FWD_MASK  (1<<BTN_FWD)
#define BTN_BWD_MASK  (1<<BTN_BWD)

static uint32_t meas_no = 0;
struct pcf_time t;
static uint8_t lcd_buf[3 * sizeof(t) - 1] = {0};
volatile uint8_t disp_digits[4];
static inline uint8_t bcd2dec(uint8_t v) { return ((v >> 4) * 10) + (v & 0x0F); }
	
struct user_flags{
	uint8_t btn_fwd:1;
	uint8_t btn_bwd:1;
	} flags;	
	
static inline void time2str(struct pcf_time *time, uint8_t *buf)
{
    uint8_t hrs = time->hours & 0x3F;

    *buf++ = (hrs >> 4) + '0';
    *buf++ = (hrs & 0x0F) + '0';
    *buf++ = ':';
    *buf++ = (time->minutes >> 4) + '0';
    *buf++ = (time->minutes & 0x0F) + '0';
    *buf++ = ':';
    *buf++ = (time->seconds >> 4) + '0';
    *buf++ = (time->seconds & 0x0F) + '0';
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

	char logbuf[32];
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
	
	DDRC  &= ~((1<<BTN_FWD)|(1<<BTN_BWD)); 
	PORTC |=  ((1<<BTN_FWD)|(1<<BTN_BWD)); 
	

	sd_iter_reset(); 
	
	char line[32];
	float temp = max31865_read_temperature();

	uint8_t cnt = 0;
	
	while (1) {
		print_time();
		
		
		/* --- обработка кнопок --------------------------------------- */
		static uint8_t prev_btn = 0xFF;
		uint8_t now = PINC;

		//if (!(now & (1<<BTN_FWD)) && (prev_btn & (1<<BTN_FWD))) { /* нажатие */
		//   if (sd_read_line(+1, logbuf, sizeof(logbuf))==0) {
		//    lcd_mov_cursor(16);           /* начало 2-й строки */
		//    lcd_disp_buf((uint8_t*)logbuf, strlen(logbuf));
		//   }
		//}
		//if (!(now & (1<<BTN_BWD)) && (prev_btn & (1<<BTN_BWD))) {
		//   if (sd_read_line(-1, logbuf, sizeof(logbuf))==0) {
		//    lcd_mov_cursor(16);
		//    lcd_disp_buf((uint8_t*)logbuf, strlen(logbuf));
		//   }
		//}
		prev_btn = now;
		
		
		temp = max31865_read_temperature();
		cnt++;
		
		if(cnt == 10){
		cnt = 0;
		int16_t t10 = (int16_t)(temp*10.0f + (temp>=0 ? 0.5f : -0.5f));

		meas_no++;
		sprintf(line, "%lu %02u:%02u,%+d.%01u\n",
		meas_no,
		bcd2dec(t.hours & 0x3F),       /* <-- ѕ–ј¬»Ћ№Ќќ */
		bcd2dec(t.minutes),
		t10/10,
		(uint16_t)abs(t10)%10);

		  sd_write_line(line);      /* курсор остаЄтс€ в конце => Ђназадї работает */
		}
		
		if (!(PINC & BTN_MASK)) {
			time_to_digits_mmss();
			} 
			else 
			{
			temperature_to_digits_tenth(temp);
		}
		
		_delay_ms(500);
		minute_led_task();
		
	}
}

ISR(TIMER0_OVF_vect)
{
	sevseg_display_process(disp_digits);     
	if((PINC & (1 << BTN_FWD)) == 0) {
		if(flags.btn_fwd == 0){
			flags.btn_fwd = 1;
			 if (sd_read_line(+1, logbuf, sizeof(logbuf))==0) {
				 lcd_mov_cursor(16);           /* начало 2-й строки */
				 lcd_disp_buf((uint8_t*)logbuf, strlen(logbuf));
			 }
		}
	} else flags.btn_fwd = 0;
	
	if((PINC & (1 << BTN_BWD)) == 0) {
		if(flags.btn_bwd == 0){
			flags.btn_bwd = 1;
			 if (sd_read_line(-1, logbuf, sizeof(logbuf))==0) {
				 lcd_mov_cursor(16);
				 lcd_disp_buf((uint8_t*)logbuf, strlen(logbuf));
			 }
		}
	} else flags.btn_bwd = 0;
}
