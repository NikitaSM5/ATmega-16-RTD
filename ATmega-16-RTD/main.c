// avr_sd_logger_refactored.c � SD?������ ����������� � 7?����. �����������
// ----------------------------------------------------------------------------
// �������� ���� ������������:  *�������* ��������� � ����SPI � ��������
// �������� (SD?�����, MAX31865, LCD) ������ ����������.  ���������� ������
// ���������� �����?�������, � ������� ���� ������������ �� ���������������.
// ----------------------------------------------------------------------------

#include "sevseg.h"
#include "max31865.h"
#include "main.h"
#include "twi.h"
#include "pcf8583.h"
#include "sdcard.h"
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <math.h>
#include <string.h>
#include <lcd16x2.h>


#define F_CPU 8000000UL

/* ----------------------------- GPIO -------------------------------------- */
#define BTN_MASK        (1<<PC3)
#define BTN_FWD         PC5
#define BTN_BWD         PC6
#define BTN_FWD_MASK    (1<<BTN_FWD)
#define BTN_BWD_MASK    (1<<BTN_BWD)
#define LED_MASK        (1<<PD7)

/* ---------------------------- ����� -------------------------------------- */
/* ---------------------------- ����� -------------------------------------- */
struct {
	uint8_t one_sec     :1;
	uint8_t ten_sec     :1;
	uint8_t nav_fwd     :1;    // ������ ������ �� ������������
	uint8_t nav_bwd     :1;    // ������ ������ �� ������������
	uint8_t uart_dump   :1;    // ������ ����� ���� � UART
	uint8_t btn_lock    :1;    // ���������������� �� ����� ������ � SD
	uint8_t history     :1;    // ������ ������������ ������ �������
	uint8_t show_history:1;    // ���� ��������� ������ �������
} volatile flags;

/* --------------------------- ���������� ---------------------------------- */
static uint32_t meas_no  = 0;               // ����� ���������
static uint32_t nav_pos  = 0;               // ������� ������� �� ����� ���������
static struct pcf_time rtc_time;            // ����� �� PCF8583
volatile uint8_t disp_digits[4];            // ����� 7?seg
static uint32_t cnt_uart = 0;
static char linebuf[32];                    // ����� ������� �����
static char linebuf_hist[32];

/* --------------------- ��������������� ������� --------------------------- */
static inline uint8_t bcd2dec(uint8_t v)  { return ((v >> 4) * 10) + (v & 0x0F); }

static inline void time2str(const struct pcf_time *t, char *buf)
{
	sprintf(buf, "%02u:%02u:%02u",
	bcd2dec(t->hours   & 0x3F),
	bcd2dec(t->minutes & 0x7F),
	bcd2dec(t->seconds & 0x7F));
}

/* ---------------------- ����������� ����������� -------------------------- */
static void temperature_to_digits_tenth(float temp)
{
	uint8_t digits[4];
	uint8_t negative = (temp < 0);
	if (negative) temp = -temp;

	uint16_t hundredths = (uint16_t)(temp * 100.0f + 0.5f);
	if (hundredths > 9999) hundredths = 9999;

	sevseg_bin2bcd(hundredths, digits);

	if (negative)      digits[3] = 0x0A;   // �����
	else if (digits[3] == 0) digits[3] = 0;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		for (uint8_t i = 0; i < 4; i++) disp_digits[i] = digits[i];
	}

	sevseg_set_dot(2);
	sevseg_blank_leading(1);
}

/* -------------------------- UART helper ---------------------------------- */
static inline void uart_putc(char c) { while(!(UCSRA & (1<<UDRE))); UDR = c; }
static void uart_puts(const char *s) { while(*s) uart_putc(*s++); }

/* --------------------------- ������ 1 � 1��� ----------------------------- */
ISR(TIMER1_COMPA_vect)
{
	static uint8_t sec_cnt = 0;
	flags.one_sec = 1;
	if (++sec_cnt >= 10) { sec_cnt = 0; flags.ten_sec = 1; }
}

/* ------------------- ������ 0 � ����������� + ������ --------------------- */
ISR(TIMER0_OVF_vect)
{
	sevseg_display_process(disp_digits);

	static uint8_t prev = BTN_FWD_MASK | BTN_BWD_MASK | (1<<PC7) | (1<<PC3);
	uint8_t cur = PINC & (BTN_FWD_MASK | BTN_BWD_MASK | (1<<PC7) | (1<<PC3));
	uint8_t changed = prev ^ cur;
	uint8_t pressed = changed & (~cur);

	if (pressed & BTN_FWD_MASK)  flags.nav_fwd  = 1;
	if (pressed & BTN_BWD_MASK)  flags.nav_bwd  = 1;
	if (pressed & (1<<PC7))      flags.uart_dump = 1;
	if (pressed & (1<<PC3))      flags.history = 1;

	prev = cur;
}

/* --------------------------- Dump log to UART --------------------------- */
static uint32_t last_sent_entry = 0; // ���������� ���������� ��� �������� �������

static void uart_dump_log(void)
{
	uint32_t current_entry = 0;
	sd_iter_reset(); // �������� � ������ �����
	
	while (sd_read_line(+1, linebuf, sizeof linebuf) == 0) {
		current_entry++;
		
		// ���������� ������ ����� ������
		if(current_entry > last_sent_entry) {
			uart_puts(linebuf);
			uart_putc('\n');
		}
	}
	
	// ��������� ������� ��������� ������������ ������
	if(current_entry > last_sent_entry) {
		last_sent_entry = current_entry;
	}
}

static void uart_init(void)
{
	UCSRA |= (1<<U2X);
	UBRRL = 51; //19.2
	UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);
	UCSRB |= (1 << RXCIE) | (1 << RXEN) | (1 << TXEN);
}

/* ------------------------------------------------------------------------ */

static void timer1_init(void)
{
	TCCR1B = (1<<WGM12) | (1<<CS12);      /* CTC, prescaler 256            */
	OCR1A  = 4095;//31249;                       /* 1���                          */
	TIMSK |= (1<<OCIE1A);                 /* ��������� ���������� Compare  */
}

int main(void)
{
	float temperature = 0.0f;

	uart_init();
	sevseg_init();
	DDRD |= (1 << PD2)|(1 << PD3)|(1 << PD4)|(1 << PD5)|(1 << PD6)|(1 << PD7);
	
	lcd_init();
	lcd_disp_str((uint8_t*) "1");
	max31865_init(0, 0);
	lcd_disp_str((uint8_t*) "2");
	lcd_mov_cursor(16);
	sd_flush();
	
	DDRC &= ~(BTN_MASK|BTN_FWD_MASK | BTN_BWD_MASK);  /* �����?������      */
	PORTC |=  (BTN_MASK|BTN_FWD_MASK | BTN_BWD_MASK); /* pull?up           */
	DDRC &= ~(1<<PC7);
	PORTC |= (1 << PC7);
	/* Timer0: ��� ������������ ����������� sevseg ��� ��������������� */
	TCCR0 = (1<<CS01) | (1<<CS00);     /* prescaler 64, ~1.024��� overflow */
	TIMSK |= (1<<TOIE0);
	
	timer1_init();                      /* 1��� ��������� ����            */
	sd_iter_reset();                    /* ������� ������ �������� */
	
	twi_init();
	sei();                              /* ��������� ��������� IRQ         */
	pcf_init();

	meas_no = sd_get_entry_count();


	/* --- ������� ���� -------------------------------------------------- */
	while (1) {
		
		
		if (flags.history && !flags.btn_lock) {
			flags.history = 0;
			flags.show_history = !flags.show_history;
			
			if (flags.show_history) {
				lcd_mov_cursor(16);
				lcd_disp_str((uint8_t *)linebuf_hist);
				lcd_disp_str((uint8_t*)"    "); // ������� ��������
				} else {
				sd_iter_reset();
				if (sd_read_line(+1, linebuf, sizeof linebuf) == 0) {
					// �������� ��������� ������� ������
					char *endl = strchr(linebuf, '\n');
					if(endl) *endl = 0;
					
					lcd_mov_cursor(16);
					lcd_disp_str((uint8_t *)linebuf);
					lcd_disp_str((uint8_t*)"    "); // ������� ������
				}
			}
		}
		
		/* --------- ������������ ������ -------------------------------- */
		if (flags.one_sec) {
			flags.one_sec = 0;

			pcf_read_time(&rtc_time);
			while (twi_status != TWI_STATUS_RX_COMPLETE);
			twi_status = TWI_STATUS_READY;

			// ���������� ����������� (X.Y)
			temperature = max31865_read_temperature();
			temperature_to_digits_tenth(temperature);

			/* ���������� ������ ������ LCD (����� + ���?�� �������) */
			char tstr[10], cnt[6];
			time2str(&rtc_time, tstr);
			sprintf(cnt, "%lu", meas_no );

			lcd_mov_cursor(0);
			lcd_disp_str((uint8_t *)tstr);
			lcd_mov_cursor(10);
			lcd_disp_str((uint8_t *)"#");
			lcd_disp_str((uint8_t *)cnt);
		}

		/* ---------- ������ 10 � � ��������� + ��� ---------------------- */
		if (flags.ten_sec) {
			flags.ten_sec = 0;
			if (!flags.btn_lock) {
				flags.btn_lock = 1;

				int16_t t100 = (int16_t)(temperature * 100.0f + 0.5f);
				++meas_no;
				sprintf(linebuf, "%lu %02u:%02u,%+d.%02u", // ������ \n � �����
				meas_no,
				bcd2dec(rtc_time.hours & 0x3F),
				bcd2dec(rtc_time.minutes),
				t100/100,
				abs(t100)%100);
				
				// �������� � �������� ������
				memset(linebuf_hist, 0, sizeof(linebuf_hist)); // ������� �����
				strncpy(linebuf_hist, linebuf, sizeof(linebuf_hist)-1); // ���������� �����������
				
				strcat(linebuf, "\n"); // ��������� ������� ������ ��� SD-�����
				sd_write_line(linebuf);
				sd_flush();

				if (flags.show_history) {
					lcd_mov_cursor(16);
					lcd_disp_str((uint8_t *)linebuf_hist);
					lcd_disp_str((uint8_t*)"    "); // ������� ������
				}

				flags.btn_lock = 0;
			}
		}

		/* --------------- ��������� �� ���� ----------------------------- */
		if (!flags.show_history) {
			if (flags.nav_fwd && !flags.btn_lock) {
				flags.nav_fwd = 0;
				if (nav_pos < meas_no) {
					++nav_pos;
					if (sd_read_line(+1, linebuf, sizeof linebuf) == 0) {
						char *endl = strchr(linebuf, '\n');
						if(endl) *endl = 0;
						
						lcd_mov_cursor(16);
						lcd_disp_str((uint8_t *)linebuf);
						lcd_disp_str((uint8_t*)"    ");
					}
				}
			}

			if (flags.nav_bwd && !flags.btn_lock) {
				flags.nav_bwd = 0;
				sd_iter_reset();
				if (sd_read_line(+1, linebuf, sizeof linebuf) == 0) {
					char *endl = strchr(linebuf, '\n');
					if(endl) *endl = 0;
					nav_pos = 0;
					lcd_mov_cursor(16);
					lcd_disp_str((uint8_t *)linebuf);
					lcd_disp_str((uint8_t*)"    ");
				}
			}
		}

		/* ---------------------- ���� � UART ---------------------------- */
		if (flags.uart_dump && !flags.btn_lock) {
			flags.uart_dump = 0;
			flags.btn_lock  = 1;
			sd_flush();
			uart_dump_log();
			sd_iter_reset();
			lcd_mov_cursor(16);
			if (sd_read_line(+1, linebuf, sizeof linebuf) == 0) {
				nav_pos = 0;
				lcd_disp_str((uint8_t *)linebuf);
			}

			flags.btn_lock = 0;
		}
	}
	
}

