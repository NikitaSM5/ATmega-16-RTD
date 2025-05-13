#include "sevseg.h"
#include "max31865.h"
#include "main.h"
#include "twi.h"
#include "pcf8583.h"
#include "sdcard.h"
#include "lcd16x2.h"
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#define F_CPU 8000000UL


/* ----------------------------- GPIO -------------------------------------- */
#define BTN_MASK (1 << PC3)
#define BTN_FWD PC5
#define BTN_BWD PC6
#define BTN_FWD_MASK (1 << BTN_FWD)
#define BTN_BWD_MASK (1 << BTN_BWD)
#define LED_MASK (1 << PD7)

#define HOLD_INITIAL_DELAY_MS 250
#define REP_DELAY_MIN_MS 1
#define REP_DELAY_BASE_MS 500

static volatile uint16_t fwd_hold_cnt, fwd_rep_cnt;
static volatile uint16_t bwd_hold_cnt, bwd_rep_cnt;
static volatile uint32_t ms_ticks = 0;
static volatile uint8_t nav_end_event = 0;
static volatile uint8_t nav_start_event = 0;
static volatile uint32_t last_sent_meas = 0;

/* ---------------------------- Flags -------------------------------------- */
struct {
	uint8_t one_sec : 1;
	uint8_t ten_sec : 1;
	uint8_t nav_fwd : 1;
	uint8_t nav_bwd : 1;
	uint8_t uart_dump : 1;
	uint8_t btn_lock : 1;
	uint8_t history : 1;
	uint8_t hist : 1;
} volatile flags;

/* --------------------------- GLOBAL ---------------------------------- */
static uint32_t meas_no = 0;
static uint32_t nav_pos = 0;
static struct pcf_time rtc_time;
volatile uint8_t disp_digits[4];
static char linebuf[32];
static char linebuf_hist[32];
static char scroll_buf[32];
static uint8_t scroll_len = 0, scroll_ofs = 0;
static uint32_t scroll_t0 = 0;
static uint32_t last_sent_entry = 0;

/* ------------------------- Misc ------------------------------- */
#define SCROLL_PAUSE_MS 1000
#define SCROLL_STEP_MS 250

static inline uint8_t bcd2dec(uint8_t v) {
	return ((v >> 4) * 10) + (v & 0x0F);
}

static inline void time2str(const struct pcf_time *t, char *buf) {
	sprintf(buf, "%02u:%02u:%02u", bcd2dec(t->hours & 0x3F),
	bcd2dec(t->minutes & 0x7F), bcd2dec(t->seconds & 0x7F));
}

static void temperature_to_digits_tenth(float temp) {
	uint8_t digits[4];
	uint8_t negative = (temp < 0);
	if (negative) temp = -temp;

	uint16_t hundredths = (uint16_t)(temp * 100.0f + 0.5f);
	if (hundredths > 9999) hundredths = 9999;

	sevseg_bin2bcd(hundredths, digits);

	if (negative)
	digits[3] = 0x0A;
	else if (digits[3] == 0)
	digits[3] = 0;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	for (uint8_t i = 0; i < 4; i++) disp_digits[i] = digits[i];

	sevseg_set_dot(2);
	sevseg_blank_leading(1);
}

/* ------------------------- Навигация лога -------------------------------- */
static void go_to_log_end(void) {
	nav_pos = meas_no;
	if (sd_read_line_at(nav_pos, linebuf, sizeof linebuf) == 0) {
		char *e = strchr(linebuf, '\n');
		if (e) *e = 0;
		lcd_mov_cursor(16);
		lcd_disp_str((uint8_t *)linebuf);
		lcd_disp_str((uint8_t *)"    ");
		strncpy(scroll_buf, linebuf, sizeof scroll_buf - 1);
		scroll_buf[sizeof scroll_buf - 1] = 0;
		scroll_len = strlen(scroll_buf);
		scroll_ofs = 0;
		scroll_t0 = ms_ticks;
	}
}

static void go_to_log_start(void) {
	nav_pos = 1;
	if (sd_read_line_at(nav_pos, linebuf, sizeof linebuf) == 0) {
		char *e = strchr(linebuf, '\n');
		if (e) *e = 0;
		lcd_mov_cursor(16);
		lcd_disp_str((uint8_t *)linebuf);
		lcd_disp_str((uint8_t *)"    ");
		strncpy(scroll_buf, linebuf, sizeof scroll_buf - 1);
		scroll_buf[sizeof scroll_buf - 1] = 0;
		scroll_len = strlen(scroll_buf);
		scroll_ofs = 0;
		scroll_t0 = ms_ticks;
	}
}

/* --------------------------- UART -------------------------------------- */
static inline void uart_putc(char c) {
	while (!(UCSRA & (1 << UDRE)));
	UDR = c;
}
static void uart_puts(const char *s) {
	while (*s) uart_putc(*s++);
}

/* -------------------------- Бегущая строка ------------------------------- */
static void scroll_update(void) {
	if (scroll_len <= 16) return;
	uint32_t now = ms_ticks;
	if (scroll_ofs == 0 && now - scroll_t0 < SCROLL_PAUSE_MS) return;
	if (now - scroll_t0 >= SCROLL_STEP_MS) {
		scroll_t0 = now;
		if (++scroll_ofs > scroll_len - 16) {
			scroll_ofs = 0;
			scroll_t0 = now;
		}
		lcd_mov_cursor(16);
		for (uint8_t i = 0; i < 16; i++) {
			char c = (i + scroll_ofs < scroll_len) ? scroll_buf[i + scroll_ofs]
			: ' ';
			lcd_send_char(c);
		}
	}
}

/* --------------------------- Timer 1 comp ----------------------------- */
ISR(TIMER1_COMPA_vect) {
	static uint8_t sec_cnt = 0;
	flags.one_sec = 1;
	if (++sec_cnt >= 2) {
		sec_cnt = 0;
		flags.ten_sec = 1;
	}
}

/* ------------------- Timer 0 OVF --------------------- */
ISR(TIMER0_OVF_vect) {
	if (flags.uart_dump)
	PORTA = 0x00;
	else
	sevseg_display_process(disp_digits);

	ms_ticks++;

	static uint8_t prev = BTN_FWD_MASK | BTN_BWD_MASK | (1 << PC7) | (1 << PC3);
	uint8_t cur =
	PINC & (BTN_FWD_MASK | BTN_BWD_MASK | (1 << PC7) | (1 << PC3));
	uint8_t changed = prev ^ cur;
	uint8_t pressed_edge = changed & (~cur);

	static uint32_t last_fwd_ms = 0, last_bwd_ms = 0;
	static uint8_t fwd_clicks = 0, bwd_clicks = 0;
	const uint32_t DOUBLE_CLICK_MS = 70;

	if (pressed_edge & BTN_FWD_MASK) {
		uint32_t now = ms_ticks;
		fwd_clicks =
		(now - last_fwd_ms <= DOUBLE_CLICK_MS) ? fwd_clicks + 1 : 1;
		last_fwd_ms = now;
		if (fwd_clicks == 2) {
			nav_end_event = 1;
			fwd_clicks = 0;
		}
		flags.nav_fwd = 1;
		fwd_hold_cnt = 0;
		fwd_rep_cnt = HOLD_INITIAL_DELAY_MS;
	}
	if (!(cur & BTN_FWD_MASK)) {
		if (fwd_hold_cnt < UINT16_MAX) fwd_hold_cnt++;
		if (fwd_hold_cnt >= HOLD_INITIAL_DELAY_MS) {
			if (fwd_rep_cnt == 0) {
				flags.nav_fwd = 1;
				uint32_t delay = REP_DELAY_BASE_MS / (meas_no ? meas_no : 1);
				if (delay < REP_DELAY_MIN_MS) delay = REP_DELAY_MIN_MS;
				fwd_rep_cnt = delay;
			} else
			fwd_rep_cnt--;
		}
	} else
	fwd_hold_cnt = 0;

	if (pressed_edge & BTN_BWD_MASK) {
		uint32_t now = ms_ticks;
		bwd_clicks =
		(now - last_bwd_ms <= DOUBLE_CLICK_MS) ? bwd_clicks + 1 : 1;
		last_bwd_ms = now;
		if (bwd_clicks == 2) {
			nav_start_event = 1;
			bwd_clicks = 0;
		}
		flags.nav_bwd = 1;
		bwd_hold_cnt = 0;
		bwd_rep_cnt = HOLD_INITIAL_DELAY_MS;
	}
	if (!(cur & BTN_BWD_MASK)) {
		if (bwd_hold_cnt < UINT16_MAX) bwd_hold_cnt++;
		if (bwd_hold_cnt >= HOLD_INITIAL_DELAY_MS) {
			if (bwd_rep_cnt == 0) {
				flags.nav_bwd = 1;
				uint32_t delay = REP_DELAY_BASE_MS / (meas_no ? meas_no : 1);
				if (delay < REP_DELAY_MIN_MS) delay = REP_DELAY_MIN_MS;
				bwd_rep_cnt = delay;
			} else
			bwd_rep_cnt--;
		}
	} else
	bwd_hold_cnt = 0;

	if (pressed_edge & (1 << PC3)) flags.history = 1;
	if (pressed_edge & (1 << PC7)) flags.uart_dump = 1;
	prev = cur;
}

/* -------------------------- UART dump ------------------------------- */
static void uart_dump_log(void) {
	uint32_t current_entry = 0;
	sd_iter_reset();
	lcd_mov_cursor(0);
	for (uint8_t i = 0; i < 16; i++) lcd_send_char(' ');
	uint8_t shown_blocks = 0;
	static char current_msg[16];

	while (current_entry < meas_no) {
		++current_entry;
		if (current_entry > last_sent_entry) {
			uint8_t blocks = (uint32_t)current_entry * 16 / meas_no;
			if (blocks > shown_blocks) {
				lcd_mov_cursor(shown_blocks);
				for (uint8_t i = shown_blocks; i < blocks; i++)
				lcd_send_char(0xFF);
				shown_blocks = blocks;
			}
			lcd_mov_cursor(16);
			sprintf(current_msg, "%lu/%lu", current_entry, meas_no);
			lcd_disp_str((uint8_t *)current_msg);
			lcd_mov_cursor(16);
			sd_read_line_at(current_entry, linebuf, sizeof linebuf);
			uart_puts(linebuf);
			uart_putc('\n');
		}
	}

	if (current_entry > last_sent_entry) {
		last_sent_meas += current_entry - last_sent_entry;
		last_sent_entry = current_entry;
	}
	sd_iter_to_end();
	_delay_ms(500);
}

/* --------------------------- Initialization ------------------------------ */
static void uart_init(void) {
	UCSRA |= (1 << U2X);
	UBRRL = 51;  // 56kBOD
	UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);
	UCSRB |= (1 << RXCIE) | (1 << RXEN) | (1 << TXEN);
}

static void timer0_init(void) {
	TCCR0 = (1 << CS01) | (1 << CS00);
	TIMSK |= (1 << TOIE0);
}

static void timer1_init(void) {
	TCCR1B = (1 << WGM12) | (1 << CS12);
	OCR1A = 1023;
	TIMSK |= (1 << OCIE1A);
}

static void port_init(void) {
	DDRC &= ~(BTN_MASK | BTN_FWD_MASK | BTN_BWD_MASK);
	PORTC |= (BTN_MASK | BTN_FWD_MASK | BTN_BWD_MASK);
	DDRC &= ~(1 << PC7);
	PORTC |= (1 << PC7);
}

/* ----------------------------- main -------------------------------------- */
int main(void) {
	float temperature = 0.0f;

	uart_init();
	port_init();
	sevseg_init();
	lcd_init();
	lcd_disp_str((uint8_t *)"1");
	max31865_init(0, 0);
	lcd_disp_str((uint8_t *)"2");
	lcd_mov_cursor(16);

	timer0_init();
	timer1_init();
	sd_iter_reset();

	twi_init();
	sei();
	pcf_init();

	meas_no = sd_get_entry_count();

	while (1) {
		
		if (flags.history && !flags.btn_lock) {
			flags.history = 0;
			flags.hist = !flags.hist;
			nav_end_event = nav_start_event = 0;
			if (flags.hist) {
				lcd_mov_cursor(16);
				lcd_disp_str((uint8_t *)linebuf_hist);
				lcd_disp_str((uint8_t *)"    ");
				strncpy(scroll_buf, linebuf_hist, sizeof scroll_buf - 1);
				scroll_buf[sizeof scroll_buf - 1] = 0;
				} else {
				sd_iter_reset();
				nav_pos = meas_no;
				sd_read_line_at(meas_no, linebuf, sizeof linebuf);
				char *e = strchr(linebuf, '\n');
				if (e) *e = 0;
				lcd_mov_cursor(16);
				lcd_disp_str((uint8_t *)linebuf);
				lcd_disp_str((uint8_t *)"    ");
				strncpy(scroll_buf, linebuf, sizeof scroll_buf - 1);
				scroll_buf[sizeof scroll_buf - 1] = 0;
			}
			scroll_len = strlen(scroll_buf);
			scroll_ofs = 0;
			scroll_t0 = ms_ticks;
		}

		if (flags.one_sec) {
			flags.one_sec = 0;
			pcf_read_time(&rtc_time);
			while (twi_status != TWI_STATUS_RX_COMPLETE);
			twi_status = TWI_STATUS_READY;

			temperature = max31865_read_temperature();
			temperature_to_digits_tenth(temperature);

			char tstr[10], cnt[6];
			time2str(&rtc_time, tstr);
			sprintf(cnt, "%lu", meas_no + last_sent_meas);
			lcd_mov_cursor(0);
			lcd_disp_str((uint8_t *)tstr);
			lcd_mov_cursor(10);
			lcd_disp_str((uint8_t *)"#");
			lcd_disp_str((uint8_t *)cnt);
		}

		if (flags.ten_sec) {
			flags.ten_sec = 0;
			if (!flags.btn_lock) {
				flags.btn_lock = 1;
				int16_t t100 = (int16_t)(temperature * 100.0f + 0.5f);
				++meas_no;
				sprintf(linebuf, "%lu %02u:%02u,%+d.%02u",
				meas_no + last_sent_meas,
				bcd2dec(rtc_time.hours & 0x3F),
				bcd2dec(rtc_time.minutes), t100 / 100, abs(t100) % 100);
				memset(linebuf_hist, 0, sizeof linebuf_hist);
				strncpy(linebuf_hist, linebuf, sizeof linebuf_hist - 1);
				strcat(linebuf, "\n");
				sd_write_line(linebuf);
				sd_erase_sector(meas_no + 1);
				if (flags.hist) {
					lcd_mov_cursor(16);
					lcd_disp_str((uint8_t *)linebuf_hist);
					lcd_disp_str((uint8_t *)"    ");
					strncpy(scroll_buf, linebuf_hist, sizeof scroll_buf - 1);
					scroll_buf[sizeof scroll_buf - 1] = 0;
					scroll_len = strlen(scroll_buf);
					scroll_ofs = 0;
					scroll_t0 = ms_ticks;
				}
				flags.btn_lock = 0;
			}
		}

		if (!flags.hist) {
			if (nav_end_event && !flags.btn_lock) {
				nav_end_event = 0;
				flags.btn_lock = 1;
				sd_iter_to_end();
				go_to_log_end();
				flags.btn_lock = 0;
			}
			if (nav_start_event && !flags.btn_lock) {
				nav_start_event = 0;
				flags.btn_lock = 1;
				sd_iter_reset();
				go_to_log_start();
				flags.nav_bwd = 0;
				flags.btn_lock = 0;
			}
			if (flags.nav_fwd && !flags.btn_lock) {
				flags.nav_fwd = 0;
				if (nav_pos < meas_no &&
				sd_read_line_at(++nav_pos, linebuf, sizeof linebuf) == 0) {
					char *e = strchr(linebuf, '\n');
					if (e) *e = 0;
					lcd_mov_cursor(16);
					lcd_disp_str((uint8_t *)linebuf);
					lcd_disp_str((uint8_t *)"    ");
					strncpy(scroll_buf, linebuf, sizeof scroll_buf - 1);
					scroll_buf[sizeof scroll_buf - 1] = 0;
					scroll_len = strlen(scroll_buf);
					scroll_ofs = 0;
					scroll_t0 = ms_ticks;
				}
			}
			if (flags.nav_bwd && !flags.btn_lock) {
				flags.nav_bwd = 0;
				if (nav_pos > 1 &&
				sd_read_line_at(--nav_pos, linebuf, sizeof linebuf) == 0) {
					char *e = strchr(linebuf, '\n');
					if (e) *e = 0;
					lcd_mov_cursor(16);
					lcd_disp_str((uint8_t *)linebuf);
					lcd_disp_str((uint8_t *)"    ");
					strncpy(scroll_buf, linebuf, sizeof scroll_buf - 1);
					scroll_buf[sizeof scroll_buf - 1] = 0;
					scroll_len = strlen(scroll_buf);
					scroll_ofs = 0;
					scroll_t0 = ms_ticks;
				}
			}
		}

		if (flags.uart_dump && !flags.btn_lock) {
			flags.btn_lock = 1;
			lcd_send_cmd(1 << LCD_CLR);
			_delay_ms(2);
			uart_dump_log();
			sd_erase_sector(0);
			sd_clear_log(1);
			meas_no = 0;
			nav_pos = 0;
			last_sent_entry = 0;
			lcd_send_cmd(1 << LCD_CLR);
			_delay_ms(2);
			lcd_mov_cursor(10);
			lcd_disp_str((uint8_t *)"#0   ");
			flags.btn_lock = 0;
			flags.uart_dump = 0;
		}

		scroll_update();
	}
}
