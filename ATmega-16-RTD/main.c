// avr_sd_logger_refactored.c – SD?логгер температуры с 7?сегм. индикатором
// ----------------------------------------------------------------------------
// Ключевая идея рефакторинга:  *никаких* обращений к шине SPI и «долгих»
// операций (SD?карта, MAX31865, LCD) внутри прерываний.  Прерывания только
// выставляют флаги?события, а главный цикл обрабатывает их последовательно.
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

#define HOLD_INITIAL_DELAY_MS 250    // после первого шага
#define REP_DELAY_MIN_MS        1   // минимальная задержка между шагами
#define REP_DELAY_BASE_MS     500   // базовая величина для расчёта

static volatile uint16_t fwd_hold_cnt, fwd_rep_cnt;
static volatile uint16_t bwd_hold_cnt, bwd_rep_cnt;
static volatile uint32_t  ms_ticks = 0;         // миллисекунды от старта
static volatile uint8_t   nav_end_event = 0;    // «двойной клик» вперёд ? конец
static volatile uint8_t   nav_start_event = 0;  // «двойной клик» назад ? начало


/* ---------------------------- Флаги -------------------------------------- */
/* ---------------------------- Флаги -------------------------------------- */
struct {
	uint8_t one_sec     :1;
	uint8_t ten_sec     :1;
	uint8_t nav_fwd     :1;    // запрос «вперёд» от пользователя
	uint8_t nav_bwd     :1;    // запрос «назад» от пользователя
	uint8_t uart_dump   :1;    // запрос дампа лога в UART
	uint8_t btn_lock    :1;    // взаимоблокировка на время работы с SD
	uint8_t history     :1;    // запрос переключения режима истории
	uint8_t show_history:1;    // флаг активного режима истории
} volatile flags;

/* --------------------------- Глобальные ---------------------------------- */
static uint32_t meas_no  = 0;               // номер измерения
static uint32_t nav_pos  = 0;               // текущая позиция во время навигации
static struct pcf_time rtc_time;            // время из PCF8583
volatile uint8_t disp_digits[4];            // буфер 7?seg
static uint32_t cnt_uart = 0;
static char linebuf[32];                    // общий рабочий буфер
static char linebuf_hist[32];

/* --------------------- Вспомогательные функции --------------------------- */
static inline uint8_t bcd2dec(uint8_t v)  { return ((v >> 4) * 10) + (v & 0x0F); }

static inline void time2str(const struct pcf_time *t, char *buf)
{
	sprintf(buf, "%02u:%02u:%02u",
	bcd2dec(t->hours   & 0x3F),
	bcd2dec(t->minutes & 0x7F),
	bcd2dec(t->seconds & 0x7F));
}

/* ---------------------- Отображение температуры -------------------------- */
static void temperature_to_digits_tenth(float temp)
{
	uint8_t digits[4];
	uint8_t negative = (temp < 0);
	if (negative) temp = -temp;

	uint16_t hundredths = (uint16_t)(temp * 100.0f + 0.5f);
	if (hundredths > 9999) hundredths = 9999;

	sevseg_bin2bcd(hundredths, digits);

	if (negative)      digits[3] = 0x0A;   // минус
	else if (digits[3] == 0) digits[3] = 0;

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		for (uint8_t i = 0; i < 4; i++) disp_digits[i] = digits[i];
	}

	sevseg_set_dot(2);
	sevseg_blank_leading(1);
}


static void go_to_log_end(void) {
	nav_pos = meas_no;
	if (sd_read_line_at(nav_pos, linebuf, sizeof linebuf) == 0) {
		char *endl = strchr(linebuf, '\n');
		if (endl) *endl = 0;
		lcd_mov_cursor(16);
		lcd_disp_str((uint8_t *)linebuf);
		lcd_disp_str((uint8_t*)"    ");
	}
}

static void go_to_log_start(void) {
	nav_pos = 1;
	if (sd_read_line_at(nav_pos, linebuf, sizeof linebuf) == 0) {
		char *endl = strchr(linebuf, '\n');
		if (endl) *endl = 0;
		lcd_mov_cursor(16);
		lcd_disp_str((uint8_t *)linebuf);
		lcd_disp_str((uint8_t*)"    ");
	}
}

/* -------------------------- UART helper ---------------------------------- */
static inline void uart_putc(char c) { while(!(UCSRA & (1<<UDRE))); UDR = c; }
static void uart_puts(const char *s) { while(*s) uart_putc(*s++); }

/* --------------------------- Таймер 1 – 1 Гц ----------------------------- */
ISR(TIMER1_COMPA_vect)
{
	static uint8_t sec_cnt = 0;
	flags.one_sec = 1;
	if (++sec_cnt >= 1) { sec_cnt = 0; flags.ten_sec = 1; }
}

/* ------------------- Таймер 0 – мультиплекс + кнопки --------------------- */
ISR(TIMER0_OVF_vect)
{
	// 1) Обновляем мультиплекс 7-segment
	sevseg_display_process(disp_digits);

	// 2) Приближённый миллисекундный счётчик (~1.024 ms на 8 MHz/64)
	ms_ticks++;

	// 3) Чтение кнопок (active-low)
	static uint8_t prev = BTN_FWD_MASK | BTN_BWD_MASK | (1<<PC7) | (1<<PC3);
	uint8_t cur         = PINC & (BTN_FWD_MASK | BTN_BWD_MASK | (1<<PC7) | (1<<PC3));
	uint8_t changed     = prev ^ cur;
	uint8_t pressed_edge= changed & (~cur);

	// 4) Детектор двойного клика
	static uint32_t last_fwd_ms = 0, last_bwd_ms = 0;
	static uint8_t  fwd_clicks  = 0, bwd_clicks  = 0;
	const uint32_t DOUBLE_CLICK_MS = 70;

	// ---- Вперёд (PC5) ----
	if (pressed_edge & BTN_FWD_MASK) {
		uint32_t now = ms_ticks;
		if (now - last_fwd_ms <= DOUBLE_CLICK_MS) {
			fwd_clicks++;
			} else {
			fwd_clicks = 1;
		}
		last_fwd_ms = now;

		if (fwd_clicks == 2) {
			nav_end_event = 1;    // выставляем флаг «перейти в конец»
			fwd_clicks   = 0;
		}
		// одиночный шаг навигации
		flags.nav_fwd = 1;
		fwd_hold_cnt  = 0;
		fwd_rep_cnt   = HOLD_INITIAL_DELAY_MS;
	}
	if (!(cur & BTN_FWD_MASK)) {
		// удержание ? автоповтор
		if (fwd_hold_cnt < UINT16_MAX) fwd_hold_cnt++;
		if (fwd_hold_cnt >= HOLD_INITIAL_DELAY_MS) {
			if (fwd_rep_cnt == 0) {
				flags.nav_fwd = 1;
				uint32_t delay = REP_DELAY_BASE_MS / (meas_no ? meas_no : 1);
				if (delay < REP_DELAY_MIN_MS) delay = REP_DELAY_MIN_MS;
				fwd_rep_cnt = delay;
				} else {
				fwd_rep_cnt--;
			}
		}
		} else {
		fwd_hold_cnt = 0;
	}

	// ---- Назад (PC6) ----
	if (pressed_edge & BTN_BWD_MASK) {
		uint32_t now = ms_ticks;
		if (now - last_bwd_ms <= DOUBLE_CLICK_MS) {
			bwd_clicks++;
			} else {
			bwd_clicks = 1;
		}
		last_bwd_ms = now;

		if (bwd_clicks == 2) {
			nav_start_event = 1;  // выставляем флаг «перейти в начало»
			bwd_clicks     = 0;
		}
		// одиночный шаг навигации
		flags.nav_bwd = 1;
		bwd_hold_cnt  = 0;
		bwd_rep_cnt   = HOLD_INITIAL_DELAY_MS;
	}
	if (!(cur & BTN_BWD_MASK)) {
		// удержание ? автоповтор
		if (bwd_hold_cnt < UINT16_MAX) bwd_hold_cnt++;
		if (bwd_hold_cnt >= HOLD_INITIAL_DELAY_MS) {
			if (bwd_rep_cnt == 0) {
				flags.nav_bwd = 1;
				uint32_t delay = REP_DELAY_BASE_MS / (meas_no ? meas_no : 1);
				if (delay < REP_DELAY_MIN_MS) delay = REP_DELAY_MIN_MS;
				bwd_rep_cnt = delay;
				} else {
				bwd_rep_cnt--;
			}
		}
		} else {
		bwd_hold_cnt = 0;
	}

	// ---- История (PC3) ----
	if (pressed_edge & (1<<PC3)) {
		flags.history = 1;
	}

	// ---- UART-дамп (PC7) ----
	if (pressed_edge & (1<<PC7)) {
		flags.uart_dump = 1;
	}

	// 5) Сохраняем текущее состояние для следующего раза
	prev = cur;
}

static uint32_t last_sent_entry = 0;

static void uart_dump_log(void)
{
	uint32_t current_entry = 0;
	sd_iter_reset(); 
	
	while (current_entry < meas_no) {
		current_entry++;
		if(current_entry > last_sent_entry) {
			sd_read_line_at(current_entry, linebuf, sizeof linebuf);
			uart_puts(linebuf);
			uart_putc('\n');
		}
	}
	
	if(current_entry > last_sent_entry) {
		last_sent_entry = current_entry;
	}
	
	sd_iter_to_end();
	
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
	OCR1A  = 2047;//31249;                       /* 1 Гц                          */
	TIMSK |= (1<<OCIE1A);                 /* разрешить прерывание Compare  */
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
	
	DDRC &= ~(BTN_MASK|BTN_FWD_MASK | BTN_BWD_MASK);  /* входы?кнопки      */
	PORTC |=  (BTN_MASK|BTN_FWD_MASK | BTN_BWD_MASK); /* pull?up           */
	DDRC &= ~(1<<PC7);
	PORTC |= (1 << PC7);
	/* Timer0: уже используется библиотекой sevseg для мультиплексации */
	TCCR0 = (1<<CS01) | (1<<CS00);     /* prescaler 64, ~1.024 мс overflow */
	TIMSK |= (1<<TOIE0);
	
	timer1_init();                      /* 1 Гц системный «тик»            */
	sd_iter_reset();                    /* позиция чтения логфайла */
	
	twi_init();
	sei();                              /* глобально разрешить IRQ         */
	pcf_init();

	meas_no = sd_get_entry_count();


	/* --- главный цикл -------------------------------------------------- */
	while (1) {
		
		
		if (flags.history && !flags.btn_lock) {
			flags.history = 0;
			flags.show_history = !flags.show_history;
			
			if (flags.show_history) {
				lcd_mov_cursor(16);
				lcd_disp_str((uint8_t *)linebuf_hist);
				lcd_disp_str((uint8_t*)"    "); // Очистка остатков
				} else {
				sd_iter_reset();
				if (sd_read_line(+1, linebuf, sizeof linebuf) == 0) {
					// Обрезаем возможный перевод строки
					char *endl = strchr(linebuf, '\n');
					if(endl) *endl = 0;
					
					lcd_mov_cursor(16);
					lcd_disp_str((uint8_t *)linebuf);
					lcd_disp_str((uint8_t*)"    "); // Очистка хвоста
				}
			}
		}
		
		
		
		
		/* --------- ежесекундные задачи -------------------------------- */
		if (flags.one_sec) {
			flags.one_sec = 0;

			pcf_read_time(&rtc_time);
			while (twi_status != TWI_STATUS_RX_COMPLETE);
			twi_status = TWI_STATUS_READY;

			// показываем температуру (X.Y)
			temperature = max31865_read_temperature();
			temperature_to_digits_tenth(temperature);

			/* обновление первой строки LCD (время + кол?во записей) */
			char tstr[10], cnt[6];
			time2str(&rtc_time, tstr);
			sprintf(cnt, "%lu", meas_no );

			lcd_mov_cursor(0);
			lcd_disp_str((uint8_t *)tstr);
			lcd_mov_cursor(10);
			lcd_disp_str((uint8_t *)"#");
			lcd_disp_str((uint8_t *)cnt);
		}

		/* ---------- каждые 10 с – измерение + лог ---------------------- */
		if (flags.ten_sec) {
			flags.ten_sec = 0;
			if (!flags.btn_lock) {
				flags.btn_lock = 1;

				int16_t t100 = (int16_t)(temperature * 100.0f + 0.5f);
				++meas_no;
				sprintf(linebuf, "%lu %02u:%02u,%+d.%02u", // Убрали \n в конце
				meas_no,
				bcd2dec(rtc_time.hours & 0x3F),
				bcd2dec(rtc_time.minutes),
				t100/100,
				abs(t100)%100);
				
				// Копируем с очисткой буфера
				memset(linebuf_hist, 0, sizeof(linebuf_hist)); // Очищаем буфер
				strncpy(linebuf_hist, linebuf, sizeof(linebuf_hist)-1); // Безопасное копирование
				
				strcat(linebuf, "\n"); // Добавляем перевод строки для SD-карты
				sd_write_line(linebuf);
				sd_flush();
				sd_erase_sector(meas_no + 1);

				if (flags.show_history) {
					lcd_mov_cursor(16);
					lcd_disp_str((uint8_t *)linebuf_hist);
					lcd_disp_str((uint8_t*)"    "); // Очистка хвоста
				}

				flags.btn_lock = 0;
			}
		}


		if (!flags.show_history) {
			// ——— обработка двойных кликов ———
			if (nav_end_event && !flags.btn_lock) {
				nav_end_event = 0;
				flags.btn_lock = 1;
				sd_iter_to_end();       // итератор в конец (next_sector)
				go_to_log_end();        // рисуем последнюю строку
				flags.btn_lock = 0;
			}
			if (nav_start_event && !flags.btn_lock) {
				nav_start_event = 0;
				flags.btn_lock = 1;
				sd_iter_reset();        // итератор в начало (sector=1)
				go_to_log_start();      // рисуем первую строку
				flags.nav_bwd = 0;      // сбросим одиночный «назад»
				flags.btn_lock = 0;
			}

			if (!flags.show_history) {
				// обработка двойных кликов прежняя …
				// …

				// ——— шаговая навигация вперёд/назад чисто по nav_pos ———
				if (flags.nav_fwd && !flags.btn_lock) {
					flags.nav_fwd = 0;
					if (nav_pos < meas_no) {
						nav_pos++;
						// сразу читаем конкретную строку
						if (sd_read_line_at(nav_pos, linebuf, sizeof linebuf) == 0) {
							char *e = strchr(linebuf, '\n');
							if (e) *e = 0;
							lcd_mov_cursor(16);
							lcd_disp_str((uint8_t*)linebuf);
							lcd_disp_str((uint8_t*)"    ");
						}
					}
				}

				if (flags.nav_bwd && !flags.btn_lock) {
					flags.nav_bwd = 0;
					if (nav_pos > 1) {
						nav_pos--;
						if (sd_read_line_at(nav_pos, linebuf, sizeof linebuf) == 0) {
							char *e = strchr(linebuf, '\n');
							if (e) *e = 0;
							lcd_mov_cursor(16);
							lcd_disp_str((uint8_t*)linebuf);
							lcd_disp_str((uint8_t*)"    ");
						}
					}
				}
			}
		}

		        if (flags.uart_dump && !flags.btn_lock) {
			        flags.uart_dump = 0;
			        flags.btn_lock  = 1;

			        sd_flush();
			        uart_dump_log();

			        // === НОВЫЙ БЛОК ОЧИСТКИ ===
			        // полностью очищаем SD-карту и сбрасываем все счётчики
			        sd_flush();                // убедимся, что всё записано/сброшено
			       // sd_erase_all();            // функция форматирования/стирания всех секторов
				    sd_erase_sector(0);
					sd_clear_log(1);
			        //meas_no = 0;               // обнуляем номер текущей записи
			        nav_pos = 0;               // обнуляем позицию навигации
			        last_sent_entry = 0;       // обнуляем маркер последнего отправленного
			        // обновим отображение счётчика записей на LCD
					lcd_send_cmd(1 << LCD_CLR);
					_delay_ms(2);
			        lcd_mov_cursor(10);
			        lcd_disp_str((uint8_t*)"#0   ");
			        // === КОНЕЦ НОВОГО БЛОКА ===

			        // для пользователя можно показать пустой лог (или оставить текущее отображение)
			        flags.btn_lock = 0;
		        }
		}
	
}

