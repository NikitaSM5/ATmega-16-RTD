#include "sevseg.h"
#include "max31865.h"
#include "main.h"
#include "twi.h"
#include "pcf8583.h"
#include "sdcard.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <util/delay.h>
#include <math.h>
#include <string.h>
#include <lcd16x2.h>

#define F_CPU 8000000UL

/* --- GPIO definitions --------------------------------------------------- */
#define BTN_MASK        (1<<PC3)          /* «режим/hold» кнопка           */
#define BTN_FWD         PC5               /* «вперёд»  – следующая запись  */
#define BTN_BWD         PC6               /* «назад»   – предыдущая запись */
#define BTN_FWD_MASK    (1<<BTN_FWD)
#define BTN_BWD_MASK    (1<<BTN_BWD)
#define LED_MASK        (1<<PD7)

/* --- Global variables ---------------------------------------------------- */
static uint32_t meas_no  = 0;             /* номер измерения/строки        */
struct pcf_time rtc_time;                 /* буфер времени из PCF8583       */

volatile uint8_t disp_digits[4];          /* буфер 7?segment индикатора     */
static inline uint8_t bcd2dec(uint8_t v)  { return ((v >> 4) * 10) + (v & 0x0F); }

volatile struct user_flags {                       /* антидребезг для навигации SD   */
    uint8_t btn_fwd:1;
    uint8_t btn_bwd:1;
	uint8_t btn_tx  :1;
	uint8_t flag_1s  :1;
	uint8_t flag_10s :1;
	uint8_t tx_req  :1;
} flags;

/* ------------------------------------------------------------------------- */
static inline void time2str(const struct pcf_time *t, char *buf)
{
    uint8_t hrs = t->hours & 0x3F;
    sprintf(buf, "%02u:%02u:%02u", bcd2dec(hrs), bcd2dec(t->minutes), bcd2dec(t->seconds));
}

static void temperature_to_digits_tenth(float temp)
{
    uint8_t digits[4];

    uint8_t negative = (temp < 0);
    if (negative) temp = -temp;

    uint16_t tenths = (uint16_t)(temp * 10.0f + 0.5f);
    tenths = (tenths > 9999) ? 9999 : tenths;

    sevseg_bin2bcd(tenths, digits);       /* младший разряд = десятые °С    */

    if (negative)       digits[3] = 0x0A; /* «минус» для 7?segment          */
    else if (digits[3] == 0) digits[3] = 0; /* убрать незначащий «0»        */

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        for (uint8_t i = 0; i < 4; i++) disp_digits[i] = digits[i];
    }
    sevseg_set_dot(1);                    /* после целых показать точку     */
}

static void uart_init(void)
{
	UCSRA |= (1<<U2X);
	UBRRL = 51; //19.2
	UCSRC |= (1 << URSEL) | (1 << UCSZ0) | (1 << UCSZ1);
	UCSRB |= (1 << RXCIE) | (1 << RXEN) | (1 << TXEN);
}

static inline void uart_putc(char c)
{
	while(!(UCSRA & (1<<UDRE))); UDRE;
	UDR = c;
}

static void uart_puts(const char *s)
{
	while(*s) uart_putc(*s++);
}

static void time_to_digits_mmss(const struct pcf_time *t)
{
    uint8_t sec = bcd2dec(t->seconds);
    uint8_t min = bcd2dec(t->minutes);

    uint8_t d[4] = { sec % 10, sec / 10, min % 10, min / 10 };
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        for (uint8_t i = 0; i < 4; i++) disp_digits[i] = d[i];
    }
    sevseg_set_dot(2);                    /* точка между ММ и SS           */
}

/* --------------------------- Timer 1 setup --------------------------------
 *  8 МГц / 256 = 31 250 тиков в секунду
 *  OCR1A = 31249 ==> прерывание ровно раз в секунду (CTC?режим)
 *  В ISR выставляем флаги для главного цикла.
 * ------------------------------------------------------------------------- */
static void timer1_init(void)
{
    TCCR1B = (1<<WGM12) | (1<<CS12);      /* CTC, prescaler 256            */
    OCR1A  = 31249;                       /* 1 Гц                          */
    TIMSK |= (1<<OCIE1A);                 /* разрешить прерывание Compare  */
}

ISR(TIMER1_COMPA_vect)
{
	
    static uint8_t sec_cnt = 0;
    flags.flag_1s = 1;                          /* каждую секунду                 */
    if (++sec_cnt >= 5) {                /* каждые 10 с                    */
        sec_cnt = 0;
        flags.flag_10s = 1;
    }
}

/* ---------------- TIMER0: сканирование индикатора + кнопки SD navigate ---- */
char buf[32];
ISR(TIMER0_OVF_vect)
{
    sevseg_display_process(disp_digits);

    /*---- навигация по файлу SD -------------------------------------------*/
    if ((PINC & BTN_FWD_MASK) == 0) {
        if (!flags.btn_fwd) {
            flags.btn_fwd = 1;
			lcd_mov_cursor(16);
			lcd_disp_str((uint8_t *)"BTN_PC5");
            if (sd_read_line(+1, buf, sizeof buf) == 0) {
                //lcd_mov_cursor(16);
                lcd_disp_str((uint8_t *)buf);
            }
        }
    } else flags.btn_fwd = 0;

    if ((PINC & BTN_BWD_MASK) == 0) {
        if (!flags.btn_bwd) {
            flags.btn_bwd = 1;
            sd_iter_reset();
			lcd_mov_cursor(16);
			lcd_disp_str((uint8_t *)"BTN_PC6");
            if (sd_read_line(+1, buf, sizeof buf) == 0) {
                //lcd_mov_cursor(16);
                lcd_disp_str((uint8_t *)buf);
            }
        }
    } else flags.btn_bwd = 0;
	
	 if(!(PINC & (1 << PC7))){
		 if(!flags.btn_tx)
		 { 
			 flags.btn_tx=1; 
			 flags.tx_req = 1;
			 }
	 }else flags.btn_tx=0;
}

/* ------------------------------------------------------------------------- */
int main(void)
{
	
	char line[32];
	float temperature = 0.0f;
    /* --- HAL/периферия ---------------------------------------------------*/
	uart_init();
	sevseg_init();
	DDRD |= (1 << PD2)|(1 << PD3)|(1 << PD4)|(1 << PD5)|(1 << PD6)|(1 << PD7);
	
    lcd_init();
    max31865_init(0, 0);
    lcd_disp_str((uint8_t *)"Time:");
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
	
	
    /* --- рабочие переменные ---------------------------------------------*/


    /* -------------------- main loop (co?operative) ----------------------*/
    while(1) {
		
		 if(flags.tx_req){
			 uart_puts(buf); 
			 flags.tx_req = 0; 
		 }
		
        if (flags.flag_1s) {                  /* ежесекундные задачи             */
            flags.flag_1s = 0;
            pcf_read_time(&rtc_time);
            while (twi_status != TWI_STATUS_RX_COMPLETE);
            twi_status = TWI_STATUS_READY;
			temperature = max31865_read_temperature();

            /* выбор отображения: кнопка удерживается – часы, иначе температура */
            if ((PINC & BTN_MASK) == 0) {
                time_to_digits_mmss(&rtc_time);
            } else {
                temperature_to_digits_tenth(temperature); /* последнее измерение */
            }

            /* строка «Time: hh:mm:ss» на LCD                                */
            char t[10];
            time2str(&rtc_time, t);
            lcd_mov_cursor(6);
            lcd_disp_str((uint8_t *)t);
        }

        if (flags.flag_10s) {                 /* каждые 10 с — измерить & лог    */
            flags.flag_10s = 0;
            
            int16_t t10 = (int16_t)(temperature * 10.0f + (temperature >= 0 ? 0.5f : -0.5f));
            meas_no++;
            sprintf(line, "%lu %02u:%02u,%+d.%01u\n",
                    meas_no,
                    bcd2dec(rtc_time.hours & 0x3F),
                    bcd2dec(rtc_time.minutes),
                    t10/10,
                    (uint16_t)abs(t10)%10);
            sd_write_line(line);
        }

        /* необязательное энергосбережение: сон до следующего прерывания */
        //sleep_mode();
    }
}
