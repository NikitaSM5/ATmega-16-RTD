
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
#define BTN_MASK        (1<<PC3)          /* Ђрежим/holdї кнопка           */
#define BTN_FWD         PC5               /* ЂвперЄдї  Ц следующа€ запись  */
#define BTN_BWD         PC6               /* Ђназадї   Ц предыдуща€ запись */
#define BTN_FWD_MASK    (1<<BTN_FWD)
#define BTN_BWD_MASK    (1<<BTN_BWD)
#define LED_MASK        (1<<PD7)

/* --- Global variables ---------------------------------------------------- */
static uint32_t meas_no  = 0;             /* номер измерени€/строки        */
struct pcf_time rtc_time;                 /* буфер времени из PCF8583       */
uint32_t btn_fwd_clicked = 0;
uint32_t btn_bwd_clicked = 0;

volatile uint8_t disp_digits[4];          /* буфер 7?segment индикатора     */
static inline uint8_t bcd2dec(uint8_t v)  { return ((v >> 4) * 10) + (v & 0x0F); }

volatile struct user_flags {                       /* антидребезг дл€ навигации SD   */
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

    // scale to hundredths and round
    uint16_t hundredths = (uint16_t)(temp * 100.0f + (temp >= 0 ? 0.5f : -0.5f));
    if (hundredths > 9999) hundredths = 9999;

    // convert to BCD for each of the 4 digits
    sevseg_bin2bcd(hundredths, digits);

    // if negative, put УminusФ code in the MSB position
    if (negative) {
	    digits[3] = 0x0A;          // dec2sevseg[0x0A] ? minus
	    } else if (digits[3] == 0) {
	    // optional: blank an unused leading zero
	    // requires enabling blank_leading below
	    digits[3] = 0;
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
	    for (uint8_t i = 0; i < 4; i++)
	    disp_digits[i] = digits[i];
    }

    sevseg_set_dot(2);            // dot between 2nd and 3rd digit (from the right)
    sevseg_blank_leading(1);      // turn on leading-zero blanking
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
    sevseg_set_dot(2);                    /* точка между ћћ и SS           */
}

/* --------------------------- Timer†1 setup --------------------------------
 *  8†ћ√ц / 256 = 31†250 тиков в секунду
 *  OCR1A = 31249 ==> прерывание ровно раз в секунду (CTC?режим)
 *  ¬ ISR выставл€ем флаги дл€ главного цикла.
 * ------------------------------------------------------------------------- */
static void timer1_init(void)
{
    TCCR1B = (1<<WGM12) | (1<<CS12);      /* CTC, prescaler 256            */
    OCR1A  = 31249;                       /* 1†√ц                          */
    TIMSK |= (1<<OCIE1A);                 /* разрешить прерывание Compare  */
}

ISR(TIMER1_COMPA_vect)
{
  
    static uint8_t sec_cnt = 0;
    flags.flag_1s = 1;                          /* каждую секунду                 */
    if (++sec_cnt >= 5) {                /* каждые 10†с                    */
        sec_cnt = 0;
        flags.flag_10s = 1;
    }
}

/* ---------------- TIMER0: сканирование индикатора + кнопки SD navigate ---- */
char buf[32];
ISR(TIMER0_OVF_vect)
{
   sevseg_display_process(disp_digits);
/*---- навигаци€ по файлу SD -------------------------------------------*/
    if ((PINC & BTN_FWD_MASK) == 0) {
        if (!flags.btn_fwd) {
            flags.btn_fwd = 1;
		  if(btn_fwd_clicked < meas_no){
			  btn_fwd_clicked++;
            lcd_mov_cursor(16);
            if (sd_read_line(+1, buf, sizeof buf) == 0) {
                lcd_disp_str((uint8_t *)buf);
            }
		}
        }
    } else flags.btn_fwd = 0;

    if ((PINC & BTN_BWD_MASK) == 0) {
        if (!flags.btn_bwd) {
            flags.btn_bwd = 1;
            sd_iter_reset();
            lcd_mov_cursor(16);
            if (sd_read_line(+1, buf, sizeof buf) == 0) {
                btn_fwd_clicked = 0;
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
    /* --- HAL/перифери€ ---------------------------------------------------*/
  uart_init();
  sevseg_init();
  DDRD |= (1 << PD2)|(1 << PD3)|(1 << PD4)|(1 << PD5)|(1 << PD6)|(1 << PD7);
  
    lcd_init();
    max31865_init(0, 0);
    //lcd_disp_str((uint8_t *)"Time: ");
    lcd_mov_cursor(16);
    sd_flush();

    DDRC &= ~(BTN_MASK|BTN_FWD_MASK | BTN_BWD_MASK);  /* входы?кнопки      */
    PORTC |=  (BTN_MASK|BTN_FWD_MASK | BTN_BWD_MASK); /* pull?up           */
    DDRC &= ~(1<<PC7);
    PORTC |= (1 << PC7);
    /* Timer0: уже используетс€ библиотекой sevseg дл€ мультиплексации */
    TCCR0 = (1<<CS01) | (1<<CS00);     /* prescaler 64, ~1.024†мс overflow */
    TIMSK |= (1<<TOIE0);

    timer1_init();                      /* 1†√ц системный Ђтикї            */
    sd_iter_reset();                    /* позици€ чтени€ логфайла */
  
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

           /* выбор отображени€: кнопка удерживаетс€ Ц часы, иначе температура */
           if ((PINC & BTN_MASK) == 0) {
               time_to_digits_mmss(&rtc_time);
           } else {
               temperature_to_digits_tenth(temperature);
           }

           /* строка Ђhh:mm:ssї + справа Ч число записей на SD */
           char t[10], cnt[6];
           time2str(&rtc_time, t);
           sprintf(cnt, "%lu", meas_no);
           lcd_mov_cursor(0);
           lcd_disp_str((uint8_t *)t);
           /* сразу после 8 символа времени */
           lcd_mov_cursor(12);
		   lcd_disp_str((uint8_t *)"#");
           lcd_disp_str((uint8_t *)cnt);
       }

	        if (flags.flag_10s) {                 /* каждые 10 с Ч измерить & лог    */
		        flags.flag_10s = 0;
		        
		        /* логируем с точностью до 0.01∞C */
		        int16_t t100 = (int16_t)(temperature * 100.0f + (temperature >= 0 ? 0.5f : -0.5f));
		        meas_no++;
		        /* формат: <номер> hh:mm,+X.YY\n */
		        sprintf(line, "%lu %02u:%02u,%+d.%02u\n",
		        meas_no,
		        bcd2dec(rtc_time.hours & 0x3F),
		        bcd2dec(rtc_time.minutes),
		        t100/100,
		        (uint16_t)abs(t100)%100);
		        sd_write_line(line);
		        }

        /* необ€зательное энергосбережение: сон до следующего прерывани€ */
        //sleep_mode();
    }
}