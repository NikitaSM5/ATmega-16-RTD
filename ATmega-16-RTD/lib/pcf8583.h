#ifndef PCF8583_H
#define PCF8583_H

#include "avr/io.h"

/* Адресный байт для записи */
#define PCF_SLAW 0xA0

/* Адреса регистров */
#define PCF_CS    0x00  // Регистр управления и статуса
#define PCF_HSEC  0x01  // Регистр сотых секунд
#define PCF_SEC   0x02  // Регистр секунд
#define PCF_MIN   0x03  // Регистр минут
#define PCF_HOURS 0x04  // Регистр часов

struct pcf_time
{
    uint8_t seconds;
    uint8_t minutes;
};

/* Функция инициализации часов реального времени PCF8583 */
static inline void pcf_init(void)
{
    uint8_t buf[5];
    buf[0] = (__TIME__[6] - '0') * 16 + (__TIME__[7] - '0');
    buf[1] = (__TIME__[3] - '0') * 16 + (__TIME__[4] - '0');
    buf[2] = (__TIME__[0] - '0') * 16 + (__TIME__[1] - '0');
    buf[3] =
        ((__DATE__[10] - '0') % 4) * 64 +
        ((__DATE__[4] == ' ' ? '0' : __DATE__[4]) - '0') * 16 +
        (__DATE__[5] - '0');
    buf[4] = 4 * 32 + 5;
    twi_write(PCF_SLAW, PCF_SEC, buf, sizeof(buf));

    while (twi_status != TWI_STATUS_TX_COMPLETE)
    {
        ;
    }
    twi_status = TWI_STATUS_READY;
}

/**
 * Функция чтения времени из часов реального времени PCF8583
 * 
 * \param time указатель на структуру, в которую будет записано время
 */
static inline void pcf_read_time(struct pcf_time *time)
{
    twi_read(PCF_SLAW, PCF_SEC, (uint8_t *)time, sizeof(*time));
}

#endif

/* End File */