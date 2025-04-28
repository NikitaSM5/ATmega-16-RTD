#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#include "twi.h"

#define LINE_FREQ 100000UL  // Частота обмена в Гц
#define F_CPU 8000000UL
volatile enum twi_status twi_status = TWI_STATUS_NOT_INIT;

static struct twi
{
    uint8_t slaw;
    uint8_t reg_addr;
    uint8_t *data;
    uint8_t count;
} twi;

/**
 * Функция инициализации модуля TWI.
 * По окончанию инициализации устанавливается статус TWI_STATUS_READY.
 * Частота обмена в Гц: LINE_FREQ.
 * Используется прерывание по событиям на шине TWI.
 */
void twi_init(void)
{
    TWBR = (F_CPU / LINE_FREQ - 16) / 2;
    TWCR |= (1 << TWEN) | (1 << TWIE);

    twi_status = TWI_STATUS_READY;
}

/**
 * Функция записи/чтения в/из регистра ведомого устройства
 * 
 * \param slaw адрес ведомого устройства SLA+W
 * \param reg_addr адрес начального регистра для записи/чтения
 * \param buf указатель на буфер для записи/чтения
 * \param size размер буфера для записи/чтения
 */
static inline void twi_txrx(
    uint8_t slaw, uint8_t reg_addr, uint8_t *buf, uint8_t size)
{
    twi.slaw = slaw;
    twi.reg_addr = reg_addr;
    twi.data = buf;
    twi.count = size;

    TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN) | (1<<TWIE);
}

/**
 * Функция записи в регистр ведомого устройства.
 * Во время записи устанавливается статус TWI_STATUS_TX_BUSY, после записи 
 * устанавливается статус TWI_STATUS_TX_COMPLETE.
 * 
 * \param slaw адрес ведомого устройства SLA+W
 * \param reg_addr адрес начального регистра для записи
 * \param buf указатель на буфер для записи
 * \param size размер буфера для записи
 */
void twi_write(uint8_t slaw, uint8_t reg_addr, uint8_t *buf, uint8_t size)
{
    twi_txrx(slaw, reg_addr, buf, size);
    twi_status = TWI_STATUS_TX_BUSY;
}

/**
 * Функция чтения из регистра ведомого устройства.
 * Во время чтения устанавливается статус TWI_STATUS_RX_BUSY, после чтения 
 * устанавливается статус TWI_STATUS_RX_COMPLETE.
 * 
 * \param slaw адрес ведомого устройства SLA+W
 * \param reg_addr адрес начального регистра для чтения
 * \param buf указатель на буфер для чтения
 * \param size размер буфера для чтения
 */
void twi_read(uint8_t slaw, uint8_t reg_addr, uint8_t *buf, uint8_t size)
{
    twi_txrx(slaw, reg_addr, buf, size);
    twi_status = TWI_STATUS_RX_BUSY;
}

/* Конечный автомат линии TWI */

enum line_status
{
    LINE_BUS_ERROR = 0,      // Ошибка на шине в результате некорректного формирования
                             // состояния СТАРТ или СТОП
    LINE_START = 1,          // Сформировано состояние СТАРТ
    LINE_REP_START = 2,      // Сформировано состояние ПОВСТАРТ
    LINE_TX_SLA_ACK = 3,     // Передан пакет SLA+W и принято подтверждение (ACK)
    LINE_TX_SLA_NACK = 4,    // Передан пакет SLA+W, а подтверждение не принято (NACK)
    LINE_TX_DATA_ACK = 5,    // Передан пакет данных и принято подтверждение (ACK)
    LINE_TX_DATA_NACK = 6,   // Передан пакет данных, а подтверждение не принято (NACK)
    LINE_ARB_LOST = 7,       // Потеря приоритета при передаче пакета адреса или данных
    LINE_RX_SLA_ACK = 8,     // Передан пакет SLA+R и принято подтверждение (ACK)
    LINE_RX_SLA_NACK = 9,    // Передан пакет SLA+R и принято неподтверждение (NACK)
    LINE_RX_DATA_ACK = 10,   // Принят байт данных и передано подтверждение (ACK)
    LINE_RX_DATA_NACK = 11,  // Принят байт данных и передано неподтверждение (NACK)
    LINE_CNT
};

static void line_error_stop(void)
{
    TWCR |= (1 << TWSTO);
    twi_status = TWI_STATUS_ERROR;
}

static void line_send_slaw(void)
{
    TWDR = twi.slaw;
    TWCR &= ~(1 << TWSTA);
}

static void line_send_slar(void)
{
    uint8_t twi_slar = twi.slaw | 0x01;
    TWDR = twi_slar;
    TWCR &= ~(1 << TWSTA);
}

static void line_send_reg_addr(void)
{
    TWDR = twi.reg_addr;
}

static void line_tx_data(void)
{
    switch (twi_status)
    {
    case TWI_STATUS_RX_BUSY:
        TWCR |= (1 << TWSTA);
        break;
    case TWI_STATUS_TX_BUSY:
        TWDR = *twi.data++;
        if (twi.count-- == 0)
        {
            TWCR |= (1 << TWSTO);
            twi_status = TWI_STATUS_TX_COMPLETE;
        }
        break;
    default:
        break;
    }
}

static void line_rx_set_ack(void)
{
    if (--twi.count)
    {
        TWCR |= (1 << TWEA);  // ACK
    }
    else
    {
        TWCR &= ~(1 << TWEA);  // NACK
    }
}

static void line_rx_data(void)
{
    *twi.data++ = TWDR;
    line_rx_set_ack();
}

static void line_rx_last_data(void)
{
    *twi.data++ = TWDR;
    TWCR |= (1 << TWSTO);
    twi_status = TWI_STATUS_RX_COMPLETE;
}

typedef void (*vfptr_t)(void);
const vfptr_t line_actions_table[LINE_CNT] PROGMEM = {
    [LINE_BUS_ERROR] = line_error_stop,
    [LINE_START] = line_send_slaw,
    [LINE_REP_START] = line_send_slar,
    [LINE_TX_SLA_ACK] = line_send_reg_addr,
    [LINE_TX_SLA_NACK] = line_error_stop,
    [LINE_TX_DATA_ACK] = line_tx_data,
    [LINE_TX_DATA_NACK] = line_error_stop,
    [LINE_ARB_LOST] = line_error_stop,
    [LINE_RX_SLA_ACK] = line_rx_set_ack,
    [LINE_RX_SLA_NACK] = line_error_stop,
    [LINE_RX_DATA_ACK] = line_rx_data,
    [LINE_RX_DATA_NACK] = line_rx_last_data,
};

/* Прерывание используется для обработки события на линии TWI */
ISR(TWI_vect)
{
    enum line_status status = (TWSR & 0xF8) >> 3;

    vfptr_t func = (vfptr_t)pgm_read_word(&line_actions_table[status]);
    if (func != 0)
    {
        func();
    }
    else
    {
        TWCR |= (1 << TWSTO);
    }

    TWCR |= (1 << TWINT);
}

/* End File */