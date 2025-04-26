#include <avr/pgmspace.h>
#include "sevseg.h"

#define DATA_PORT PORTC  // Порт данных к которому подключены
#define DATA_DDR  DDRC   // семисегментные индикаторы.

#define CTRL_PORT PORTB  // Управляющий порт к которому подключены общие
#define CTRL_DDR  DDRB   // катоды/аноды семисегментных индикаторов
#define CTRL_PIN3 3      // Номер вывода общего а/к четвертого индикатора
#define CTRL_PIN2 2      // Номер вывода общего а/к третьего индикатора
#define CTRL_PIN1 1      // Номер вывода общего а/к второго индикатора
#define CTRL_PIN0 0      // Номер вывода общего а/к первого индикатора

#define CTRL_PIN_MASK ((1 << CTRL_PIN3) | (1 << CTRL_PIN2) | \
                       (1 << CTRL_PIN1) | (1 << CTRL_PIN0))

const uint8_t ctrl_pins[4] PROGMEM = {
    (1 << CTRL_PIN0),
    (1 << CTRL_PIN1),
    (1 << CTRL_PIN2),
    (1 << CTRL_PIN3),
};

/* Таблица перекодировки символов в памяти программ */
const uint8_t dec2sevseg[10] PROGMEM = {
    0b11111100,  // код "0"
    0b01100000,  // код "1"
    0b11011010,  // код "2"
    0b11110010,  // код "3"
    0b01100110,  // код "4"
    0b10110110,  // код "5"
    0b10111110,  // код "6"
    0b11100000,  // код "7"
    0b11111110,  // код "8"
    0b11110110   // код "9"
};

/**
 * Функция инициализации 4-х семисегментных индикаторов
 * Выводы данных подключены к DATA_PORT
 * Выводы управления подключены к CTRL_PORT
 */
void sevseg_init(void)
{
    /* Выводы данных и управления - выходы */
    DATA_DDR = 0xFF;
    DATA_PORT = 0x00;
    CTRL_DDR |= CTRL_PIN_MASK;
    CTRL_PORT &= ~CTRL_PIN_MASK;

    // TODO проверить нужна ли данная строчка
    CTRL_PORT |= (1 << CTRL_PIN0);
}

/**
 * Функция перевода 2-х байтного числа в 4-х разрядное двоично-десятичное число.
 * 
 * \param value исходное 2-х байтное число
 * \param digits указатель на 4-х байтный буфер, в который записываются разряды
 *               двоично-десятичного числа.
 */
void sevseg_bin2bcd(uint16_t value, uint8_t *digits)
{
    /* Установка указателя на последний (старший) разряд */
    digits += 4 - 1;

    /* Вычисление разрядов числа, начиная со старшего */
    *digits-- = value / 1000;
    value %= 1000;
    *digits-- = value / 100;
    value %= 100;
    *digits-- = value / 10;
    value %= 10;
    *digits-- = value / 1;
    value %= 1;  // ?
}

/**
 * Функция циклически инкрементирует номер отображаемого разряда
 * 4-х разрядного числа
 * 
 * \return номер следующего отображаемого разряда
 */
static uint8_t inc_number_digit(void)
{
    static uint8_t num_dig = 0;
    num_dig = (num_dig + 1) & 0x03;
    return num_dig;
}

/**
 * Функция отображения разряда 4-х разрядного числа на семисегментном индикаторе.
 * Для преобразования разряда числа в код семисегментного индикатора используется
 * таблица перекодировки символов dec2sevseg в памяти программ.
 * 
 * \param digits указатель на 4-х байтное двоично-десятичное число
 * \param num_dig номер отображаемого разряда
 */
static void display_digit(const uint8_t *digits, uint8_t num_dig)
{
    /* Выключение отображения на семисегментных индикаторах */
    DATA_PORT = 0x00;
    CTRL_PORT &= ~CTRL_PIN_MASK;

    /* Включение индикатора и вывод соответствующего символа */
    CTRL_PORT |= pgm_read_byte(&ctrl_pins[num_dig]);
    uint8_t digit = digits[num_dig];
    uint8_t symbol = pgm_read_byte(&dec2sevseg[digit]);
    DATA_PORT = symbol;
}

/**
 * Функция отображения 4-х разрядного двоично-десятичного числа на
 * 4-х семисегментных индикаторах.
 * При каждом вызове отображается один разряд, поэтому для отображения всего
 * числа, функцию необходимо переодически вызывать.
 * 
 * \param digits указатель на 4-х байтное двоично-десятичное число
 */
void sevseg_display_process(const uint8_t *digits)
{
    uint8_t num_dig = inc_number_digit();
    display_digit(digits, num_dig);
}

/**
 * Функция выключения семисегментных индикаторов
 */
void sevseg_off(void)
{
    DATA_PORT = 0x00;
}

/* End File */