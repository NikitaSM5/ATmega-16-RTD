/*
 * sdcard.c Ч RAW SD logger, 1?sector?per?line edition
 *
 * ѕишем дл€ ATmega16 @†8?MHz, SPI?режим 1.
 *  ажда€ строка измерени€ занимает ровно один сектор (512†байт):
 *   <ASCII?строка> '\n' [0xFF Е 0xFF]
 * Ёто упрощает код и убирает расчЄт смещений.
 */

#include "sdcard.h"
#include "spi.h"
#include <avr/eeprom.h>
#include <string.h>
#define F_CPU 8000000UL
#include <util/delay.h>

/* -------------------------- SPI helpers -------------------------------- */
static inline void sd_select(void)   { SD_CS_PORT &= ~(1 << SD_CS_PIN); }
static inline void sd_deselect(void) { SD_CS_PORT |=  (1 << SD_CS_PIN); }
static uint8_t spi_x(uint8_t v)      { return spi_transfer(v); }

/* ------------------------- Global state -------------------------------- */
static uint8_t  sec_buf[512];            /* RAM?буфер сектора            */
static uint8_t  card_sdhc   = 0;         /* 0†Ц†SDSC,†1†Ц†SDHC           */
static uint32_t next_sector = 1;         /* куда писать следующую строку */
static uint32_t it_sector;               /* курсор при чтении            */
uint32_t EEMEM eeprom_next_sector;
/* ====================================================================== */
static uint8_t sd_cmd(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    sd_deselect(); spi_x(0xFF);          /* 8†такт SCK с CS=1 */
    sd_select();

    spi_x(cmd | 0x40);
    spi_x(arg >> 24); spi_x(arg >> 16); spi_x(arg >> 8); spi_x(arg);
    spi_x(crc);

    for (uint8_t i = 0; i < 10; i++) {
        uint8_t r = spi_x(0xFF);
        if (!(r & 0x80)) return r;
    }
    return 0xFF;                         /* timeout */
}

static uint8_t wait_token(uint8_t token, uint16_t tout_ms)
{
    while (tout_ms--) {
        if (spi_x(0xFF) == token) return 0;
        _delay_ms(1);
    }
    return 1;
}

static uint8_t wait_ready(uint16_t tout_ms)
{
    while (tout_ms--) {
        if (spi_x(0xFF) == 0xFF) return 0;
        _delay_ms(1);
    }
    return 1;
}

/* ------------------------- Low?level R/W -------------------------------- */
static uint8_t read_sector(uint32_t lba)
{
    if (lba == 0) return 1;              /* защита MBR */

    uint32_t addr = card_sdhc ? lba : (lba << 9);

    if (sd_cmd(17, addr, 0xFF))  return 2;   /* READ_SINGLE_BLOCK */
    if (wait_token(0xFE, 100))   return 3;   /* data token */

    for (uint16_t i = 0; i < 512; i++)
        sec_buf[i] = spi_x(0xFF);

    spi_x(0xFF); spi_x(0xFF);            /* dummy CRC */
    sd_deselect(); spi_x(0xFF);
    return 0;
}

uint32_t sd_get_entry_count(void)
{
	return (next_sector > 1) ? (next_sector - 1) : 0;
}

static uint8_t write_sector(uint32_t lba)
{
    if (lba == 0) return 5;              /* защита MBR */

    uint32_t addr = card_sdhc ? lba : (lba << 9);

    if (wait_ready(250)) return 6;
    if (sd_cmd(24, addr, 0xFF)) return 1;    /* WRITE_BLOCK */

    spi_x(0xFE);                          /* start token */
    for (uint16_t i = 0; i < 512; i++)
        spi_x(sec_buf[i]);
    spi_x(0xFF); spi_x(0xFF);             /* dummy CRC */

    uint8_t resp = spi_x(0xFF) & 0x1F;
    if (resp != 0x05) return 2;           /* data?resp error */

    if (wait_ready(250)) return 3;        /* busy timeout */
    sd_deselect(); spi_x(0xFF);
    return 0;
}

/* ------------------------- Tail detector -------------------------------- */
static void find_tail(void)
{
   // uint32_t s = 1;
   // while (1) {
   //     if (read_sector(s)) break;        /* ошибка чтени€†Ц хвост=1 */
   //     if (sec_buf[0] == 0xFF) break;    /* пустой сектор ? хвост   */
   //     ++s;
   // }
   // next_sector = s;
   next_sector = 1;
}

/* ====================================================================== */
void sd_init(void)
{
    SD_CS_DDR  |= (1 << SD_CS_PIN);
    SD_CS_PORT |= (1 << SD_CS_PIN);

    for (uint8_t i = 0; i < 10; i++) spi_x(0xFF); /* 80†тактов на пробуждение */

    if (sd_cmd(0, 0, 0x95) != 0x01) return;       /* CMD0 GO_IDLE */

    uint8_t r = sd_cmd(8, 0x1AA, 0x87);           /* CMD8 */
    if (r == 0x01) { for (uint8_t i = 0; i < 4; i++) spi_x(0xFF); }

    do {                                          /* ACMD41 */
        sd_cmd(55, 0, 0x65);
        r = sd_cmd(41, 0x40000000, 0x77);
    } while (r != 0);

    if (sd_cmd(58, 0, 0xFF) == 0) {               /* CMD58 OCR */
        uint32_t ocr = 0;
        for (uint8_t i = 0; i < 4; i++) ocr = (ocr << 8) | spi_x(0xFF);
        card_sdhc = (ocr & (1UL << 30)) ? 1 : 0;
    }

    sd_deselect(); spi_x(0xFF);
    uint32_t stored = eeprom_read_dword(&eeprom_next_sector);
    if (stored >= 1 && stored <  0xFFFFFFFF) {
	    next_sector = stored;
	    } else {
	    find_tail();
    }
}

void sd_clear_log(uint16_t n_sectors)
{
    memset(sec_buf, 0xFF, 512);
    for (uint16_t i = 0; i < n_sectors; i++)
        write_sector(1 + i);
    next_sector = 1;
}

/* --------------------------- WRITE line -------------------------------- */
uint8_t sd_write_line(const char *str)
{
    uint16_t len = strlen(str);
    if (!len || len > 510) return 4;      /* 0†/ слишком длинно */

    memset(sec_buf, 0xFF, 512);
    memcpy(sec_buf, str, len);
    sec_buf[len] = '\n';

    uint8_t res = write_sector(next_sector);
    if (res == 0) {
	    ++next_sector;
	    eeprom_update_dword(&eeprom_next_sector, next_sector);
    }
    return res;
}

void sd_flush(void) { /* no?op: всегда пишем полный сектор */ }

/* --------------------------- READ iterator ------------------------------ */
void sd_iter_reset(void) { it_sector = 1; }

static uint8_t copy_line(char *dst, uint8_t dst_sz)
{
    uint16_t i = 0;
    while (i < dst_sz - 1 && i < 512 && sec_buf[i] != 0xFF && sec_buf[i] != '\n')
        {
			dst[i] = sec_buf[i];
			i++;
		}
    dst[i] = 0;
    return 0;
}

uint8_t sd_read_line(int8_t dir, char *dst, uint8_t dst_sz)
{
    /* -------- вперед -------- */
    if (dir > 0) {
        if (read_sector(it_sector)) return 1;
        if (sec_buf[0] == 0xFF)     return 2; /* конец журнала */
        uint8_t r = copy_line(dst, dst_sz);
        ++it_sector;
        return r;
    }

    /* -------- назад --------- */
    if (it_sector <= 1) return 3;          /* уже в начале */

    do {
        --it_sector;
        if (read_sector(it_sector)) return 1;
    } while (sec_buf[0] == 0xFF && it_sector > 1);

    return copy_line(dst, dst_sz);
}

uint8_t sd_read_line_at(uint32_t line_num, char *dst, uint8_t dst_sz)
{
	// допустимый диапазон: [1, next_sector-1]
	if (line_num < 1 || line_num >= next_sector)
	return 1;

	// читаем нужный сектор
	if (read_sector(line_num))
	return 2;

	// копируем строку из сектора в dst (останавливаемс€ на '\n' или 0xFF)
	return copy_line(dst, dst_sz);
}

void sd_iter_to_end(void)
{
	it_sector = next_sector;
}