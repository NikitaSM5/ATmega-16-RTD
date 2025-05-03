/*  sdcard.c Ц работа с SDSC/SDHC?картами без†FAT
 *  -------------------------------------------------
 *  v2 †(04?ма€?2025)
 *     Х учЄт SDSC†?†SDHC адресации (card_sdhc)
 *     Х sd_write_line() ищет конец лога во†всех†
 *       секторов, а†не только в первом
 */

#include "sdcard.h"
#include "spi.h"
#include <string.h>
#define F_CPU 8000000UL
#include <util/delay.h>

/* ------------ GPIO Ђchip?selectї --------------------------------------- */
static inline void sd_select  (void){ SD_CS_PORT &= ~(1<<SD_CS_PIN); }
static inline void sd_deselect(void){ SD_CS_PORT |=  (1<<SD_CS_PIN); }
static uint32_t buf_sector = 0xFFFFFFFF;

/* ------------ ѕозици€ Ђитератораї дл€ sd_read_line() ------------------- */
static uint32_t it_sector;            /* сектор, в котором сейчас курсор  */
static uint16_t it_off;               /* смещение в этом секторе           */
static uint32_t it_index;             /* номер строки (записи)             */

static uint32_t cur_sector = 1;       /* текущий сектор дл€ записи         */
static uint16_t cur_off    = 0;       /* смещение внутри cur_sector        */

static uint8_t  sec_buf[512];         /* секторный буфер                   */

/* ------------ ѕриватные ------------------------------------------------- */
static uint8_t  card_sdhc = 0;        /* 1 Ц SDHC/SDXC, 0 Ц Ђстара€ї SDSC  */

static uint8_t spi_x(uint8_t v) { return spi_transfer(v); }

/* ---------- ќтправка SD?команды (CMD или ACMD) ------------------------- */
static uint8_t sd_cmd(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    sd_deselect(); spi_x(0xFF);          /* 8†тактов с†CS = 1               */
    sd_select();

    spi_x(cmd | 0x40);
    spi_x(arg >> 24); spi_x(arg >> 16); spi_x(arg >> 8); spi_x(arg);
    spi_x(crc);

    /* ждЄм первый байт ответа (MSB=0) */
    for (uint8_t i = 0; i < 10; i++) {
        uint8_t r = spi_x(0xFF);
        if (!(r & 0x80)) return r;
    }
    return 0xFF;                         /* timeout                         */
}

/* ---------- ожидание токена / готовности ------------------------------- */
static uint8_t wait_token(uint8_t token, uint16_t tout_ms)
{
    while (tout_ms--) {
        uint8_t r = spi_x(0xFF);
        if (r == token) return 0;        /* получили ожидаемый токен        */
        _delay_ms(1);
    }
    return 1;                            /* timeout                         */
}

static uint8_t wait_ready(uint16_t tout_ms)
{
    while (tout_ms--) {
        if (spi_x(0xFF) == 0xFF) return 1;
        _delay_ms(1);
    }
    return 0;                            /* timeout                         */
}

/* ------------ „тение одного сектора ------------------------------------ */
static uint8_t read_sector(uint32_t lba)
{
    uint32_t addr = card_sdhc ? lba : (lba << 9);   /* SDSC: ?512            */
    if (sd_cmd(17, addr, 0xFF))                    /* CMD17 READ_SINGLE      */
        return 1;

    if (wait_token(0xFE, 100))                     /* старт?токен 0xFE       */
        return 2;

    for (uint16_t i = 0; i < 512; i++)
        sec_buf[i] = spi_x(0xFF);
	buf_sector = lba;

    spi_x(0xFF); spi_x(0xFF);                      /* dummy CRC             */
    sd_deselect(); spi_x(0xFF);
    return 0;
}

/* ------------ »тератор чтени€ строк ------------------------------------ */
void sd_iter_reset(void)
{
    it_sector = 1;
    it_off    = 0;
    it_index  = 1;
    read_sector(it_sector);
}

static uint8_t get_char(uint8_t *ch)              /* 1?Ч?ошибка/конец      */
{
    if (it_off == 512) {                           /* дошли до конца буфера */
        if (read_sector(++it_sector)) return 1;
        it_off = 0;
    }
    *ch = sec_buf[it_off++];
    return 0;
}

/* dir?>?0 Ц вперЄд, dir?<?0 Ц назад */
uint8_t sd_read_line(int8_t dir, char *dst, uint8_t dst_sz)
{
    /* ---------- вперЄд (+1) ---------- */
    if (dir > 0) {
        uint8_t c, n = 0;
        while (!get_char(&c) && c != 0xFF) {       /* 0xFF Ц Ђпустої        */
            if (c == '\n') { dst[n] = 0; ++it_index; return 0; }
            if (n < dst_sz - 1) dst[n++] = c;      /* пропускаем '\r'       */
        }
        return 2;                                  /* конец лога           */
    }

    /* ---------- назад (-1) ---------- */

    /* 0. уже в начале файла? */
    if (it_sector == 1 && it_off == 0) return 3;

    /* 1. сместитьс€ на один байт назад                               */
    if (it_off == 0) {                             /* нужен предыдущий сектор */
        if (read_sector(--it_sector)) return 1;
        it_off = 512;
    }
    --it_off;

    /* 2. ищем предыдущий '\n'                                         */
    uint8_t nl_seen = 0;
    while (1) {
        if (sec_buf[it_off] == '\n') {
            if (nl_seen) { ++it_off; break; }      /* нашли начало строки   */
            nl_seen = 1;
        }
        if (it_off == 0) {
            if (it_sector == 1) break;             /* дошли до MBR         */
            if (read_sector(--it_sector)) return 1;
            it_off = 512;
        }
        --it_off;
    }

    --it_index;

    /* 3. а теперь Ц вперЄд, чтобы скопировать строку                  */
    return sd_read_line(+1, dst, dst_sz);
}

/* ====================================================================== */
/*                      »Ќ»÷»јЋ»«ј÷»я  ј–“џ                               */
/* ====================================================================== */
void sd_init(void)
{
    SD_CS_DDR  |= (1 << SD_CS_PIN);
    SD_CS_PORT |= (1 << SD_CS_PIN);

    /* 80?тактов SCK с†CS†=†1, чтобы Ђразбудитьї карту */
    for (uint8_t i = 0; i < 10; i++) spi_x(0xFF);

    /* CMD0 Ц GO_IDLE_STATE */
    if (sd_cmd(0, 0, 0x95) != 0x01) return;

    /* CMD8 Ц проверка SDHC?совместимости */
    uint8_t r = sd_cmd(8, 0x1AA, 0x87);
    if (r == 0x01) { for (uint8_t i = 0; i < 4; i++) spi_x(0xFF); }

    /* ACMD41 Ц инициализаци€ */
    do {
        sd_cmd(55, 0, 0x65);
        r = sd_cmd(41, 0x40000000, 0x77);          /* HCS=1                */
    } while (r != 0);

    /* CMD58 Ц прочитать OCR и узнать SDHC†ли это карта */
    if (sd_cmd(58, 0, 0xFF) == 0) {
        uint32_t ocr = 0;
        for (uint8_t i = 0; i < 4; i++) ocr = (ocr << 8) | spi_x(0xFF);
        card_sdhc = (ocr & (1UL << 30)) ? 1 : 0;   /* бит 30 Ц CCS         */
    }

    sd_deselect(); spi_x(0xFF);
}

/* ====================================================================== */
/*                       «јѕ»—№ ƒјЌЌџ’                                    */
/* ====================================================================== */
static uint8_t write_sector(uint32_t lba)
{
    uint32_t addr = card_sdhc ? lba : (lba << 9);  /* SDSC: ?512          */
    if (!wait_ready(250)) return 4;
    if (sd_cmd(24, addr, 0xFF)) return 1;          /* CMD24 WRITE_BLOCK   */

    spi_x(0xFE);                                   /* старт?токен         */
    for (uint16_t i = 0; i < 512; i++)
        spi_x(sec_buf[i]);

    spi_x(0xFF); spi_x(0xFF);                      /* dummy CRC           */

    uint8_t resp = spi_x(0xFF) & 0x1F;
    if (resp != 0x05) return 2;                    /* data?resp           */

    if (!wait_ready(250)) return 3;                /* ждЄм busy           */
    sd_deselect(); spi_x(0xFF);
	buf_sector = lba;
    return 0;
}

/* ---------- очистить лог (n_sectors с†1?го) ---------------------------- */
void sd_clear_log(uint16_t n_sectors)
{
    memset(sec_buf, 0xFF, 512);

    for (uint16_t i = 0; i < n_sectors; i++) {
        if (write_sector(1 + i)) break;
    }

    /* сброс указателей */
    cur_sector = 1;
    cur_off    = 0;
}

/* ---------- записать строку (0Е64†символа) ---------------------------- */
uint8_t sd_write_line(const char *str)
{
    uint8_t len = strlen(str);
    if (len == 0 || len > 64) return 4;            /* длина неверна       */

    /* первый вызов после сброса: ищем конец лога                        */
    if (cur_off == 0) {
        while (1) {
           if (buf_sector != cur_sector) {            /* буфер Ђчужойї?              */
	           if (read_sector(cur_sector)) return 5; /* подгружаем Ђсвойї сектор    */
           }
            while (cur_off < 512 && sec_buf[cur_off] != 0xFF) cur_off++;
            if (cur_off < 512) break;              /* нашли свободное место*/
            ++cur_sector;                          /* иначе Ц следующий   */
            cur_off = 0;
        }
    }

    /* если не помещаетс€ Ц пишем сектор и переходим к следующему       */
    if (cur_off + len > 512) {
        if (write_sector(cur_sector)) return 6;
        ++cur_sector;
        memset(sec_buf, 0xFF, 512);
        cur_off = 0;
    }

    memcpy(&sec_buf[cur_off], str, len);
    cur_off += len;

    return write_sector(cur_sector);               /* flush              */
}

/* ---------- Ђручнойї flush -------------------------------------------- */
void sd_flush(void)
{
    if (cur_off) write_sector(cur_sector);
}
