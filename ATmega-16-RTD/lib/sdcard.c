
#include "sdcard.h"
#include "spi.h"
#include <string.h>
#define F_CPU 8000000UL
#include <util/delay.h>
static inline void sd_select  (void){ SD_CS_PORT &= ~(1<<SD_CS_PIN); }
static inline void sd_deselect(void){ SD_CS_PORT |=  (1<<SD_CS_PIN); }

static uint32_t it_sector;      
static uint16_t it_off;
static uint32_t it_index;      
static uint32_t cur_sector = 1;    
static uint16_t cur_off    = 0;    
static uint8_t  sec_buf[512];


/* ==== Вспомогательные функции ======================================= */
static uint8_t spi_x(uint8_t v){ return spi_transfer(v); }

/* Отправить команду SD (CMD или ACMD) */
static uint8_t sd_cmd(uint8_t cmd, uint32_t arg, uint8_t crc)
{
	sd_deselect(); spi_x(0xFF);            /* 8 клоков перед CS low         */
	sd_select  ();
	spi_x(cmd | 0x40);
	spi_x(arg>>24); spi_x(arg>>16); spi_x(arg>>8); spi_x(arg);
	spi_x(crc);
	/* ждём первый байт ответа (MSB=0)                           */
	for(uint8_t i=0;i<10;i++){
		uint8_t r = spi_x(0xFF);
		if(!(r & 0x80)) return r;
	}
	return 0xFF;                           /* timeout                       */
}


static uint8_t wait_token(uint8_t token, uint16_t tout_ms)
{
	while (tout_ms--) {
		uint8_t r = spi_x(0xFF);
		if (r == token) return 0;         /* успех                    */
		_delay_ms(1);
	}
	return 1;                             /* timeout                  */
}

/* --- ожидание готовности карты (busy==0xFF) ------------------------ */
static uint8_t wait_ready(uint16_t tout_ms)
{
	while (tout_ms--) {
		if (spi_x(0xFF) == 0xFF) return 1;
		_delay_ms(1);
	}
	return 0;                             /* timeout                  */
}

static uint8_t read_sector(uint32_t lba)
{
	if (sd_cmd(17, lba, 0xFF))            /* CMD17 READ_SINGLE_BLOCK  */
	return 1;

	if (wait_token(0xFE, 100))            /* ждём токен 0xFE (?100 мс)*/
	return 2;                         /* карта зависла            */

	for (uint16_t i = 0; i < 512; i++)
	sec_buf[i] = spi_x(0xFF);

	spi_x(0xFF); spi_x(0xFF);             /* CRC (игнор)              */
	sd_deselect(); spi_x(0xFF);
	return 0;
}

void sd_iter_reset(void)
{
	it_sector = 1;
	it_off    = 0;
	it_index  = 1;
	read_sector(it_sector);         
}

static uint8_t get_char(uint8_t *ch)      /* 1-байтовое чтение */
{
    if (it_off == 512) {                   /* сектор закончился – вперёд   */
        if (read_sector(++it_sector))  return 1;
        it_off = 0;
    }
    *ch = sec_buf[it_off++];
    return 0;
}

uint8_t sd_read_line(int8_t dir, char *dst, uint8_t dst_sz)
{
	/* ---------- ВПЕРЁД (+1) ---------- */
	if (dir > 0) {
		uint8_t c, n = 0;
		while (!get_char(&c) && c != 0xFF) {          /* 0xFF = пусто    */
			if (c == '\n') { dst[n] = 0; ++it_index; return 0; }
			if (n < dst_sz - 1) dst[n++] = c;         /* без '\r'        */
		}
		return 2;                                     /* конец журнала   */
	}

	/* ---------- НАЗАД (-1) ---------- */

	/* 0. Уже в самом начале? */
	if (it_sector == 1 && it_off == 0) return 3;

	/* 1. Шагаем к символу перед курсором                  */
	if (it_off == 0) {                    /* граница сектора       */
		if (read_sector(--it_sector))  return 1;
		it_off = 512;                     /* <-- было 511          */
	}
	--it_off;                             /* теперь точно позади   */

	/* 2. Ищем предыдущее '\n' (при первом же срабатывании nl_seen=1) */
	uint8_t nl_seen = 0;
	while (1) {
		if (sec_buf[it_off] == '\n') {
			if (nl_seen) { ++it_off; break; }  /* нашли начало строки */
			nl_seen = 1;
		}
		if (it_off == 0) {                      /* переход сектора    */
			if (it_sector == 1) break;          /* достигли MBR       */
			if (read_sector(--it_sector)) return 1;
			it_off = 512;                       /* <-- тоже 512       */
		}
		--it_off;
	}

	--it_index;                                 /* скорректировали №  */

	/* 3. Теперь просто прочитать найденную строку вперёд            */
	return sd_read_line(+1, dst, dst_sz);
}



/* ==== Мини-инициализация (только SD/SDHC, без FAT) ==================== */
void sd_init(void)
{

    SD_CS_DDR  |= (1<<SD_CS_PIN);
    SD_CS_PORT |= (1<<SD_CS_PIN);

    /* Десять пустых байт при CS=high – как требует SD-спецификация */
    for(uint8_t i=0;i<10;i++) spi_x(0xFF);

    /* CMD0 – GO_IDLE_STATE */
    if(sd_cmd(0,0,0x95)!=0x01) return;

    /* CMD8 – проверяем SDHC */
    uint8_t r = sd_cmd(8,0x1AA,0x87);
    if(r==0x01){ for(uint8_t i=0;i<4;i++) spi_x(0xFF); }

    /* ACMD41 – инициализация */
    do{
        sd_cmd(55,0,0x65);
        r = sd_cmd(41,0x40000000,0x77);
    }while(r!=0);

    sd_deselect(); spi_x(0xFF);
}

/* ==== Простейший текстовый лог ======================================= */
/* Один сектор = 512 байт => поместится 25 строк вида "hh:mm:ss,+123.4\r\n"
 * Мы храним указатель offset (0–511). При переполнении – следующий сектор.
 * Текущий сектор в RAM не держим, а читаем-правим-записываем (медленно, но
 * логируем всего раз в час). */


static uint8_t write_sector(uint32_t lba)
{
	if (!wait_ready(250)) return 4;       /* карта ещё занята         */
	if (sd_cmd(24, lba, 0xFF)) return 1;  /* CMD24 WRITE_BLOCK        */

	spi_x(0xFE);                          /* токен начала             */
	for (uint16_t i = 0; i < 512; i++)
	spi_x(sec_buf[i]);

	spi_x(0xFF); spi_x(0xFF);             /* CRC-заглушка             */

	uint8_t resp = spi_x(0xFF) & 0x1F;
	if (resp != 0x05) return 2;           /* карта отказалась         */

	if (!wait_ready(250)) return 3;       /* слишком долго busy       */
	sd_deselect(); spi_x(0xFF);
	return 0;
}

void sd_clear_log(uint16_t n_sectors)
{
	memset(sec_buf, 0xFF, 512);        /* шаблон «пустой сектор» */

	for(uint16_t i = 0; i < n_sectors; i++)
	{
		if (write_sector(1 + i))       /* сектор 0 — MBR, начнём с 1 */
		break;                     /* ошибка — выходим */
	}
	/* сбрасываем указатель журнала */
	cur_sector = 1;
	cur_off    = 0;
}

uint8_t sd_write_line(const char *str)
{
	uint8_t len = strlen(str);
	if (len == 0 || len > 64) return 4;       /* защита                 */

	if (cur_off == 0) {                       /* свежий сектор?         */
		if (read_sector(cur_sector)) return 5;
		while (cur_off < 512 && sec_buf[cur_off] != 0xFF) cur_off++;
	}

	if (cur_off + len > 512) {                /*  >  (раньше ?)         */
		if (write_sector(cur_sector)) return 6;
		cur_sector++;
		memset(sec_buf, 0xFF, 512);
		cur_off = 0;
	}

	memcpy(&sec_buf[cur_off], str, len);
	cur_off += len;

	return write_sector(cur_sector);          /* каждый раз flush       */
}

void sd_flush(void)
{
	if (cur_off) write_sector(cur_sector);
}
