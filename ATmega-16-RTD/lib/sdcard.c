
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


/* ==== ��������������� ������� ======================================= */
static uint8_t spi_x(uint8_t v){ return spi_transfer(v); }

/* ��������� ������� SD (CMD ��� ACMD) */
static uint8_t sd_cmd(uint8_t cmd, uint32_t arg, uint8_t crc)
{
	sd_deselect(); spi_x(0xFF);            /* 8 ������ ����� CS low         */
	sd_select  ();
	spi_x(cmd | 0x40);
	spi_x(arg>>24); spi_x(arg>>16); spi_x(arg>>8); spi_x(arg);
	spi_x(crc);
	/* ��� ������ ���� ������ (MSB=0)                           */
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
		if (r == token) return 0;         /* �����                    */
		_delay_ms(1);
	}
	return 1;                             /* timeout                  */
}

/* --- �������� ���������� ����� (busy==0xFF) ------------------------ */
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

	if (wait_token(0xFE, 100))            /* ��� ����� 0xFE (?100 ��)*/
	return 2;                         /* ����� �������            */

	for (uint16_t i = 0; i < 512; i++)
	sec_buf[i] = spi_x(0xFF);

	spi_x(0xFF); spi_x(0xFF);             /* CRC (�����)              */
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

static uint8_t get_char(uint8_t *ch)      /* 1-�������� ������ */
{
    if (it_off == 512) {                   /* ������ ���������� � �����   */
        if (read_sector(++it_sector))  return 1;
        it_off = 0;
    }
    *ch = sec_buf[it_off++];
    return 0;
}

uint8_t sd_read_line(int8_t dir, char *dst, uint8_t dst_sz)
{
	/* ---------- ���Ш� (+1) ---------- */
	if (dir > 0) {
		uint8_t c, n = 0;
		while (!get_char(&c) && c != 0xFF) {          /* 0xFF = �����    */
			if (c == '\n') { dst[n] = 0; ++it_index; return 0; }
			if (n < dst_sz - 1) dst[n++] = c;         /* ��� '\r'        */
		}
		return 2;                                     /* ����� �������   */
	}

	/* ---------- ����� (-1) ---------- */

	/* 0. ��� � ����� ������? */
	if (it_sector == 1 && it_off == 0) return 3;

	/* 1. ������ � ������� ����� ��������                  */
	if (it_off == 0) {                    /* ������� �������       */
		if (read_sector(--it_sector))  return 1;
		it_off = 512;                     /* <-- ���� 511          */
	}
	--it_off;                             /* ������ ����� ������   */

	/* 2. ���� ���������� '\n' (��� ������ �� ������������ nl_seen=1) */
	uint8_t nl_seen = 0;
	while (1) {
		if (sec_buf[it_off] == '\n') {
			if (nl_seen) { ++it_off; break; }  /* ����� ������ ������ */
			nl_seen = 1;
		}
		if (it_off == 0) {                      /* ������� �������    */
			if (it_sector == 1) break;          /* �������� MBR       */
			if (read_sector(--it_sector)) return 1;
			it_off = 512;                       /* <-- ���� 512       */
		}
		--it_off;
	}

	--it_index;                                 /* ��������������� �  */

	/* 3. ������ ������ ��������� ��������� ������ �����            */
	return sd_read_line(+1, dst, dst_sz);
}



/* ==== ����-������������� (������ SD/SDHC, ��� FAT) ==================== */
void sd_init(void)
{

    SD_CS_DDR  |= (1<<SD_CS_PIN);
    SD_CS_PORT |= (1<<SD_CS_PIN);

    /* ������ ������ ���� ��� CS=high � ��� ������� SD-������������ */
    for(uint8_t i=0;i<10;i++) spi_x(0xFF);

    /* CMD0 � GO_IDLE_STATE */
    if(sd_cmd(0,0,0x95)!=0x01) return;

    /* CMD8 � ��������� SDHC */
    uint8_t r = sd_cmd(8,0x1AA,0x87);
    if(r==0x01){ for(uint8_t i=0;i<4;i++) spi_x(0xFF); }

    /* ACMD41 � ������������� */
    do{
        sd_cmd(55,0,0x65);
        r = sd_cmd(41,0x40000000,0x77);
    }while(r!=0);

    sd_deselect(); spi_x(0xFF);
}

/* ==== ���������� ��������� ��� ======================================= */
/* ���� ������ = 512 ���� => ���������� 25 ����� ���� "hh:mm:ss,+123.4\r\n"
 * �� ������ ��������� offset (0�511). ��� ������������ � ��������� ������.
 * ������� ������ � RAM �� ������, � ������-������-���������� (��������, ��
 * �������� ����� ��� � ���). */


static uint8_t write_sector(uint32_t lba)
{
	if (!wait_ready(250)) return 4;       /* ����� ��� ������         */
	if (sd_cmd(24, lba, 0xFF)) return 1;  /* CMD24 WRITE_BLOCK        */

	spi_x(0xFE);                          /* ����� ������             */
	for (uint16_t i = 0; i < 512; i++)
	spi_x(sec_buf[i]);

	spi_x(0xFF); spi_x(0xFF);             /* CRC-��������             */

	uint8_t resp = spi_x(0xFF) & 0x1F;
	if (resp != 0x05) return 2;           /* ����� ����������         */

	if (!wait_ready(250)) return 3;       /* ������� ����� busy       */
	sd_deselect(); spi_x(0xFF);
	return 0;
}

void sd_clear_log(uint16_t n_sectors)
{
	memset(sec_buf, 0xFF, 512);        /* ������ ������� ������ */

	for(uint16_t i = 0; i < n_sectors; i++)
	{
		if (write_sector(1 + i))       /* ������ 0 � MBR, ����� � 1 */
		break;                     /* ������ � ������� */
	}
	/* ���������� ��������� ������� */
	cur_sector = 1;
	cur_off    = 0;
}

uint8_t sd_write_line(const char *str)
{
	uint8_t len = strlen(str);
	if (len == 0 || len > 64) return 4;       /* ������                 */

	if (cur_off == 0) {                       /* ������ ������?         */
		if (read_sector(cur_sector)) return 5;
		while (cur_off < 512 && sec_buf[cur_off] != 0xFF) cur_off++;
	}

	if (cur_off + len > 512) {                /*  >  (������ ?)         */
		if (write_sector(cur_sector)) return 6;
		cur_sector++;
		memset(sec_buf, 0xFF, 512);
		cur_off = 0;
	}

	memcpy(&sec_buf[cur_off], str, len);
	cur_off += len;

	return write_sector(cur_sector);          /* ������ ��� flush       */
}

void sd_flush(void)
{
	if (cur_off) write_sector(cur_sector);
}
