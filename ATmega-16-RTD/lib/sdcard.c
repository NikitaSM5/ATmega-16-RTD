#include "sdcard.h"
#include "spi.h"
#include <string.h>
#define F_CPU 8000000UL
#include <util/delay.h>
static inline void sd_select  (void){ SD_CS_PORT &= ~(1<<SD_CS_PIN); }
static inline void sd_deselect(void){ SD_CS_PORT |=  (1<<SD_CS_PIN); }

/* ==== ¬спомогательные функции ======================================= */
static uint8_t spi_x(uint8_t v){ return spi_transfer(v); }

static uint8_t wait_ready(uint16_t tout_ms)
{
    while(tout_ms--){
        if (spi_x(0xFF)==0xFF) return 1;
        _delay_ms(1);
    }
    return 0;
}

/* ќтправить команду SD (CMD или ACMD) */
static uint8_t sd_cmd(uint8_t cmd, uint32_t arg, uint8_t crc)
{
    sd_deselect(); spi_x(0xFF);            /* 8 клоков перед CS low         */
    sd_select  ();
    spi_x(cmd | 0x40);
    spi_x(arg>>24); spi_x(arg>>16); spi_x(arg>>8); spi_x(arg);
    spi_x(crc);
    /* ждЄм первый байт ответа (MSB=0)                           */
    for(uint8_t i=0;i<10;i++){
        uint8_t r = spi_x(0xFF);
        if(!(r & 0x80)) return r;
    }
    return 0xFF;                           /* timeout                       */
}

/* ==== ћини-инициализаци€ (только SD/SDHC, без FAT) ==================== */
void sd_init(void)
{
    /* CS pin как output = inactive high */
    SD_CS_DDR  |= (1<<SD_CS_PIN);
    SD_CS_PORT |= (1<<SD_CS_PIN);

    /* ƒес€ть пустых байт при CS=high Ц как требует SD-спецификаци€ */
    for(uint8_t i=0;i<10;i++) spi_x(0xFF);

    /* CMD0 Ц GO_IDLE_STATE */
    if(sd_cmd(0,0,0x95)!=0x01) return;

    /* CMD8 Ц провер€ем SDHC */
    uint8_t r = sd_cmd(8,0x1AA,0x87);
    if(r==0x01){ for(uint8_t i=0;i<4;i++) spi_x(0xFF); }

    /* ACMD41 Ц инициализаци€ */
    do{
        sd_cmd(55,0,0x65);
        r = sd_cmd(41,0x40000000,0x77);
    }while(r!=0);

    sd_deselect(); spi_x(0xFF);
}

/* ==== ѕростейший текстовый лог ======================================= */
/* ќдин сектор = 512 байт => поместитс€ 25 строк вида "hh:mm:ss,+123.4\r\n"
 * ћы храним указатель offset (0Ц511). ѕри переполнении Ц следующий сектор.
 * “екущий сектор в RAM не держим, а читаем-правим-записываем (медленно, но
 * логируем всего раз в час). */

static uint32_t cur_sector = 1;     /* сектор 1 Ц свободен после MBR   */
static uint16_t cur_off    = 0;     /* смещение внутри сектора         */
static uint8_t  sec_buf[512];

static uint8_t read_sector(uint32_t lba)
{
    if(sd_cmd(17,lba,0xFF)) return 1;
    /* ждЄм токен 0xFE */
    while(spi_x(0xFF)==0xFF);
    for(uint16_t i=0;i<512;i++) sec_buf[i]=spi_x(0xFF);
    spi_x(0xFF); spi_x(0xFF);         /* CRC */
    sd_deselect(); spi_x(0xFF);
    return 0;
}

static uint8_t write_sector(uint32_t lba)
{
    if(sd_cmd(24,lba,0xFF)) return 1;
    spi_x(0xFE);                      /* токен начала */
    for(uint16_t i=0;i<512;i++) spi_x(sec_buf[i]);
    spi_x(0xFF); spi_x(0xFF);         /* CRC */
    uint8_t resp = spi_x(0xFF) & 0x1F;
    if(resp!=0x05) return 2;
    if(!wait_ready(250)) return 3;
    sd_deselect(); spi_x(0xFF);
    return 0;
}

void sd_clear_log(uint16_t n_sectors)
{
	memset(sec_buf, 0xFF, 512);        /* шаблон Ђпустой секторї */

	for(uint16_t i = 0; i < n_sectors; i++)
	{
		if (write_sector(1 + i))       /* сектор 0 Ч MBR, начнЄм с 1 */
		break;                     /* ошибка Ч выходим */
	}
	/* сбрасываем указатель журнала */
	cur_sector = 1;
	cur_off    = 0;
}

uint8_t sd_write_line(const char *str)
{
    uint8_t len = strlen(str);
    if(len==0 || len>64) return 4;        /* safety */

    if(cur_off==0){                       /* читаем свежий сектор */
        if(read_sector(cur_sector)) return 5;
        /* ищем конец уже записанного текста (0x00/0xFF) */
        while(cur_off<512 && sec_buf[cur_off]!=0xFF) cur_off++;
    }

    /* если места не хватит Ц сектор вперЄд */
    if(cur_off + len >= 512){
        if(write_sector(cur_sector)) return 6;
        cur_sector++;
        memset(sec_buf,0xFF,512);
        cur_off = 0;
    }
    memcpy(&sec_buf[cur_off],str,len);
    cur_off += len;
    /* «аписываем каждый раз Ц лога немного */
    return write_sector(cur_sector);
}
