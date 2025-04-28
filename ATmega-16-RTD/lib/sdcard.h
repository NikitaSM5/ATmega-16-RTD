#ifndef SDCARD_H
#define SDCARD_H

#include <avr/io.h>
#include <stdint.h>

/* Chip-select */
#define SD_CS_PORT PORTA
#define SD_CS_DDR  DDRA
#define SD_CS_PIN  PA0

void  sd_init(void);                       
uint8_t sd_write_line(const char *str);    
void sd_iter_reset(void);
uint8_t sd_read_line(int8_t dir, char *dst, uint8_t dst_size);
static uint8_t sd_cmd(uint8_t cmd, uint32_t arg, uint8_t crc);
void sd_flush(void);

#endif
