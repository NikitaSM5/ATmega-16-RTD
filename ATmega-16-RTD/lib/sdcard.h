#ifndef SDCARD_H
#define SDCARD_H

#include <avr/io.h>
#include <stdint.h>

/* Chip-select */
#define SD_CS_PORT PORTC
#define SD_CS_DDR  DDRC
#define SD_CS_PIN  PC4

void  sd_init(void);                       
uint8_t sd_write_line(const char *str);    
void sd_iter_reset(void);
uint8_t sd_read_line(int8_t dir, char *dst, uint8_t dst_size);
void sd_flush(void);
uint32_t sd_get_entry_count(void);
uint8_t sd_read_line_at(uint32_t line_num, char *dst, uint8_t dst_sz);
void sd_iter_to_end(void);

#endif
