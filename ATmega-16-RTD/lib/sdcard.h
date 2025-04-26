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

#endif
