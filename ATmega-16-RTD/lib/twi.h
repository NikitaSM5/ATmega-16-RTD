#ifndef TWI_H
#define TWI_H

#include <avr/io.h>

enum twi_status
{
    TWI_STATUS_NOT_INIT = 0,
    TWI_STATUS_READY,
    TWI_STATUS_TX_BUSY,
    TWI_STATUS_TX_COMPLETE,
    TWI_STATUS_RX_BUSY,
    TWI_STATUS_RX_COMPLETE,
    TWI_STATUS_ERROR,
};

extern volatile enum twi_status twi_status;

void twi_init(void);
void twi_write(uint8_t slaw, uint8_t reg_addr, uint8_t *buf, uint8_t size);
void twi_read(uint8_t slaw, uint8_t reg_addr, uint8_t *buf, uint8_t size);

#endif

/* End File */