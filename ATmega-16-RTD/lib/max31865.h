#ifndef MAX31865_H
#define MAX31865_H

#include <avr/io.h>
#define F_CPU 8000000UL
#include <util/delay.h>
#include <stdint.h>

/* ===== Register Map ===== */
#define MAX31865_REG_CONFIG           0x00
#define MAX31865_REG_RTD_MSB          0x01
#define MAX31865_REG_RTD_LSB          0x02
#define MAX31865_REG_HFT_MSB          0x03
#define MAX31865_REG_HFT_LSB          0x04
#define MAX31865_REG_LFT_MSB          0x05
#define MAX31865_REG_LFT_LSB          0x06
#define MAX31865_REG_FAULT_STATUS     0x07

/* ===== Configuration Bits ===== */
#define MAX31865_CFG_BIAS             (1<<7)
#define MAX31865_CFG_MODE_AUTO        (1<<6)
#define MAX31865_CFG_1SHOT            (1<<5)
#define MAX31865_CFG_3WIRE            (1<<4)
#define MAX31865_CFG_FAULT_CYCLE_NONE     (0<<2)
#define MAX31865_CFG_FAULT_CYCLE_AUTO     (1<<2)
#define MAX31865_CFG_FAULT_CYCLE_MAN1     (2<<2)
#define MAX31865_CFG_FAULT_CYCLE_MAN2     (3<<2)
#define MAX31865_CFG_CLEAR_FAULT      (1<<1)
#define MAX31865_CFG_FILT50HZ         (1<<0)  /* 1=50?Hz filter, 0=60?Hz */

/* ===== Conversion Constants ===== */
#define MAX31865_RREF         400.0f   /* Reference resistor (?) */
#define MAX31865_R0           100.0f   /* RTD resistance @ 0?°C (PT100) */
#define MAX31865_A            3.9083e-3f
#define MAX31865_B           -5.775e-7f
#define MAX31865_C           -4.18301e-12f /* only used below 0?°C */

/* ===== Chip?select pin (adjust as needed) ===== */
#define MAX31865_CS_PORT  PORTB
#define MAX31865_CS_DDR   DDRB
#define MAX31865_CS_PIN   PB4      /* SS */

/* ===== Public API ===== */
void max31865_init(uint8_t three_wire, uint8_t filter50Hz);
float max31865_read_temperature(void);
uint16_t max31865_read_raw(void);
uint8_t  max31865_read_fault(void);
void     max31865_clear_fault(void);

#endif // MAX31865_H
//
