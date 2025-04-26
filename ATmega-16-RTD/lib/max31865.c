#include "max31865.h"
#include "spi.h"

static inline void cs_low(void)  { MAX31865_CS_PORT &= ~(1<<MAX31865_CS_PIN); }
static inline void cs_high(void) { MAX31865_CS_PORT |=  (1<<MAX31865_CS_PIN); }

static void write_reg(uint8_t addr, uint8_t val)
{
	cs_low();
	spi_transfer(addr | 0x80);   /* write flag */
	spi_transfer(val);
	cs_high();
}

static uint8_t read_reg(uint8_t addr)
{
	cs_low();
	spi_transfer(addr & 0x7F);
	uint8_t v = spi_transfer(0xFF);
	cs_high();
	return v;
}

/* Read 16‑bit register (RTD value, thresholds). */
static uint16_t read_reg16(uint8_t addr)
{
	cs_low();
	spi_transfer(addr & 0x7F);
	uint8_t msb = spi_transfer(0xFF);
	uint8_t lsb = spi_transfer(0xFF);
	cs_high();
	return ((uint16_t)msb << 8) | lsb;
}

void max31865_init(uint8_t three_wire, uint8_t filter50Hz)
{
	/* Configure CS pin */
	MAX31865_CS_DDR |= (1<<MAX31865_CS_PIN);
	cs_high();

	spi_init(); /* in spi.c */

	uint8_t cfg = 0;
	if (three_wire) cfg |= MAX31865_CFG_3WIRE;
	if (filter50Hz) cfg |= MAX31865_CFG_FILT50HZ;

	write_reg(MAX31865_REG_CONFIG, cfg | MAX31865_CFG_BIAS); /* Enable bias */
	_delay_ms(10);
	/* Start automatic conversions */
	write_reg(MAX31865_REG_CONFIG, cfg | MAX31865_CFG_BIAS | MAX31865_CFG_MODE_AUTO);
}

uint16_t max31865_read_raw(void)
{
	uint16_t v = read_reg16(MAX31865_REG_RTD_MSB);
	return (v >> 1) & 0x7FFF;
}

float max31865_read_temperature(void)
{
	uint16_t raw = max31865_read_raw();
	float Rt = (float)raw * MAX31865_RREF / 32768.0f;

	/* Callendar‑Van Dusen, simplify for >=0 °C */
	float Z = Rt / MAX31865_R0;
	float temp;

	if (Rt >= MAX31865_R0) {
		/* quadratic formula */
		float d = (MAX31865_A*MAX31865_A) - (4*MAX31865_B*(1-Z));
		temp = (-MAX31865_A + sqrtf(d)) / (2*MAX31865_B);
		} else {
		/* Use full equation with C term for <0 °C */
		/* Solve iteratively (Newton) */
		temp = -50.0f; /* initial */
		for (uint8_t i=0;i<6;i++) {
			float f = MAX31865_R0*(1 + MAX31865_A*temp + MAX31865_B*temp*temp + MAX31865_C*(temp-100)*temp*temp*temp) - Rt;
			float df = MAX31865_R0*(MAX31865_A + 2*MAX31865_B*temp + 3*MAX31865_C*temp*temp - 200*MAX31865_C*temp);
			temp -= f/df;
		}
	}
	return temp;
}

uint8_t max31865_read_fault(void)
{
	return read_reg(MAX31865_REG_FAULT_STATUS);
}

void max31865_clear_fault(void)
{
	uint8_t cfg = read_reg(MAX31865_REG_CONFIG);
	cfg &= ~MAX31865_CFG_MODE_AUTO; /* halt conv */
	write_reg(MAX31865_REG_CONFIG, cfg | MAX31865_CFG_CLEAR_FAULT);
	/* restart */
	write_reg(MAX31865_REG_CONFIG, cfg | MAX31865_CFG_BIAS | MAX31865_CFG_MODE_AUTO);
}
//asd