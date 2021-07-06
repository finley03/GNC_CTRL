#include "spi_eeprom.h"


#define WREN 0b00000110
#define WRDI 0b00000100
#define RDSR 0b00000101
#define WRSR 0b00000001
#define READ 0b00000011
#define WRITE 0b00000010

#define RDY_MASK 0b00000001

#define MAX_ADDR 0x1fff

#define SPI_EEPROM_SS PORT_PA27


void spi_eeprom_init() {
	// setup SS pins
	REG_PORT_OUTSET0 = SPI_EEPROM_SS;
	REG_PORT_DIRSET0 = SPI_EEPROM_SS;
}


void spi_eeprom_write_enable() {
	REG_PORT_OUTCLR0 = SPI_EEPROM_SS;
	spi_command(WREN);
	REG_PORT_OUTSET0 = SPI_EEPROM_SS;
}

void spi_eeprom_write_disable() {
	REG_PORT_OUTCLR0 = SPI_EEPROM_SS;
	spi_command(WRDI);
	REG_PORT_OUTSET0 = SPI_EEPROM_SS;
}


uint8_t spi_eeprom_read_status() {
	REG_PORT_OUTCLR0 = SPI_EEPROM_SS;
	spi_command(RDSR);
	uint8_t out = spi_command(0);
	REG_PORT_OUTSET0 = SPI_EEPROM_SS;
	return out;
}

void spi_eeprom_write_status(uint8_t data) {
	REG_PORT_OUTCLR0 = SPI_EEPROM_SS;
	spi_command(WRSR);
	spi_command(data);
	REG_PORT_OUTSET0 = SPI_EEPROM_SS;
}


uint8_t spi_eeprom_read_byte(uint32_t address) {
	REG_PORT_OUTCLR0 = SPI_EEPROM_SS;
	spi_command(READ);
	spi_command((uint8_t)(address >> 8));
	spi_command((uint8_t)address);
	uint8_t out = spi_command(0);
	REG_PORT_OUTSET0 = SPI_EEPROM_SS;
	return out;
}

void spi_eeprom_write_byte(uint32_t address, uint8_t data) {
	REG_PORT_OUTCLR0 = SPI_EEPROM_SS;
	spi_command(WRITE);
	spi_command((uint8_t)(address >> 8));
	spi_command((uint8_t)address);
	spi_command(data);
	REG_PORT_OUTSET0 = SPI_EEPROM_SS;
	// wait until write cycle is done
	while (spi_eeprom_read_status() & RDY_MASK);
}