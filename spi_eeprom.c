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

void spi_eeprom_read_n(uint32_t address, void* writeback, uint32_t n) {
	uint8_t* writeback8 = (uint8_t*) writeback;
	REG_PORT_OUTCLR0 = SPI_EEPROM_SS;
	spi_command(READ);
	spi_command((uint8_t)(address >> 8));
	spi_command((uint8_t)address);
	for (uint32_t i = 0; i < n; ++i) {
		writeback8[i] = spi_command(0);
	}
	REG_PORT_OUTSET0 = SPI_EEPROM_SS;
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

void spi_eeprom_write_n(uint32_t address, void* data, uint32_t n) {
	uint8_t* data8 = (uint8_t*) data;
	n = (n <= 64) ? n : 64;
	REG_PORT_OUTCLR0 = SPI_EEPROM_SS;
	spi_command(WRITE);
	spi_command((uint8_t)(address >> 8));
	spi_command((uint8_t)address);
	for (uint32_t i = 0; i < n; ++i) {
		spi_command(data8[i]);
	}
	REG_PORT_OUTSET0 = SPI_EEPROM_SS;
	// wait until write cycle is done
	while (spi_eeprom_read_status() & RDY_MASK);
}

void spi_eeprom_write_n_s(uint32_t address, void* data, uint32_t n) {
	uint8_t* data8 = (uint8_t*) data;
	uint32_t top = address + n - 1;
	uint32_t botton_multiples = address / EEPROM_PAGE_SIZE;
	uint32_t top_multiples = top / EEPROM_PAGE_SIZE;
	uint32_t nr_pages = top_multiples - botton_multiples + 1;
	uint32_t data_index = 0;
	for (uint32_t i = 0; i < nr_pages; ++i) {
		spi_eeprom_write_enable();
		REG_PORT_OUTCLR0 = SPI_EEPROM_SS;
		spi_command(WRITE);
		spi_command((uint8_t)(address >> 8));
		spi_command((uint8_t)address);
		for (uint32_t j = address % EEPROM_PAGE_SIZE; address <= top && j < EEPROM_PAGE_SIZE; ++j) {
			spi_command(data8[data_index++]);
			++address;
		}
		REG_PORT_OUTSET0 = SPI_EEPROM_SS;
		// wait until write cycle is done
		while (spi_eeprom_read_status() & RDY_MASK);
		spi_eeprom_write_disable();
	}
}