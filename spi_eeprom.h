#ifndef SPI_EEPROM_H
#define SPI_EEPROM_H

#include "util.h"
#include "spi.h"

// memory map
#define EEPROM_PAGE_SIZE 64
// programmed waypoints occupy first 32k of eeprom
#define EEPROM_PATH_START 0x0000
// configurations occupy second 32k of eeprom
#define EEPROM_CONFIG_START 0x1000
// pid values for control loop
#define EEPROM_PID_X 0x1000
#define EEPROM_PID_Y 0x100C
#define EEPROM_PID_Z 0x1018
#define EEPROM_PID_SIZE 12
// 

void spi_eeprom_init();

void spi_eeprom_write_enable();
void spi_eeprom_write_disable();

uint8_t spi_eeprom_read_status();
void spi_eeprom_write_status(uint8_t data);

uint8_t spi_eeprom_read_byte(uint32_t address);
void spi_eeprom_write_byte(uint32_t address, uint8_t data);

void spi_eeprom_read_n(uint32_t address, void* writeback, uint32_t n);
// function is not page safe, but is faster
void spi_eeprom_write_n(uint32_t address, void* data, uint32_t n);
// function is page safe but slower. Handling of write enable is automatic.
void spi_eeprom_write_n_s(uint32_t address, void* data, uint32_t n);


#endif