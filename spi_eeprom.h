#ifndef SPI_EEPROM_H
#define SPI_EEPROM_H

#include "util.h"
#include "spi.h"

void spi_eeprom_init();

void spi_eeprom_write_enable();
void spi_eeprom_write_disable();

uint8_t spi_eeprom_read_status();
void spi_eeprom_write_status(uint8_t data);

uint8_t spi_eeprom_read_byte(uint32_t address);
void spi_eeprom_write_byte(uint32_t address, uint8_t data);

#endif