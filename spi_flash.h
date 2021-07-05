#ifndef SPI_FLASH_H
#define SPI_FLASH_H

#include "util.h"
#include "spi.h"


void spi_flash_init();

uint8_t spi_flash_checkid();

void spi_flash_write_enable();
void spi_flash_write_disable();

#endif