#ifndef SPI_H
#define SPI_H

#include "samd21g18a.h"
#include "time.h"

// SERCOM3
// PA22 - PA25

void spi_init();

uint8_t spi_command(uint8_t opcode);

#endif