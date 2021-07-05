#ifndef UTIL_H
#define UTIL_H

#include "samd21g18a.h"
#include "time.h"

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#define LED PORT_PB11

void LED_print_8(uint8_t data);

void crc_init();

#define CRC32_CHECK 0x2144DF1C
uint32_t gen_crc32(uint32_t data_addr, uint32_t data_size);

#endif