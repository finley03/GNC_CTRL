#ifndef UTIL_H
#define UTIL_H

#include "samd21g18a.h"
#include "time.h"

void crc_init();

uint32_t gen_crc32(uint32_t data_addr, uint32_t data_size);

#endif