#ifndef UART_H
#define UART_H

#include "samd21g18a.h"
#include "time.h"

#define SERIAL_REG SERCOM2->USART.DATA.reg
#define SERIAL_RX_FLAG SERCOM2->USART.INTFLAG.bit.RXC

// USB serial connection
// SERCOM2

void serial_init();
void serial_flush();
void serial_read(uint8_t* addr, uint32_t n);
void serial_send(uint8_t data);
void serial_stream(uint8_t* addr, uint32_t nr_bytes);
void serial_print(char *addr);

void nav_uart_init();
void nav_flush();
void nav_uart_send(uint8_t data);
void nav_read(uint8_t* addr, uint32_t n);
void nav_stream(uint8_t* addr, uint32_t nr_bytes);


#endif