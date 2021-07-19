#ifndef UART_H
#define UART_H

#include "util.h"

#define SERIAL_REG SERCOM2->USART.DATA.reg
#define SERIAL_RX_FLAG SERCOM2->USART.INTFLAG.bit.RXC

#define WIRELESS_REG SERCOM1->USART.DATA.reg
#define WIRELESS_RX_FLAG SERCOM1->USART.INTFLAG.bit.RXC

// USB serial connection
// SERCOM2

void serial_init();
void serial_flush();
void serial_read(uint8_t* addr, uint32_t n);
void serial_send(uint8_t data);
void serial_stream(uint8_t* addr, uint32_t nr_bytes);
void serial_print(char *addr);

// wireless connection
// SERCOM1
void wireless_init();
void wireless_flush();
void wireless_read(uint8_t* addr, uint32_t n);
void wireless_send(uint8_t data);
void wireless_stream(uint8_t* addr, uint32_t nr_bytes);
void wireless_print(char *addr);

// connection to navigation computer
// SERCOM0
void nav_uart_init();
void nav_flush();
void nav_uart_send(uint8_t data);
void nav_read(uint8_t* addr, uint32_t n);
void nav_stream(uint8_t* addr, uint32_t nr_bytes);


#endif