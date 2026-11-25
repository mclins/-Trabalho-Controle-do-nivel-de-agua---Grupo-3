#ifndef UART_H
#define UART_H

#include <stdint.h>

void uart_init(void);
int  uart_getc(void);                 /* -1 se nao ha byte recebido */
void uart_putc(char c);               /* bloqueia so se o buffer de TX encher */
void uart_puts(const char *s);        /* string na RAM   */
void uart_puts_P(const char *s);      /* string na flash (PSTR) */
void uart_putu(uint32_t v);
void uart_puti(int32_t v);
void uart_putd1(uint32_t decimos);    /* 1234 -> "123.4" */
void uart_puthex8(uint8_t v);
void uart_nl(void);
uint8_t uart_estouro_rx(void);        /* 1 se bytes foram perdidos (e limpa) */

#endif
