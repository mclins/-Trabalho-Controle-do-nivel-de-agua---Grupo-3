/*
 * uart_fmt.c - Formatacao numerica leve (sem printf) sobre uart_putc.
 * Independente de hardware: compilado tambem nos testes no PC.
 */
#include "uart.h"
#include "pgm_compat.h"

void uart_puts(const char *s)
{
    while (*s)
        uart_putc(*s++);
}

void uart_puts_P(const char *s)
{
    char c;
    while ((c = (char)pgm_read_byte(s++)) != 0)
        uart_putc(c);
}

void uart_putu(uint32_t v)
{
    char buf[11];
    uint8_t i = 0;
    do {
        buf[i++] = (char)('0' + (v % 10u));
        v /= 10u;
    } while (v);
    while (i)
        uart_putc(buf[--i]);
}

void uart_puti(int32_t v)
{
    if (v < 0) {
        uart_putc('-');
        uart_putu((uint32_t)(-v));
    } else {
        uart_putu((uint32_t)v);
    }
}

void uart_putd1(uint32_t decimos)
{
    uart_putu(decimos / 10u);
    uart_putc('.');
    uart_putc((char)('0' + decimos % 10u));
}

void uart_puthex8(uint8_t v)
{
    static const char hex[] PROGMEM = "0123456789ABCDEF";
    uart_putc((char)pgm_read_byte(&hex[v >> 4]));
    uart_putc((char)pgm_read_byte(&hex[v & 0x0F]));
}

void uart_nl(void)
{
    uart_putc('\r');
    uart_putc('\n');
}
