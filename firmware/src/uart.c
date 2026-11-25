/*
 * uart.c - USART0 com recepcao e transmissao por interrupcao
 *
 * RX: a ISR USART_RX_vect guarda cada byte num buffer circular; o laco
 *     principal monta as linhas de comando sem nunca esperar pela serial.
 * TX: uart_putc so copia o byte para o buffer circular e habilita a
 *     interrupcao "registrador de dados vazio" (UDRE). A ISR envia um
 *     byte por vez e se desabilita quando o buffer esvazia. Assim, uma
 *     resposta longa nao atrasa a aquisicao nem o controle.
 *
 * Buffers produtor/consumidor de um so produtor e um so consumidor:
 * cada indice e escrito por um unico contexto e tem 8 bits (acesso
 * atomico no AVR), dispensando secoes criticas.
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include "config.h"
#include "uart.h"

#define UBRR_VALOR  ((F_CPU / (8UL * UART_BAUD)) - 1UL)   /* U2X = 1 -> 51 */

static volatile uint8_t rx_buf[UART_RX_TAM];
static volatile uint8_t rx_ini, rx_fim;
static volatile uint8_t rx_estouro;

static volatile uint8_t tx_buf[UART_TX_TAM];
static volatile uint8_t tx_ini, tx_fim;    /* 8 bits: volta sozinho em 256 */

void uart_init(void)
{
    UBRR0H = (uint8_t)(UBRR_VALOR >> 8);
    UBRR0L = (uint8_t)UBRR_VALOR;
    UCSR0A = (1 << U2X0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);               /* 8N1 */
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
}

ISR(USART_RX_vect)
{
    uint8_t d = UDR0;
    uint8_t prox = (uint8_t)((rx_fim + 1u) & (UART_RX_TAM - 1u));
    if (prox != rx_ini) {
        rx_buf[rx_fim] = d;
        rx_fim = prox;
    } else {
        rx_estouro = 1;
    }
}

ISR(USART_UDRE_vect)
{
    if (tx_ini != tx_fim) {
        UDR0 = tx_buf[tx_ini];
        tx_ini++;
    } else {
        UCSR0B &= (uint8_t)~(1 << UDRIE0);  /* nada a enviar */
    }
}

int uart_getc(void)
{
    uint8_t d;
    if (rx_ini == rx_fim)
        return -1;
    d = rx_buf[rx_ini];
    rx_ini = (uint8_t)((rx_ini + 1u) & (UART_RX_TAM - 1u));
    return d;
}

void uart_putc(char c)
{
    uint8_t prox = (uint8_t)(tx_fim + 1u);
    while (prox == tx_ini)
        ;                                   /* buffer cheio: a ISR esvazia */
    tx_buf[tx_fim] = (uint8_t)c;
    tx_fim = prox;
    UCSR0B |= (1 << UDRIE0);
}

uint8_t uart_estouro_rx(void)
{
    uint8_t e = rx_estouro;
    rx_estouro = 0;
    return e;
}
