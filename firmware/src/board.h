/*
 * board.h - Mapeamento de pinos e acesso direto aos registradores de E/S
 *
 * Pino         Funcao                      Arduino Uno
 * PC0/ADC0     sensor de nivel (analogico)  A0
 * PD0/RXD      USART RX                     D0
 * PD1/TXD      USART TX                     D1
 * PD2          botao "reconhecer alarme"    D2   (pull-up interno, ativo em 0)
 * PB0          rele da bomba (via NPN)      D8   (ativo em 1)
 * PB1          buzzer ativo (via NPN)       D9   (ativo em 1)
 * PB2          LED vermelho - alarme        D10
 * PB3          LED verde   - bomba ligada   D11
 * PB5          LED de vida (heartbeat)      D13  (LED da placa)
 */
#ifndef BOARD_H
#define BOARD_H

#include <avr/io.h>

#define PINO_BOMBA      PB0
#define PINO_BUZZER     PB1
#define PINO_LED_ALARME PB2
#define PINO_LED_BOMBA  PB3
#define PINO_LED_VIDA   PB5
#define PINO_BOTAO      PD2

#define SAIDAS_PORTB    ((1 << PINO_BOMBA) | (1 << PINO_BUZZER) | \
                         (1 << PINO_LED_ALARME) | (1 << PINO_LED_BOMBA) | \
                         (1 << PINO_LED_VIDA))

static inline void board_init(void)
{
    PORTB &= (uint8_t)~SAIDAS_PORTB;        /* tudo desligado antes de virar saida */
    DDRB  |= SAIDAS_PORTB;
    DDRD  &= (uint8_t)~(1 << PINO_BOTAO);   /* entrada */
    PORTD |= (1 << PINO_BOTAO);             /* pull-up interno */
}

static inline void saida(uint8_t pino, uint8_t ligado)
{
    if (ligado) PORTB |= (uint8_t)(1 << pino);
    else        PORTB &= (uint8_t)~(1 << pino);
}

static inline void saida_inverte(uint8_t pino)
{
    PINB = (uint8_t)(1 << pino);            /* escrever 1 em PINx inverte PORTx */
}

static inline uint8_t botao_pressionado(void)
{
    return (PIND & (1 << PINO_BOTAO)) == 0;
}

#endif /* BOARD_H */
