/*
 * config.h - Constantes de compilacao do sistema de controle de nivel
 *
 * ENGC50 - Sistemas Microprocessados - UFBA - 2026.2
 * Problema 2: Controle inteligente de nivel de agua (ATmega328P)
 */
#ifndef CONFIG_H
#define CONFIG_H

#define FW_VERSAO       "1.0"

#ifndef F_CPU
#define F_CPU           16000000UL      /* cristal de 16 MHz (Arduino Uno) */
#endif

/* ---------------- Comunicacao serial ---------------- */
#define UART_BAUD       38400UL         /* 8N1, modo U2X (erro de 0,2 %) */
#define UART_RX_TAM     64u             /* potencia de 2 */
#define UART_TX_TAM     256u            /* indice de 8 bits: volta sozinho */

/* ---------------- Base de tempo ---------------- */
#define TICK_HZ         100u            /* Timer1 em CTC: interrupcao a cada 10 ms */
#define PASSO_HZ        10u             /* logica de controle a cada 100 ms */
#define TICKS_POR_PASSO (TICK_HZ / PASSO_HZ)
#define PASSOS_POR_S    PASSO_HZ

/* ---------------- Aquisicao ---------------- */
#define ADC_CANAL       0u              /* PC0 / ADC0 (pino A0 do Uno) */
#define ADC_MEDIA_LOG2  5u              /* media movel de 2^5 = 32 amostras */
#define ADC_MEDIA_N     (1u << ADC_MEDIA_LOG2)

/* ---------------- Botao ---------------- */
#define BOTAO_DEBOUNCE  5u              /* 5 x 10 ms = 50 ms estavel */

#endif /* CONFIG_H */
