/*
 * adc.c - Aquisicao do sensor de nivel por interrupcao de fim de conversao
 *
 * Referencia AVcc (5 V): o MPX5010 e raciometrico (Vout proporcional a
 * Vs), entao usar a propria alimentacao como referencia cancela, na
 * razao Vout/Vref, as variacoes da fonte.
 *
 * Clock do ADC = 16 MHz / 128 = 125 kHz (dentro de 50-200 kHz exigidos
 * para 10 bits); conversao = 13 ciclos = 104 us.
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "config.h"
#include "adc.h"

static volatile uint16_t soma;              /* soma das ultimas N amostras */
static volatile uint16_t ultima;
static volatile uint8_t  pronto;
static uint16_t janela[ADC_MEDIA_N];
static uint8_t  idx;

void adc_init(void)
{
    ADMUX  = (1 << REFS0) | (ADC_CANAL & 0x0F);          /* AVcc, ajuste a direita */
    DIDR0  = (1 << ADC0D);                               /* desliga buffer digital do pino */
    ADCSRA = (1 << ADEN) | (1 << ADIE) |
             (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); /* prescaler 128 */
    /* conversoes disparadas pela ISR do Timer1 (timer.c) */
}

ISR(ADC_vect)
{
    uint16_t v = ADC;                       /* le ADCL e depois ADCH */
    ultima = v;

    if (!pronto) {
        /* primeira amostra preenche a janela inteira: evita que a media
         * comece em zero e seja interpretada como sensor desconectado */
        for (uint8_t i = 0; i < ADC_MEDIA_N; i++)
            janela[i] = v;
        soma = (uint16_t)(v * ADC_MEDIA_N);
        pronto = 1;
        return;
    }
    /* media movel recursiva: soma += nova - mais_antiga  (O(1)) */
    soma = (uint16_t)(soma + v - janela[idx]);
    janela[idx] = v;
    idx = (uint8_t)((idx + 1) & (ADC_MEDIA_N - 1));
}

uint16_t adc_soma(void)
{
    uint16_t s;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {     /* leitura de 16 bits atomica */
        s = soma;
    }
    return s;
}

uint16_t adc_ultima(void)
{
    uint16_t u;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        u = ultima;
    }
    return u;
}

uint8_t adc_pronto(void)
{
    return pronto;
}
