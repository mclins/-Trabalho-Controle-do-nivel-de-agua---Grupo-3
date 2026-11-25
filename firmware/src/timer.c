/*
 * timer.c - Base de tempo do sistema com o Timer/Counter1 em modo CTC
 *
 * f_tick = F_CPU / (N * (1 + OCR1A)) = 16 MHz / (64 * 2500) = 100 Hz
 *
 * A ISR de 10 ms faz apenas o trabalho que precisa de periodicidade
 * exata: dispara a conversao do ADC, amostra o botao e sinaliza ao laco
 * principal que um passo de controle (100 ms) venceu. O processamento
 * pesado fica fora da interrupcao.
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "config.h"
#include "board.h"
#include "timer.h"

#define TIMER1_PRESC    64u
#define TIMER1_TOPO     ((F_CPU / (TIMER1_PRESC * TICK_HZ)) - 1u)   /* 2499 */

#if TIMER1_TOPO > 65535u
#error "TIMER1_TOPO nao cabe em 16 bits"
#endif

static volatile uint8_t passos_pendentes;
static volatile uint8_t evento_botao;

void timer_init(void)
{
    OCR1A  = TIMER1_TOPO;
    TCCR1A = 0;
    TCCR1B = (1 << WGM12);                  /* CTC, topo = OCR1A */
    TCNT1  = 0;
    TIFR1  = (1 << OCF1A);                  /* limpa flag pendente */
    TIMSK1 = (1 << OCIE1A);                 /* interrupcao de comparacao A */
    TCCR1B |= (1 << CS11) | (1 << CS10);    /* prescaler 64: inicia contagem */
}

ISR(TIMER1_COMPA_vect)
{
    static uint8_t div_passo;
    static uint8_t cont_botao;
    static uint8_t botao_estavel;

    /* 1) Dispara uma conversao do ADC (100 amostras/s).
     * ADIF e "limpo escrevendo 1": um ADCSRA |= (1<<ADSC) poderia apagar
     * uma conversao concluida e ainda nao atendida. Por isso ADIF e
     * escrito com 0 explicitamente. */
    ADCSRA = (uint8_t)((ADCSRA & ~(1 << ADIF)) | (1 << ADSC));

    /* 2) Debounce do botao por integracao: o estado so muda apos
     * BOTAO_DEBOUNCE leituras consecutivas iguais (50 ms). */
    uint8_t lido = botao_pressionado();
    if (lido != botao_estavel) {
        if (++cont_botao >= BOTAO_DEBOUNCE) {
            botao_estavel = lido;
            cont_botao = 0;
            if (lido)
                evento_botao = 1;           /* borda de pressionamento */
        }
    } else {
        cont_botao = 0;
    }

    /* 3) Divisor para o passo de controle de 100 ms */
    if (++div_passo >= TICKS_POR_PASSO) {
        div_passo = 0;
        if (passos_pendentes < 255)
            passos_pendentes++;
    }
}

uint8_t timer_pega_passo(void)
{
    uint8_t tem = 0;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        if (passos_pendentes) {
            passos_pendentes--;
            tem = 1;
        }
    }
    return tem;
}

uint8_t timer_ha_trabalho(void)
{
    return passos_pendentes != 0 || evento_botao != 0;
}

uint8_t timer_pega_botao(void)
{
    uint8_t ev;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        ev = evento_botao;
        evento_botao = 0;
    }
    return ev;
}
