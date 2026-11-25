/*
 * main.c - Sistema inteligente de controle de nivel de agua (ATmega328P)
 *
 * ENGC50 - Sistemas Microprocessados - UFBA - 2026.2 - Problema 2
 *
 * Organizacao das tarefas (primeiro plano / segundo plano):
 *
 *   ISR TIMER1_COMPA (10 ms) -> dispara ADC, debounce do botao, conta passos
 *   ISR ADC                  -> media movel de 32 amostras
 *   ISR USART_RX / UDRE      -> buffers circulares de recepcao/transmissao
 *
 *   laco principal           -> comandos seriais, controle (100 ms),
 *                               saidas, eventos, telemetria; dorme (IDLE)
 *                               quando nao ha trabalho
 *
 * Nenhuma tarefa espera ativamente por outra: cada ISR e curta e o laco
 * principal so processa o que ja esta pronto.
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include "config.h"
#include "board.h"
#include "timer.h"
#include "adc.h"
#include "uart.h"
#include "params.h"
#include "controle.h"
#include "protocolo.h"

static params_t P;
static ctrl_t   C;

/* Causa do reset: MCUSR precisa ser lido e zerado logo no inicio, antes
 * de main(), e o watchdog desligado; senao um reset por watchdog vira um
 * laco de resets (o WDT continua ativo apos o reset). */
static uint8_t mcusr_copia __attribute__((section(".noinit")));
void le_mcusr(void) __attribute__((naked, used, section(".init3")));
void le_mcusr(void)
{
    mcusr_copia = MCUSR;
    MCUSR = 0;
    wdt_disable();
}

void app_salvar(void)
{
    params_salva(&P);
}

uint8_t app_carregar(void)
{
    return params_carrega(&P);
}

/* Padrao do buzzer e dos LEDs, avaliado a cada 100 ms */
static void atualiza_saidas(void)
{
    static uint8_t fase;                    /* 0..9: ciclo de 1 s */
    fase = (uint8_t)((fase + 1u) % 10u);

    saida(PINO_BOMBA, C.bomba);
    saida(PINO_LED_BOMBA, C.bomba);
    saida(PINO_LED_ALARME, C.alarmes != 0);

    /* falhas que bloqueiam a bomba: bipe 0,5 s ligado / 0,5 s desligado;
     * avisos (nivel critico, entrada indevida): bipe curto de 0,2 s */
    uint8_t grave = C.alarmes & (AL_SENSOR | AL_ABAST | AL_TMAX | AL_BLOQUEIO | AL_CRIT_ALTO);
    uint8_t bip   = grave ? (fase < 5) : (fase < 2);
    saida(PINO_BUZZER, C.buzzer && bip);

    if (fase == 0)
        saida_inverte(PINO_LED_VIDA);       /* pisca: firmware vivo */
}

int main(void)
{
    uint8_t da_eeprom;

    board_init();
    da_eeprom = params_carrega(&P);
    ctrl_init(&C);
    uart_init();
    adc_init();
    timer_init();
    prot_init(&P, &C);

    set_sleep_mode(SLEEP_MODE_IDLE);        /* timers, ADC e USART seguem ativos */
    wdt_enable(WDTO_1S);
    sei();

    prot_boot(mcusr_copia & 0x0F, da_eeprom);

    for (;;) {
        int ch;

        wdt_reset();

        while ((ch = uart_getc()) >= 0)
            prot_rx((char)ch);
        if (uart_estouro_rx())
            uart_puts_P(PSTR("ERRO: estouro do buffer de recepcao\r\n"));

        while (timer_pega_passo()) {
            if (adc_pronto())
                ctrl_passo(&C, &P, adc_soma());
            atualiza_saidas();
            prot_pos_passo();
        }

        if (timer_pega_botao())
            prot_botao();

        /* Dorme ate a proxima interrupcao. As interrupcoes sao desligadas
         * para testar se ha trabalho; sei seguido de sleep e atomico no
         * AVR, entao uma interrupcao entre o teste e o sleep nao se perde. */
        cli();
        if (!timer_ha_trabalho()) {
            sleep_enable();
            sei();
            sleep_cpu();
            sleep_disable();
        }
        sei();
    }
}
