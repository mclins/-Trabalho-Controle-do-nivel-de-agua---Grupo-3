#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>

void    timer_init(void);
uint8_t timer_pega_passo(void);    /* 1 se um passo de 100 ms esta pendente */
uint8_t timer_pega_botao(void);    /* 1 se o botao foi pressionado          */
uint8_t timer_ha_trabalho(void);   /* chamado com interrupcoes desabilitadas */

#endif
