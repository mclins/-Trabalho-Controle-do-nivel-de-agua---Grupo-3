/*
 * protocolo.h - Interface serial com o operador
 *
 * Linhas recebidas (terminadas em CR e/ou LF, maiusculas ou minusculas)
 * sao interpretadas como comandos. O sistema envia:
 *   - respostas aos comandos, terminadas em "OK" ou "ERRO: ...";
 *   - eventos espontaneos, iniciados por '!' (mudanca da bomba, alarmes);
 *   - telemetria periodica, iniciada por "#D," (formato CSV).
 */
#ifndef PROTOCOLO_H
#define PROTOCOLO_H

#include <stdint.h>
#include "params.h"
#include "controle.h"

#define PROT_LINHA_MAX 64u

void prot_init(params_t *p, ctrl_t *c);
void prot_rx(char ch);          /* alimenta um byte recebido            */
void prot_pos_passo(void);      /* eventos + telemetria, a cada 100 ms  */
void prot_boot(uint8_t causa_reset, uint8_t params_da_eeprom);
void prot_botao(void);          /* reconhecimento pelo botao local      */

/* Fornecidas pela aplicacao (main.c) ou pelos testes */
void    app_salvar(void);
uint8_t app_carregar(void);     /* 1 se a EEPROM era valida */

#endif
