/*
 * params.h - Parametros de operacao configuraveis pela serial e
 * persistidos na EEPROM.
 */
#ifndef PARAMS_H
#define PARAMS_H

#include <stdint.h>

/* Todos os campos sao uint16_t para permitir uma tabela generica de
 * acesso (comando SET/PARAMS). A ordem dos campos define o layout na
 * EEPROM: ao alterar, incremente PARAMS_VERSAO. */
typedef struct {
    uint16_t n_liga;     /* mm  - liga a bomba quando nivel <= n_liga        */
    uint16_t n_desl;     /* mm  - desliga a bomba quando nivel >= n_desl     */
    uint16_t n_crit_b;   /* mm  - alarme de nivel criticamente baixo         */
    uint16_t n_crit_a;   /* mm  - alarme de nivel criticamente alto          */
    uint16_t altura;     /* mm  - altura util do reservatorio (100 %)        */
    uint16_t adc_zero;   /* LSB - leitura do ADC com o reservatorio vazio    */
    uint16_t adc_ref;    /* LSB - leitura do ADC no nivel de referencia      */
    uint16_t n_ref;      /* mm  - nivel de referencia da calibracao          */
    uint16_t adc_fmin;   /* LSB - abaixo disso: sensor aberto/em curto p/ GND */
    uint16_t adc_fmax;   /* LSB - acima disso: sensor em curto p/ VCC        */
    uint16_t t_conf;     /* s   - tempo de confirmacao de cruzamento         */
    uint16_t t_minoff;   /* s   - tempo minimo com a bomba desligada         */
    uint16_t t_max;      /* s   - tempo maximo ligada (0 = desabilitado)     */
    uint16_t janela;     /* s   - janela de verificacao de subida (0 = desab.) */
    uint16_t sub_min;    /* mm  - subida minima esperada por janela          */
    uint16_t t_retry;    /* s   - espera antes de nova tentativa             */
    uint16_t n_retry;    /* -   - tentativas automaticas antes do bloqueio   */
    uint16_t telem;      /* s   - periodo da telemetria (0 = desligada)      */
} params_t;

#define PARAMS_N        (sizeof(params_t) / sizeof(uint16_t))
#define PARAMS_VERSAO   1u

/* Codigos de validacao */
typedef enum {
    PARAM_OK = 0,
    PARAM_ERR_ORDEM,     /* exige crit_b < liga < desl < crit_a <= altura */
    PARAM_ERR_CAL,       /* calibracao degenerada                         */
    PARAM_ERR_FAIXA_ADC, /* adc_fmin >= adc_fmax                          */
    PARAM_ERR_JANELA     /* janela > 0 exige sub_min > 0                  */
} param_erro_t;

void         params_padrao(params_t *p);
param_erro_t params_valida(const params_t *p);

/* Acesso generico por indice (usado pelo interpretador de comandos) */
uint16_t     params_get(const params_t *p, uint8_t idx);
void         params_set(params_t *p, uint8_t idx, uint16_t v);

#ifdef __AVR__
/* Persistencia: retorna 1 se carregou da EEPROM, 0 se usou o padrao */
uint8_t      params_carrega(params_t *p);
void         params_salva(const params_t *p);
#endif

#endif /* PARAMS_H */
