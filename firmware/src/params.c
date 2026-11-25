/*
 * params.c - Valores padrao, validacao e persistencia dos parametros
 */
#include "params.h"

#ifdef __AVR__
#include <avr/eeprom.h>
#endif

/*
 * Valores padrao para um reservatorio de teste com 500 mm de altura util
 * e sensor MPX5010DP alimentado com 5 V (curva nominal do datasheet):
 *   Vout = Vs (0,09 P + 0,04),  P em kPa,  P = rho g h
 *   h = 0      -> 0,200 V -> ADC =  41
 *   h = 500 mm -> 2,407 V -> ADC = 493   (~0,9 LSB/mm)
 */
void params_padrao(params_t *p)
{
    p->n_liga   = 200;
    p->n_desl   = 420;
    p->n_crit_b = 80;
    p->n_crit_a = 470;
    p->altura   = 500;
    p->adc_zero = 41;
    p->adc_ref  = 493;
    p->n_ref    = 500;
    p->adc_fmin = 15;    /* ~73 mV: abaixo do offset minimo do sensor (88 mV) */
    p->adc_fmax = 1010;  /* ~4,93 V: acima do fundo de escala do sensor (4,7 V) */
    p->t_conf   = 2;
    p->t_minoff = 10;
    p->t_max    = 900;
    p->janela   = 30;
    p->sub_min  = 10;
    p->t_retry  = 300;
    p->n_retry  = 2;
    p->telem    = 5;
}

param_erro_t params_valida(const params_t *p)
{
    if (!(p->n_crit_b < p->n_liga && p->n_liga < p->n_desl &&
          p->n_desl < p->n_crit_a && p->n_crit_a <= p->altura))
        return PARAM_ERR_ORDEM;

    int16_t d = (int16_t)p->adc_ref - (int16_t)p->adc_zero;
    if (d < 0) d = (int16_t)-d;
    if (p->n_ref == 0 || d < 10 || p->adc_ref > 1023 || p->adc_zero > 1023)
        return PARAM_ERR_CAL;

    if (p->adc_fmin >= p->adc_fmax)
        return PARAM_ERR_FAIXA_ADC;

    if (p->janela > 0 && p->sub_min == 0)
        return PARAM_ERR_JANELA;

    return PARAM_OK;
}

uint16_t params_get(const params_t *p, uint8_t idx)
{
    return ((const uint16_t *)p)[idx];
}

void params_set(params_t *p, uint8_t idx, uint16_t v)
{
    ((uint16_t *)p)[idx] = v;
}

#ifdef __AVR__
/* ------------------------------------------------------------------ */
/* Bloco na EEPROM: [marca][versao][parametros][CRC-16]                */
/* ------------------------------------------------------------------ */
#define EE_MARCA 0xA55Au

typedef struct {
    uint16_t marca;
    uint16_t versao;
    params_t p;
    uint16_t crc;
} ee_bloco_t;

static ee_bloco_t EEMEM ee_bloco;

/* CRC-16/CCITT (polinomio 0x1021) */
static uint16_t crc16(const uint8_t *d, uint16_t n)
{
    uint16_t crc = 0xFFFF;
    while (n--) {
        crc ^= (uint16_t)(*d++) << 8;
        for (uint8_t i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }
    return crc;
}

uint8_t params_carrega(params_t *p)
{
    ee_bloco_t b;
    eeprom_read_block(&b, &ee_bloco, sizeof b);
    if (b.marca == EE_MARCA && b.versao == PARAMS_VERSAO &&
        b.crc == crc16((const uint8_t *)&b.p, sizeof b.p) &&
        params_valida(&b.p) == PARAM_OK) {
        *p = b.p;
        return 1;
    }
    params_padrao(p);
    return 0;
}

void params_salva(const params_t *p)
{
    ee_bloco_t b;
    b.marca  = EE_MARCA;
    b.versao = PARAMS_VERSAO;
    b.p      = *p;
    b.crc    = crc16((const uint8_t *)&b.p, sizeof b.p);
    /* eeprom_update_block so grava os bytes que mudaram: reduz desgaste
     * (100 mil ciclos por celula) e tempo de escrita (~3,4 ms/byte). */
    eeprom_update_block(&b, &ee_bloco, sizeof b);
}
#endif
