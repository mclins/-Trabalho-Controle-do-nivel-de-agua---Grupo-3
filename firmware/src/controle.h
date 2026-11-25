/*
 * controle.h - Logica de controle da bomba e deteccao de falhas.
 *
 * Modulo independente de hardware: recebe a leitura media do ADC a cada
 * passo de 100 ms e decide o estado da bomba e dos alarmes. Por nao
 * acessar registradores, e testado no PC (test/host) e reutilizado sem
 * alteracao no firmware.
 */
#ifndef CONTROLE_H
#define CONTROLE_H

#include <stdint.h>
#include <stdbool.h>
#include "params.h"

#define CTRL_PASSOS_POR_S   10u     /* ctrl_passo() chamado a 10 Hz       */
#define CTRL_SOBREAMOSTRA   32u     /* entrada = soma de 32 amostras ADC  */
#define CTRL_SENSOR_FALHA   10u     /* 1 s invalido -> alarme de sensor   */
#define CTRL_SENSOR_OK      30u     /* 3 s valido   -> limpa o alarme     */
#define CTRL_HIST_ALARME_MM 10      /* histerese dos alarmes criticos (mm) */

/* Bits de alarme */
#define AL_SENSOR     0x01u  /* leitura fora da faixa eletrica do sensor    */
#define AL_CRIT_BAIXO 0x02u  /* nivel criticamente baixo                    */
#define AL_CRIT_ALTO  0x04u  /* nivel criticamente alto (risco de transbordo)*/
#define AL_ABAST      0x08u  /* bomba ligada sem subida de nivel (fonte seca,
                                bomba/tubulacao com defeito)                 */
#define AL_TMAX       0x10u  /* bomba ligada alem do tempo maximo           */
#define AL_BLOQUEIO   0x20u  /* tentativas esgotadas: exige reconhecimento  */
#define AL_INDEVIDO   0x40u  /* nivel subindo com a bomba desligada
                                (rele colado, valvula/entrada indevida)      */
#define AL_TRAVADOS   (AL_ABAST | AL_TMAX | AL_BLOQUEIO | AL_INDEVIDO)
#define AL_N          7u

typedef enum { MODO_AUTO = 0, MODO_MANUAL = 1 } modo_t;

typedef enum {
    EST_DESLIGADA = 0,   /* aguardando nivel baixo                      */
    EST_LIGADA,          /* enchendo                                    */
    EST_FALHA            /* bomba bloqueada por falha de abastecimento  */
} estado_t;

typedef enum {
    MOT_NENHUM = 0,
    MOT_NIVEL_BAIXO,     /* liga: nivel <= n_liga                        */
    MOT_NIVEL_ALTO,      /* desliga: nivel >= n_desl                     */
    MOT_MANUAL,          /* comando do operador                          */
    MOT_SENSOR,          /* desliga por falha do sensor                  */
    MOT_CRIT_ALTO,       /* desliga por nivel critico alto               */
    MOT_ABAST,           /* desliga por falha de abastecimento           */
    MOT_TMAX,            /* desliga por tempo maximo                     */
    MOT_NOVA_TENTATIVA,  /* sai do estado de falha apos t_retry          */
    MOT_RECONHECIDO      /* sai do estado de falha por reconhecimento    */
} motivo_t;

typedef struct {
    /* saidas e estado observavel */
    modo_t   modo;
    estado_t estado;
    bool     bomba;          /* comando do rele                          */
    bool     buzzer;         /* ha alarme nao reconhecido                */
    uint8_t  alarmes;
    uint8_t  tentativas;
    motivo_t motivo;         /* motivo da ultima transicao da bomba      */
    int16_t  nivel_mm;       /* ultima estimativa valida                 */
    uint16_t adc;            /* media do ADC (LSB)                       */
    uint32_t passos;         /* tempo desde o boot, em passos de 100 ms  */

    /* estado interno */
    bool     cmd_manual;
    bool     silenciado;
    uint8_t  alarmes_ant;
    uint16_t c_liga, c_desl;             /* confirmacao dos limiares     */
    uint16_t c_ca_on, c_ca_off;          /* critico alto                 */
    uint16_t c_cb_on, c_cb_off;          /* critico baixo                */
    uint16_t c_sens_ruim, c_sens_bom;
    uint32_t t_ligada, t_desligada, t_janela, t_falha;
    int16_t  nivel_ref;                  /* inicio da janela de subida   */
} ctrl_t;

void    ctrl_init(ctrl_t *c);
void    ctrl_passo(ctrl_t *c, const params_t *p, uint16_t adc_x32);
void    ctrl_reconhece(ctrl_t *c);
void    ctrl_modo(ctrl_t *c, modo_t m);
bool    ctrl_bomba_manual(ctrl_t *c, bool ligar);   /* false = recusado */
int16_t ctrl_nivel_mm(const params_t *p, uint16_t adc_x32);
uint8_t ctrl_percentual(const params_t *p, int16_t nivel_mm);

#endif /* CONTROLE_H */
