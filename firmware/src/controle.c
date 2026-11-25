/*
 * controle.c - Controle liga/desliga com histerese e deteccao de falhas
 *
 * Executado a cada 100 ms. Todas as temporizacoes sao contadas em passos
 * de 100 ms; os parametros (em segundos) sao convertidos aqui.
 */
#include "controle.h"

#define SAT_INC16(x) do { if ((x) < 0xFFFFu) (x)++; } while (0)
#define SAT_INC32(x) do { if ((x) < 0xFFFFFFFFul) (x)++; } while (0)

/* Retorna true quando 'cond' permaneceu verdadeira por pelo menos 'n'
 * passos consecutivos. E o filtro temporal que impede que uma onda na
 * superficie ou um pico de ruido provoque uma transicao. */
static bool confirma(uint16_t *cont, bool cond, uint16_t n)
{
    if (!cond) {
        *cont = 0;
        return false;
    }
    SAT_INC16(*cont);
    return *cont >= n;
}

/* ------------------------------------------------------------------ */
/* Conversao ADC -> nivel                                              */
/* ------------------------------------------------------------------ */
/* Reta por dois pontos de calibracao (adc_zero -> 0 mm, adc_ref -> n_ref).
 * A entrada e a SOMA de 32 amostras: usar a soma (e nao a media
 * truncada) preserva a resolucao extra obtida pela sobreamostragem. */
int16_t ctrl_nivel_mm(const params_t *p, uint16_t adc_x32)
{
    int32_t num = ((int32_t)adc_x32 - (int32_t)CTRL_SOBREAMOSTRA * p->adc_zero) *
                  (int32_t)p->n_ref;
    int32_t den = (int32_t)CTRL_SOBREAMOSTRA * ((int32_t)p->adc_ref - (int32_t)p->adc_zero);
    int32_t q;

    if (den == 0)
        return 0;
    if (den < 0) {               /* sensor com inclinacao negativa */
        den = -den;
        num = -num;
    }
    if (num >= 0) q = (num + den / 2) / den;     /* arredondamento */
    else          q = -((-num + den / 2) / den);

    if (q < 0)     q = 0;
    if (q > 32767) q = 32767;
    return (int16_t)q;
}

uint8_t ctrl_percentual(const params_t *p, int16_t nivel_mm)
{
    int32_t pc;
    if (nivel_mm <= 0 || p->altura == 0)
        return 0;
    pc = ((int32_t)nivel_mm * 100 + p->altura / 2) / p->altura;
    return (uint8_t)(pc > 255 ? 255 : pc);
}

/* ------------------------------------------------------------------ */
/* Transicoes                                                          */
/* ------------------------------------------------------------------ */
static void liga(ctrl_t *c, motivo_t m)
{
    c->bomba     = true;
    c->estado    = EST_LIGADA;
    c->motivo    = m;
    c->t_ligada  = 0;
    c->t_janela  = 0;
    c->nivel_ref = c->nivel_mm;
    c->c_desl    = 0;
}

static void desliga(ctrl_t *c, motivo_t m)
{
    c->bomba       = false;
    c->estado      = EST_DESLIGADA;
    c->motivo      = m;
    c->t_desligada = 0;
    c->t_janela    = 0;
    c->nivel_ref   = c->nivel_mm;
    c->c_liga      = 0;
}

static void entra_falha(ctrl_t *c)
{
    c->estado     = EST_FALHA;
    c->t_falha    = 0;
    c->cmd_manual = false;
}

/* Com a bomba desligada o nivel so pode cair ou ficar parado. Uma subida
 * maior que sub_min dentro da janela indica entrada indevida de agua:
 * contato do rele soldado, bomba acionada por outro circuito, etc. */
static void janela_desligada(ctrl_t *c, const params_t *p)
{
    if ((int32_t)c->nivel_mm - c->nivel_ref >= (int32_t)p->sub_min)
        c->alarmes |= AL_INDEVIDO;
    c->nivel_ref = c->nivel_mm;
    c->t_janela  = 0;
}

/* ------------------------------------------------------------------ */
void ctrl_init(ctrl_t *c)
{
    *c = (ctrl_t){0};
    c->modo   = MODO_AUTO;
    c->estado = EST_DESLIGADA;
    c->motivo = MOT_NENHUM;
    /* t_desligada = 0: o tempo minimo desligada tambem vale apos a
     * energizacao (partida retardada), o que da tempo do filtro do ADC
     * se estabilizar e evita religar o motor logo apos uma queda de energia. */
}

void ctrl_passo(ctrl_t *c, const params_t *p, uint16_t adc_x32)
{
    const uint16_t n_conf = (uint16_t)(p->t_conf * CTRL_PASSOS_POR_S);
    const uint32_t n_jan  = (uint32_t)p->janela * CTRL_PASSOS_POR_S;
    uint16_t adc = (uint16_t)((adc_x32 + CTRL_SOBREAMOSTRA / 2) / CTRL_SOBREAMOSTRA);
    bool invalido, sensor_ok, fim_janela;

    SAT_INC32(c->passos);
    c->adc = adc;

    /* 1) Plausibilidade eletrica do sensor ----------------------------- */
    invalido = (adc < p->adc_fmin) || (adc > p->adc_fmax);
    if (confirma(&c->c_sens_ruim, invalido, CTRL_SENSOR_FALHA))
        c->alarmes |= AL_SENSOR;
    if (confirma(&c->c_sens_bom, !invalido, CTRL_SENSOR_OK))
        c->alarmes &= (uint8_t)~AL_SENSOR;
    sensor_ok = !invalido && !(c->alarmes & AL_SENSOR);

    /* 2) Nivel e alarmes de nivel critico (confirmacao + histerese) ---- */
    if (sensor_ok) {
        int16_t n = ctrl_nivel_mm(p, adc_x32);
        c->nivel_mm = n;
        if (confirma(&c->c_ca_on, n >= (int32_t)p->n_crit_a, n_conf))
            c->alarmes |= AL_CRIT_ALTO;
        if (confirma(&c->c_ca_off, n < (int32_t)p->n_crit_a - CTRL_HIST_ALARME_MM, n_conf))
            c->alarmes &= (uint8_t)~AL_CRIT_ALTO;
        if (confirma(&c->c_cb_on, n <= (int32_t)p->n_crit_b, n_conf))
            c->alarmes |= AL_CRIT_BAIXO;
        if (confirma(&c->c_cb_off, n > (int32_t)p->n_crit_b + CTRL_HIST_ALARME_MM, n_conf))
            c->alarmes &= (uint8_t)~AL_CRIT_BAIXO;
    } else {
        c->c_ca_on = c->c_ca_off = c->c_cb_on = c->c_cb_off = 0;
    }

    /* 3) Temporizadores ------------------------------------------------ */
    if (c->bomba) SAT_INC32(c->t_ligada);
    else          SAT_INC32(c->t_desligada);
    SAT_INC32(c->t_janela);
    if (!sensor_ok)
        c->t_janela = 0;                 /* janela reinicia quando o sensor volta */
    else if (c->t_janela == 1)
        c->nivel_ref = c->nivel_mm;      /* primeiro passo de uma nova janela */
    fim_janela = sensor_ok && p->janela && c->t_janela >= n_jan;

    /* 4) Maquina de estados da bomba ----------------------------------- */
    switch (c->estado) {
    case EST_DESLIGADA:
        if (fim_janela)
            janela_desligada(c, p);
        if (!sensor_ok || (c->alarmes & AL_CRIT_ALTO)) {
            c->c_liga = 0;
            c->cmd_manual = false;
            break;
        }
        if (c->modo == MODO_AUTO) {
            bool baixo = confirma(&c->c_liga, c->nivel_mm <= (int32_t)p->n_liga, n_conf);
            if (baixo && c->t_desligada >= (uint32_t)p->t_minoff * CTRL_PASSOS_POR_S)
                liga(c, MOT_NIVEL_BAIXO);
        } else {
            c->c_liga = 0;
            if (c->cmd_manual)
                liga(c, MOT_MANUAL);
        }
        break;

    case EST_LIGADA:
        /* intertravamentos: valem nos modos automatico e manual */
        if (!sensor_ok) {
            desliga(c, MOT_SENSOR);
            c->cmd_manual = false;
            break;
        }
        if (c->alarmes & AL_CRIT_ALTO) {
            desliga(c, MOT_CRIT_ALTO);
            c->cmd_manual = false;
            break;
        }
        if (c->modo == MODO_AUTO) {
            if (confirma(&c->c_desl, c->nivel_mm >= (int32_t)p->n_desl, n_conf)) {
                desliga(c, MOT_NIVEL_ALTO);
                c->tentativas = 0;               /* enchimento completo */
                c->alarmes &= (uint8_t)~AL_ABAST;
                break;
            }
        } else if (!c->cmd_manual) {
            desliga(c, MOT_MANUAL);
            break;
        }
        /* protecao: tempo maximo de funcionamento continuo */
        if (p->t_max && c->t_ligada >= (uint32_t)p->t_max * CTRL_PASSOS_POR_S) {
            desliga(c, MOT_TMAX);
            c->alarmes |= AL_TMAX;
            entra_falha(c);
            break;
        }
        /* protecao: a bomba ligada precisa fazer o nivel subir */
        if (fim_janela) {
            if ((int32_t)c->nivel_mm - c->nivel_ref < (int32_t)p->sub_min) {
                desliga(c, MOT_ABAST);
                c->alarmes |= AL_ABAST;
                if (c->tentativas < 255) c->tentativas++;
                if (c->tentativas > p->n_retry)
                    c->alarmes |= AL_BLOQUEIO;
                entra_falha(c);
            } else {
                c->alarmes &= (uint8_t)~AL_ABAST;   /* abastecimento normalizado */
                c->nivel_ref = c->nivel_mm;
                c->t_janela  = 0;
            }
        }
        break;

    case EST_FALHA:
        SAT_INC32(c->t_falha);
        if (fim_janela)
            janela_desligada(c, p);
        /* falha de abastecimento: nova tentativa automatica apos t_retry,
         * ate n_retry vezes. Tempo maximo e bloqueio exigem o operador. */
        if (!(c->alarmes & (AL_TMAX | AL_BLOQUEIO)) &&
            c->t_falha >= (uint32_t)p->t_retry * CTRL_PASSOS_POR_S) {
            c->estado = EST_DESLIGADA;
            c->motivo = MOT_NOVA_TENTATIVA;
        }
        break;
    }

    /* 5) Sinalizacao: qualquer alarme novo reativa o buzzer ------------- */
    if (c->alarmes & (uint8_t)~c->alarmes_ant)
        c->silenciado = false;
    c->alarmes_ant = c->alarmes;
    c->buzzer = (c->alarmes != 0) && !c->silenciado;
}

void ctrl_reconhece(ctrl_t *c)
{
    c->alarmes    &= (uint8_t)~AL_TRAVADOS;
    c->alarmes_ant = c->alarmes;
    c->tentativas  = 0;
    c->silenciado  = true;
    c->buzzer      = false;
    if (c->estado == EST_FALHA) {
        c->estado = EST_DESLIGADA;
        c->motivo = MOT_RECONHECIDO;
    }
}

void ctrl_modo(ctrl_t *c, modo_t m)
{
    if (m == c->modo)
        return;
    c->modo = m;
    /* transferencia sem solavanco: ao entrar no manual mantem o estado atual */
    c->cmd_manual = (m == MODO_MANUAL) ? c->bomba : false;
    c->c_liga = c->c_desl = 0;
}

bool ctrl_bomba_manual(ctrl_t *c, bool ligar)
{
    if (c->modo != MODO_MANUAL)
        return false;
    if (ligar && (c->estado == EST_FALHA || (c->alarmes & (AL_SENSOR | AL_CRIT_ALTO))))
        return false;
    c->cmd_manual = ligar;
    return true;
}
