/*
 * test_controle.c - Testes unitarios (no PC) da logica de controle,
 * dos parametros e do protocolo serial. Os mesmos arquivos-fonte do
 * firmware sao compilados com o gcc nativo.
 *
 *   make -C test/host
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "controle.h"
#include "params.h"
#include "protocolo.h"
#include "uart.h"

/* ------------------------------------------------------------------ */
/* Mini-framework                                                      */
/* ------------------------------------------------------------------ */
static int n_testes, n_falhas;
#define CHECK(cond) do { n_testes++; if (!(cond)) { n_falhas++; \
    printf("  FALHOU %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define TESTE(nome) static void nome(void)
#define RODA(nome) do { printf("- %s\n", #nome); nome(); } while (0)

/* ------------------------------------------------------------------ */
/* Stub da UART: captura a saida num buffer                            */
/* ------------------------------------------------------------------ */
static char saida[8192];
static size_t n_saida;
void uart_putc(char c) { if (n_saida < sizeof saida - 1) { saida[n_saida++] = c; saida[n_saida] = 0; } }
int  uart_getc(void) { return -1; }
void uart_init(void) {}
uint8_t uart_estouro_rx(void) { return 0; }
static void limpa_saida(void) { n_saida = 0; saida[0] = 0; }
static int  contem(const char *s) { return strstr(saida, s) != NULL; }

static int salvou;
void    app_salvar(void)   { salvou++; }
uint8_t app_carregar(void) { return 1; }

static void envia(const char *linha)
{
    while (*linha) prot_rx(*linha++);
    prot_rx('\r');
}

/* ------------------------------------------------------------------ */
/* Auxiliares                                                          */
/* ------------------------------------------------------------------ */
static params_t P;
static ctrl_t   C;

/* soma de 32 amostras correspondente a um nivel com a calibracao padrao */
static uint16_t x32(double mm)
{
    double adc = 41.0 + mm * (493.0 - 41.0) / 500.0;
    return (uint16_t)(adc * 32.0 + 0.5);
}

static void inicia(void)
{
    params_padrao(&P);
    ctrl_init(&C);
    prot_init(&P, &C);
    limpa_saida();
}

/* executa n passos (100 ms cada) com nivel constante */
static void passos(int n, double mm)
{
    for (int i = 0; i < n; i++)
        ctrl_passo(&C, &P, x32(mm));
}

/* ------------------------------------------------------------------ */
TESTE(conversao_adc_nivel)
{
    params_padrao(&P);
    CHECK(ctrl_nivel_mm(&P, 41 * 32) == 0);
    CHECK(ctrl_nivel_mm(&P, 493 * 32) == 500);
    CHECK(abs(ctrl_nivel_mm(&P, x32(250)) - 250) <= 1);
    CHECK(ctrl_nivel_mm(&P, 10 * 32) == 0);          /* abaixo do zero satura em 0 */
    /* resolucao fina via sobreamostragem: +1 na soma = +1/32 LSB */
    CHECK(ctrl_nivel_mm(&P, 267 * 32 + 16) > ctrl_nivel_mm(&P, 267 * 32));
    /* sensor com inclinacao negativa */
    P.adc_zero = 900; P.adc_ref = 400; P.n_ref = 500;
    CHECK(ctrl_nivel_mm(&P, 900 * 32) == 0);
    CHECK(ctrl_nivel_mm(&P, 400 * 32) == 500);
    CHECK(ctrl_nivel_mm(&P, 650 * 32) == 250);
    params_padrao(&P);
    CHECK(ctrl_percentual(&P, 250) == 50);
    CHECK(ctrl_percentual(&P, 0) == 0);
}

TESTE(validacao_parametros)
{
    params_t p;
    params_padrao(&p);
    CHECK(params_valida(&p) == PARAM_OK);
    p.n_liga = 430;                                   /* liga acima de desliga */
    CHECK(params_valida(&p) == PARAM_ERR_ORDEM);
    params_padrao(&p); p.n_crit_a = 600;              /* acima da altura */
    CHECK(params_valida(&p) == PARAM_ERR_ORDEM);
    params_padrao(&p); p.adc_ref = p.adc_zero + 5;
    CHECK(params_valida(&p) == PARAM_ERR_CAL);
    params_padrao(&p); p.adc_fmin = 1020;
    CHECK(params_valida(&p) == PARAM_ERR_FAIXA_ADC);
    params_padrao(&p); p.sub_min = 0;
    CHECK(params_valida(&p) == PARAM_ERR_JANELA);
    p.janela = 0;                                     /* sem janela, sub_min livre */
    CHECK(params_valida(&p) == PARAM_OK);
}

TESTE(histerese_basica)
{
    inicia();
    passos(200, 300);                  /* faixa morta: permanece desligada */
    CHECK(!C.bomba);
    passos(19, 199);                   /* abaixo de LIGA ha 1,9 s: ainda nao */
    CHECK(!C.bomba);
    passos(1, 199);                    /* 2,0 s confirmados */
    CHECK(C.bomba && C.motivo == MOT_NIVEL_BAIXO);
    passos(100, 300);                  /* sobe na faixa: continua ligada */
    CHECK(C.bomba);
    passos(19, 421);
    CHECK(C.bomba);
    passos(1, 421);
    CHECK(!C.bomba && C.motivo == MOT_NIVEL_ALTO);
    passos(200, 300);                  /* desce na faixa: continua desligada */
    CHECK(!C.bomba);
}

TESTE(pico_curto_ignorado)
{
    inicia();
    passos(150, 300);
    for (int k = 0; k < 10; k++) {     /* picos de 1 s abaixo do limiar */
        passos(10, 150);
        passos(5, 300);
    }
    CHECK(!C.bomba);
}

TESTE(tempo_minimo_desligada)
{
    inicia();
    passos(20, 150);                   /* 2 s apos o boot: confirmado, mas < 10 s */
    CHECK(!C.bomba);
    passos(80, 150);                   /* 10 s apos o boot */
    CHECK(C.bomba);
}

TESTE(falha_sensor)
{
    inicia();
    passos(150, 150);
    CHECK(C.bomba);
    for (int i = 0; i < 9; i++) ctrl_passo(&C, &P, 0);
    CHECK(!(C.alarmes & AL_SENSOR));
    CHECK(!C.bomba);                   /* leitura invalida ja impede operar */
    ctrl_passo(&C, &P, 0);
    CHECK(C.alarmes & AL_SENSOR);
    CHECK(C.motivo == MOT_SENSOR);
    CHECK(C.buzzer);
    passos(29, 150);
    CHECK(C.alarmes & AL_SENSOR);
    passos(1, 150);                    /* 3 s validos: alarme limpo */
    CHECK(!(C.alarmes & AL_SENSOR));
    passos(25, 150);                   /* 6,5 s desde o desligamento: TMINOFF=10 s */
    CHECK(!C.bomba);
    passos(36, 150);                   /* 10 s cumpridos: volta a operar */
    CHECK(C.bomba);
    /* curto para VCC */
    for (int i = 0; i < 10; i++) ctrl_passo(&C, &P, 1023 * 32);
    CHECK((C.alarmes & AL_SENSOR) && !C.bomba);
}

TESTE(falha_abastecimento_tentativas_bloqueio)
{
    inicia();
    P.t_retry = 60;
    passos(100, 150);
    CHECK(C.bomba);
    passos(300, 150);                  /* 30 s sem subida */
    CHECK(!C.bomba);
    CHECK(C.estado == EST_FALHA && (C.alarmes & AL_ABAST) && C.tentativas == 1);
    passos(599, 150);
    CHECK(C.estado == EST_FALHA);
    passos(1, 150);                    /* 60 s: nova tentativa */
    CHECK(C.estado == EST_DESLIGADA && C.motivo == MOT_NOVA_TENTATIVA);
    passos(19, 150);
    CHECK(!C.bomba);                   /* nivel baixo ainda precisa de 2 s */
    passos(1, 150);
    CHECK(C.bomba);                    /* religou (tempo minimo ja cumprido) */
    passos(300, 150);
    CHECK(C.tentativas == 2 && !(C.alarmes & AL_BLOQUEIO));
    passos(600 + 20, 150);             /* espera + confirmacao */
    CHECK(C.bomba);
    passos(300, 150);                  /* terceira falha > NRETRY=2 */
    CHECK(C.tentativas == 3 && (C.alarmes & AL_BLOQUEIO));
    passos(2000, 150);                 /* bloqueado: nao tenta mais */
    CHECK(C.estado == EST_FALHA && !C.bomba);
    ctrl_reconhece(&C);
    CHECK(C.estado == EST_DESLIGADA && C.alarmes == 0 && C.tentativas == 0);
    passos(21, 150);
    CHECK(C.bomba);
}

TESTE(abastecimento_normal_nao_alarma)
{
    inicia();
    passos(100, 150);
    CHECK(C.bomba);
    double n = 150;
    for (int i = 0; i < 3000 && C.bomba; i++) {
        n += 0.1;                      /* 1 mm/s: 30 mm por janela > SUBMIN */
        ctrl_passo(&C, &P, x32(n));
    }
    CHECK(!C.bomba && C.motivo == MOT_NIVEL_ALTO && C.alarmes == 0);
}

TESTE(tempo_maximo)
{
    inicia();
    P.t_max = 120;
    passos(100, 150);
    CHECK(C.bomba);
    double n = 150;
    for (int i = 0; i < 1199; i++) {   /* 0,5 mm/s: passa na janela, mas e lento */
        n += 0.05;
        ctrl_passo(&C, &P, x32(n));
    }
    CHECK(C.bomba);
    ctrl_passo(&C, &P, x32(n));
    CHECK(!C.bomba && (C.alarmes & AL_TMAX) && C.estado == EST_FALHA);
    passos(5000, 150);                 /* TMAX nao tem nova tentativa automatica */
    CHECK(!C.bomba);
    ctrl_reconhece(&C);
    passos(21, 150);
    CHECK(C.bomba);
}

TESTE(manual_com_intertravamento_critico_alto)
{
    inicia();
    passos(50, 300);
    CHECK(!ctrl_bomba_manual(&C, true));          /* recusado no modo auto */
    ctrl_modo(&C, MODO_MANUAL);
    CHECK(ctrl_bomba_manual(&C, true));
    passos(1, 300);
    CHECK(C.bomba && C.motivo == MOT_MANUAL);
    passos(50, 440);                               /* manual ignora DESLIGA */
    CHECK(C.bomba);
    passos(19, 471);
    CHECK(C.bomba);
    passos(1, 471);                                /* 2 s acima de CRITA */
    CHECK(!C.bomba && (C.alarmes & AL_CRIT_ALTO) && C.motivo == MOT_CRIT_ALTO);
    CHECK(!ctrl_bomba_manual(&C, true));           /* intertravado */
    passos(20, 465);                               /* dentro da histerese do alarme */
    CHECK(C.alarmes & AL_CRIT_ALTO);
    passos(20, 455);
    CHECK(!(C.alarmes & AL_CRIT_ALTO));
    CHECK(ctrl_bomba_manual(&C, true));
    ctrl_modo(&C, MODO_AUTO);                      /* volta ao automatico */
    passos(30, 300);
    CHECK(!C.bomba);
}

TESTE(critico_baixo)
{
    inicia();
    P.n_crit_b = 80;
    passos(19, 70);
    CHECK(!(C.alarmes & AL_CRIT_BAIXO));
    passos(1, 70);
    CHECK(C.alarmes & AL_CRIT_BAIXO);
    passos(20, 85);                                /* histerese de 10 mm */
    CHECK(C.alarmes & AL_CRIT_BAIXO);
    passos(20, 95);
    CHECK(!(C.alarmes & AL_CRIT_BAIXO));
}

TESTE(subida_indevida_com_bomba_desligada)
{
    inicia();
    passos(100, 300);
    CHECK(!C.bomba && !(C.alarmes & AL_INDEVIDO));
    double n = 300;
    for (int i = 0; i < 600; i++) {   /* sobe 0,5 mm/s sem a bomba */
        n += 0.05;
        ctrl_passo(&C, &P, x32(n));
    }
    CHECK(C.alarmes & AL_INDEVIDO);
    ctrl_reconhece(&C);
    CHECK(!(C.alarmes & AL_INDEVIDO));
    /* consumo normal (nivel caindo) nao gera alarme */
    for (int i = 0; i < 600; i++) {
        n -= 0.05;
        ctrl_passo(&C, &P, x32(n));
    }
    CHECK(!(C.alarmes & AL_INDEVIDO));
}

TESTE(buzzer_silenciado_e_reativado)
{
    inicia();
    passos(20, 70);                    /* critico baixo */
    CHECK(C.buzzer);
    ctrl_reconhece(&C);                /* silencia, alarme persiste */
    CHECK(!C.buzzer && (C.alarmes & AL_CRIT_BAIXO));
    passos(10, 70);
    CHECK(!C.buzzer);
    for (int i = 0; i < 10; i++) ctrl_passo(&C, &P, 0);   /* alarme novo */
    CHECK(C.buzzer);
}

TESTE(protocolo_comandos)
{
    inicia();
    passos(5, 300);

    limpa_saida(); envia("status");
    CHECK(contem("Nivel: 300 mm (60 %)") && contem("OK"));

    limpa_saida(); envia("SET LIGA 450");          /* violaria a ordem */
    CHECK(contem("ERRO") && P.n_liga == 200);

    limpa_saida(); envia("set liga 300 desliga 301"); /* atomico */
    CHECK(contem("OK") && P.n_liga == 300 && P.n_desl == 301);

    limpa_saida(); envia("SET XYZ 1");
    CHECK(contem("desconhecido"));

    limpa_saida(); envia("SET TCONF 9999");
    CHECK(contem("fora da faixa") && P.t_conf == 2);

    limpa_saida(); envia("SET LIGA");
    CHECK(contem("uso"));

    limpa_saida(); envia("BOMBA LIGA");
    CHECK(contem("MANUAL"));                       /* so no modo manual */

    limpa_saida(); envia("modo manual"); envia("bomba liga");
    CHECK(C.modo == MODO_MANUAL && C.cmd_manual);

    limpa_saida(); envia("CAL ZERO");
    CHECK(contem("ADCZERO=") && P.adc_zero == C.adc);

    limpa_saida(); envia("SALVAR");
    CHECK(salvou == 1);

    limpa_saida(); envia("PADRAO");
    CHECK(P.n_liga == 200);

    limpa_saida(); envia("PARAMS");
    CHECK(contem("LIGA=200 mm") && contem("TELEM=5 s"));

    limpa_saida(); envia("FOO");
    CHECK(contem("desconhecido"));

    limpa_saida();                                 /* linha longa descartada */
    for (int i = 0; i < 100; i++) prot_rx('A');
    prot_rx('\n');
    CHECK(contem("muito longa"));

    limpa_saida(); envia("STATUX\b\bUS");          /* backspace */
    CHECK(contem("Nivel:"));

    limpa_saida();                                 /* 6 pares: aceito */
    envia("SET TELEM 1 TCONF 0 TMINOFF 0 LIGA 300 DESLIGA 301 TMAX 600");
    CHECK(contem("OK") && P.t_max == 600 && P.t_conf == 0);

    limpa_saida();                                 /* 7 pares: recusado */
    envia("SET TELEM 2 TCONF 1 TMAX 5 TELEM 3 TCONF 2 TMAX 6 TELEM 4"); /* < 64 car. */
    CHECK(contem("tokens demais") && P.telem == 1);
}

TESTE(protocolo_eventos_e_telemetria)
{
    inicia();
    P.telem = 1;
    limpa_saida();
    for (int i = 0; i < 120; i++) {
        ctrl_passo(&C, &P, x32(150));
        prot_pos_passo();
    }
    CHECK(contem("#D,"));
    CHECK(contem("BOMBA=LIGADA EST=LIGADA MOT=NIVEL_BAIXO"));
    limpa_saida();
    for (int i = 0; i < 10; i++) {
        ctrl_passo(&C, &P, 0);
        prot_pos_passo();
    }
    CHECK(contem("ALARME+ SENSOR"));
    CHECK(contem("MOT=SENSOR"));
}

int main(void)
{
    RODA(conversao_adc_nivel);
    RODA(validacao_parametros);
    RODA(histerese_basica);
    RODA(pico_curto_ignorado);
    RODA(tempo_minimo_desligada);
    RODA(falha_sensor);
    RODA(falha_abastecimento_tentativas_bloqueio);
    RODA(abastecimento_normal_nao_alarma);
    RODA(tempo_maximo);
    RODA(manual_com_intertravamento_critico_alto);
    RODA(critico_baixo);
    RODA(subida_indevida_com_bomba_desligada);
    RODA(buzzer_silenciado_e_reativado);
    RODA(protocolo_comandos);
    RODA(protocolo_eventos_e_telemetria);
    printf("\n%d verificacoes, %d falhas\n", n_testes, n_falhas);
    return n_falhas ? 1 : 0;
}
