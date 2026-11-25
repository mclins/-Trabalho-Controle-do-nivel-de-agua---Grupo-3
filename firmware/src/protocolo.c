/*
 * protocolo.c - Interpretador de comandos, eventos e telemetria
 *
 * Todas as strings constantes ficam na memoria de programa (PSTR /
 * PROGMEM): o ATmega328P tem so 2 KiB de SRAM.
 */
#include <stdbool.h>
#include "protocolo.h"
#include "uart.h"
#include "pgm_compat.h"
#include "config.h"

static params_t *P;
static ctrl_t   *C;

static char    linha[PROT_LINHA_MAX + 1];
static uint8_t tam;
static bool    descartando;

/* estado anterior, para gerar eventos so nas mudancas */
static bool     ant_bomba;
static estado_t ant_estado;
static uint8_t  ant_alarmes;
static uint32_t prox_telem;

/* ------------------------------------------------------------------ */
/* Tabelas na flash                                                    */
/* ------------------------------------------------------------------ */
typedef struct {
    char     nome[8];
    char     unid[4];
    uint16_t max;
} pdesc_t;

/* Mesma ordem dos campos de params_t */
static const pdesc_t PDESC[PARAMS_N] PROGMEM = {
    {"LIGA",    "mm",  30000}, {"DESLIGA", "mm",  30000},
    {"CRITB",   "mm",  30000}, {"CRITA",   "mm",  30000},
    {"ALTURA",  "mm",  30000}, {"ADCZERO", "LSB",  1023},
    {"ADCREF",  "LSB",  1023}, {"NREF",    "mm",  30000},
    {"ADCMIN",  "LSB",  1023}, {"ADCMAX",  "LSB",  1023},
    {"TCONF",   "s",     600}, {"TMINOFF", "s",    3600},
    {"TMAX",    "s",   65535}, {"JANELA",  "s",    3600},
    {"SUBMIN",  "mm",  30000}, {"TRETRY",  "s",   65535},
    {"NRETRY",  "",       20}, {"TELEM",   "s",    3600},
};

static const char s_al0[] PROGMEM = "SENSOR";
static const char s_al1[] PROGMEM = "CRIT_BAIXO";
static const char s_al2[] PROGMEM = "CRIT_ALTO";
static const char s_al3[] PROGMEM = "ABAST";
static const char s_al4[] PROGMEM = "TMAX";
static const char s_al5[] PROGMEM = "BLOQUEIO";
static const char s_al6[] PROGMEM = "INDEVIDO";
static const char * const NOME_AL[AL_N] PROGMEM = {
    s_al0, s_al1, s_al2, s_al3, s_al4, s_al5, s_al6
};

static const char s_mo0[] PROGMEM = "-";
static const char s_mo1[] PROGMEM = "NIVEL_BAIXO";
static const char s_mo2[] PROGMEM = "NIVEL_ALTO";
static const char s_mo3[] PROGMEM = "MANUAL";
static const char s_mo4[] PROGMEM = "SENSOR";
static const char s_mo5[] PROGMEM = "CRIT_ALTO";
static const char s_mo6[] PROGMEM = "ABAST";
static const char s_mo7[] PROGMEM = "TMAX";
static const char s_mo8[] PROGMEM = "NOVA_TENTATIVA";
static const char s_mo9[] PROGMEM = "RECONHECIDO";
static const char * const NOME_MOT[] PROGMEM = {
    s_mo0, s_mo1, s_mo2, s_mo3, s_mo4, s_mo5, s_mo6, s_mo7, s_mo8, s_mo9
};

static const char s_es0[] PROGMEM = "DESLIGADA";
static const char s_es1[] PROGMEM = "LIGADA";
static const char s_es2[] PROGMEM = "FALHA";
static const char * const NOME_EST[] PROGMEM = { s_es0, s_es1, s_es2 };

static const char AJUDA[] PROGMEM =
    "Comandos:\r\n"
    "  STATUS (S)            estado atual\r\n"
    "  PARAMS (P)            lista parametros\r\n"
    "  SET <nome> <v> [...]  ate 6 pares, validados em conjunto\r\n"
    "  CAL ZERO | CAL <mm>   calibra com o nivel atual\r\n"
    "  MODO AUTO | MANUAL\r\n"
    "  BOMBA LIGA | DESL     (somente no modo manual)\r\n"
    "  ACK                   reconhece alarmes / libera bloqueio\r\n"
    "  SALVAR | CARREGAR | PADRAO   EEPROM\r\n"
    "  VERSAO | AJUDA (?)\r\n";

/* ------------------------------------------------------------------ */
/* Auxiliares de saida                                                 */
/* ------------------------------------------------------------------ */
static void ok(void)                 { uart_puts_P(PSTR("OK\r\n")); }
static void erro(const char *msg_P)  { uart_puts_P(PSTR("ERRO: ")); uart_puts_P(msg_P); uart_nl(); }

static void puts_tab(const char * const *tab, uint8_t i)
{
    uart_puts_P((const char *)pgm_read_ptr(&tab[i]));
}

static void put_alarmes(uint8_t a)
{
    if (!a) {
        uart_puts_P(PSTR("nenhum"));
        return;
    }
    bool primeiro = true;
    for (uint8_t i = 0; i < AL_N; i++) {
        if (a & (1u << i)) {
            if (!primeiro) uart_putc(' ');
            puts_tab(NOME_AL, i);
            primeiro = false;
        }
    }
}

static void put_tempo(void)
{
    uart_putd1(C->passos);              /* passos de 100 ms -> segundos */
}

static void put_nome_param(uint8_t i)
{
    uart_puts_P(PDESC[i].nome);
}

static void msg_validacao(param_erro_t e)
{
    switch (e) {
    case PARAM_ERR_ORDEM:     erro(PSTR("exige CRITB < LIGA < DESLIGA < CRITA <= ALTURA")); break;
    case PARAM_ERR_CAL:       erro(PSTR("calibracao invalida (ADCREF-ADCZERO >= 10 LSB, NREF > 0)")); break;
    case PARAM_ERR_FAIXA_ADC: erro(PSTR("exige ADCMIN < ADCMAX")); break;
    case PARAM_ERR_JANELA:    erro(PSTR("JANELA > 0 exige SUBMIN > 0")); break;
    default: break;
    }
}

/* ------------------------------------------------------------------ */
/* Analise lexica                                                      */
/* ------------------------------------------------------------------ */
static bool num_u16(const char *s, uint16_t *v)
{
    uint32_t acc = 0;
    if (!*s)
        return false;
    while (*s) {
        if (*s < '0' || *s > '9')
            return false;
        acc = acc * 10u + (uint32_t)(*s - '0');
        if (acc > 65535u)
            return false;
        s++;
    }
    *v = (uint16_t)acc;
    return true;
}

static bool igual(const char *tok, const char *lit_P)
{
    return strcmp_P(tok, lit_P) == 0;
}

static int8_t acha_param(const char *tok)
{
    for (uint8_t i = 0; i < PARAMS_N; i++)
        if (strcmp_P(tok, PDESC[i].nome) == 0)
            return (int8_t)i;
    return -1;
}

/* ------------------------------------------------------------------ */
/* Comandos                                                            */
/* ------------------------------------------------------------------ */
static void cmd_status(void)
{
    uart_puts_P(PSTR("Nivel: "));    uart_puti(C->nivel_mm);
    uart_puts_P(PSTR(" mm ("));      uart_putu(ctrl_percentual(P, C->nivel_mm));
    uart_puts_P(PSTR(" %)  ADC: ")); uart_putu(C->adc); uart_nl();
    uart_puts_P(PSTR("Bomba: "));    uart_puts_P(C->bomba ? PSTR("LIGADA") : PSTR("DESLIGADA"));
    uart_puts_P(PSTR("  Estado: ")); puts_tab(NOME_EST, (uint8_t)C->estado);
    uart_puts_P(PSTR("  Modo: "));   uart_puts_P(C->modo == MODO_AUTO ? PSTR("AUTO") : PSTR("MANUAL"));
    uart_nl();
    uart_puts_P(PSTR("Alarmes: "));  put_alarmes(C->alarmes);
    uart_puts_P(PSTR("  Tentativas: ")); uart_putu(C->tentativas); uart_nl();
    uart_puts_P(PSTR("Ultimo motivo: ")); puts_tab(NOME_MOT, (uint8_t)C->motivo);
    uart_puts_P(PSTR("  Tempo: ")); put_tempo(); uart_puts_P(PSTR(" s")); uart_nl();
    ok();
}

static void cmd_params(void)
{
    for (uint8_t i = 0; i < PARAMS_N; i++) {
        uart_puts_P(PSTR("  "));
        put_nome_param(i);
        uart_putc('=');
        uart_putu(params_get(P, i));
        uart_putc(' ');
        uart_puts_P(PDESC[i].unid);
        uart_nl();
    }
    ok();
}

/* SET <nome> <valor> [<nome> <valor> ...]: todas as alteracoes sao
 * aplicadas numa copia e validadas em conjunto; ou entram todas, ou
 * nenhuma. Assim e possivel mover a faixa de histerese inteira de uma vez. */
static void cmd_set(char **tok, uint8_t n)
{
    params_t tmp = *P;
    if (n < 3 || (n - 1) % 2 != 0) {
        erro(PSTR("uso: SET <nome> <valor> [<nome> <valor> ...]"));
        return;
    }
    for (uint8_t i = 1; i < n; i += 2) {
        int8_t idx = acha_param(tok[i]);
        uint16_t v;
        if (idx < 0) {
            erro(PSTR("parametro desconhecido (veja PARAMS)"));
            return;
        }
        if (!num_u16(tok[i + 1], &v) || v > pgm_read_word(&PDESC[idx].max)) {
            erro(PSTR("valor fora da faixa"));
            return;
        }
        params_set(&tmp, (uint8_t)idx, v);
    }
    param_erro_t e = params_valida(&tmp);
    if (e != PARAM_OK) {
        msg_validacao(e);
        return;
    }
    *P = tmp;
    ok();
}

static void cmd_cal(char **tok, uint8_t n)
{
    params_t tmp = *P;
    uint16_t mm;
    if (n != 2) {
        erro(PSTR("uso: CAL ZERO | CAL <nivel_mm>"));
        return;
    }
    if ((C->alarmes & AL_SENSOR) || C->adc < P->adc_fmin || C->adc > P->adc_fmax) {
        erro(PSTR("sensor fora da faixa"));
        return;
    }
    if (igual(tok[1], PSTR("ZERO"))) {
        tmp.adc_zero = C->adc;
    } else if (num_u16(tok[1], &mm) && mm > 0 && mm <= 30000) {
        tmp.adc_ref = C->adc;
        tmp.n_ref   = mm;
    } else {
        erro(PSTR("uso: CAL ZERO | CAL <nivel_mm>"));
        return;
    }
    param_erro_t e = params_valida(&tmp);
    if (e != PARAM_OK) {
        msg_validacao(e);
        return;
    }
    *P = tmp;
    uart_puts_P(PSTR("ADCZERO=")); uart_putu(P->adc_zero);
    uart_puts_P(PSTR(" ADCREF="));  uart_putu(P->adc_ref);
    uart_puts_P(PSTR(" NREF="));    uart_putu(P->n_ref);
    uart_nl();
    ok();
}

static void executa(char **tok, uint8_t n)
{
    const char *c0 = tok[0];

    if (igual(c0, PSTR("STATUS")) || igual(c0, PSTR("S"))) {
        cmd_status();
    } else if (igual(c0, PSTR("PARAMS")) || igual(c0, PSTR("P"))) {
        cmd_params();
    } else if (igual(c0, PSTR("SET"))) {
        cmd_set(tok, n);
    } else if (igual(c0, PSTR("CAL"))) {
        cmd_cal(tok, n);
    } else if (igual(c0, PSTR("MODO")) && n == 2) {
        if (igual(tok[1], PSTR("AUTO")))        { ctrl_modo(C, MODO_AUTO);   ok(); }
        else if (igual(tok[1], PSTR("MANUAL"))) { ctrl_modo(C, MODO_MANUAL); ok(); }
        else erro(PSTR("uso: MODO AUTO | MANUAL"));
    } else if (igual(c0, PSTR("BOMBA")) && n == 2) {
        bool ligar;
        if (igual(tok[1], PSTR("LIGA")))      ligar = true;
        else if (igual(tok[1], PSTR("DESL"))) ligar = false;
        else { erro(PSTR("uso: BOMBA LIGA | DESL")); return; }
        if (C->modo != MODO_MANUAL)
            erro(PSTR("disponivel apenas no modo MANUAL"));
        else if (!ctrl_bomba_manual(C, ligar))
            erro(PSTR("intertravamento ativo (sensor, nivel critico alto ou falha)"));
        else
            ok();
    } else if (igual(c0, PSTR("ACK"))) {
        ctrl_reconhece(C);
        ok();
    } else if (igual(c0, PSTR("SALVAR"))) {
        app_salvar();
        ok();
    } else if (igual(c0, PSTR("CARREGAR"))) {
        if (app_carregar()) ok();
        else erro(PSTR("EEPROM invalida, padrao carregado"));
    } else if (igual(c0, PSTR("PADRAO"))) {
        params_padrao(P);
        ok();
    } else if (igual(c0, PSTR("VERSAO"))) {
        uart_puts_P(PSTR("Controle de nivel ATmega328P fw " FW_VERSAO "\r\n"));
        ok();
    } else if (igual(c0, PSTR("AJUDA")) || igual(c0, PSTR("?")) || igual(c0, PSTR("HELP"))) {
        uart_puts_P(AJUDA);
        ok();
    } else {
        erro(PSTR("comando desconhecido (digite AJUDA)"));
    }
}

#define MAX_TOKENS 13u                      /* SET + ate 6 pares nome/valor */

static void processa_linha(void)
{
    char *tok[MAX_TOKENS];
    uint8_t n = 0;
    char *s = linha;

    for (char *q = linha; *q; q++)                  /* maiusculas */
        if (*q >= 'a' && *q <= 'z')
            *q = (char)(*q - 'a' + 'A');

    while (*s && n < MAX_TOKENS) {                          /* separa por espacos */
        while (*s == ' ' || *s == '\t') s++;
        if (!*s) break;
        tok[n++] = s;
        while (*s && *s != ' ' && *s != '\t') s++;
        if (*s) *s++ = '\0';
    }
    while (*s == ' ' || *s == '\t') s++;
    if (*s) {                                       /* sobrou texto */
        erro(PSTR("tokens demais (SET aceita ate 6 pares por linha)"));
        return;
    }
    if (n)
        executa(tok, n);
}

/* ------------------------------------------------------------------ */
/* Interface publica                                                   */
/* ------------------------------------------------------------------ */
void prot_init(params_t *p, ctrl_t *c)
{
    P = p;
    C = c;
    tam = 0;
    descartando = false;
    ant_bomba   = c->bomba;
    ant_estado  = c->estado;
    ant_alarmes = c->alarmes;
    prox_telem  = 0;
}

void prot_rx(char ch)
{
    if (ch == '\r' || ch == '\n') {
        if (descartando) {
            erro(PSTR("linha muito longa"));
            descartando = false;
        } else if (tam) {
            linha[tam] = '\0';
            processa_linha();
        }
        tam = 0;
        return;
    }
    if (ch == 0x08 || ch == 0x7F) {                 /* backspace */
        if (tam) tam--;
        return;
    }
    if (descartando)
        return;
    if (tam >= PROT_LINHA_MAX) {
        descartando = true;
        return;
    }
    if (ch >= ' ' && ch <= '~')
        linha[tam++] = ch;
}

void prot_pos_passo(void)
{
    /* eventos espontaneos */
    if (C->bomba != ant_bomba || C->estado != ant_estado) {
        uart_putc('!'); put_tempo();
        uart_puts_P(PSTR(" BOMBA="));  uart_puts_P(C->bomba ? PSTR("LIGADA") : PSTR("DESL"));
        uart_puts_P(PSTR(" EST="));    puts_tab(NOME_EST, (uint8_t)C->estado);
        uart_puts_P(PSTR(" MOT="));    puts_tab(NOME_MOT, (uint8_t)C->motivo);
        uart_puts_P(PSTR(" N="));      uart_puti(C->nivel_mm);
        uart_nl();
        ant_bomba  = C->bomba;
        ant_estado = C->estado;
    }
    if (C->alarmes != ant_alarmes) {
        uint8_t mudou = C->alarmes ^ ant_alarmes;
        for (uint8_t i = 0; i < AL_N; i++) {
            if (mudou & (1u << i)) {
                uart_putc('!'); put_tempo();
                uart_puts_P((C->alarmes & (1u << i)) ? PSTR(" ALARME+ ") : PSTR(" ALARME- "));
                puts_tab(NOME_AL, i);
                uart_puts_P(PSTR(" N=")); uart_puti(C->nivel_mm);
                uart_nl();
            }
        }
        ant_alarmes = C->alarmes;
    }

    /* telemetria periodica: #D,t,adc,nivel,pct,bomba,estado,modo,alarmes */
    if (P->telem && C->passos >= prox_telem) {
        prox_telem = C->passos + (uint32_t)P->telem * PASSOS_POR_S;
        uart_puts_P(PSTR("#D,"));
        put_tempo();                          uart_putc(',');
        uart_putu(C->adc);                    uart_putc(',');
        uart_puti(C->nivel_mm);               uart_putc(',');
        uart_putu(ctrl_percentual(P, C->nivel_mm)); uart_putc(',');
        uart_putc(C->bomba ? '1' : '0');      uart_putc(',');
        uart_putc("DLF"[C->estado]);          uart_putc(',');
        uart_putc(C->modo == MODO_AUTO ? 'A' : 'M'); uart_putc(',');
        uart_puthex8(C->alarmes);
        uart_nl();
    }
}

void prot_boot(uint8_t causa_reset, uint8_t params_da_eeprom)
{
    static const char s_rs0[] PROGMEM = "PWR";
    static const char s_rs1[] PROGMEM = "EXT";
    static const char s_rs2[] PROGMEM = "BOD";
    static const char s_rs3[] PROGMEM = "WDT";
    static const char * const NOME_RS[] PROGMEM = { s_rs0, s_rs1, s_rs2, s_rs3 };

    uart_puts_P(PSTR("\r\n!0.0 BOOT fw=" FW_VERSAO " reset="));
    bool algum = false;
    for (uint8_t i = 0; i < 4; i++) {
        if (causa_reset & (1u << i)) {
            if (algum) uart_putc('|');
            puts_tab(NOME_RS, i);
            algum = true;
        }
    }
    if (!algum) uart_putc('-');
    uart_puts_P(params_da_eeprom ? PSTR(" params=EEPROM") : PSTR(" params=PADRAO"));
    uart_nl();
    if (causa_reset & (1u << 3))
        uart_puts_P(PSTR("!0.0 ALERTA reinicio pelo watchdog\r\n"));
    if (causa_reset & (1u << 2))
        uart_puts_P(PSTR("!0.0 ALERTA reinicio por subtensao (brown-out)\r\n"));
}

void prot_botao(void)
{
    ctrl_reconhece(C);
    uart_putc('!'); put_tempo();
    uart_puts_P(PSTR(" ACK botao local\r\n"));
}
