/*
 * tanque_sim.c - Simulacao "software-in-the-loop" do firmware real
 *
 * Executa o binario AVR compilado (build/nivel.elf) no simulador simavr
 * (ATmega328P a 16 MHz, ciclo a ciclo) e fecha a malha com um modelo
 * fisico do reservatorio:
 *
 *   - balanco de massa:  A dh/dt = Q_bomba - Q_consumo
 *   - bomba com partida de 1a ordem (tau = 2 s) e fonte que pode secar
 *   - sensor MPX5010DP: Vout = Vs (0,09 P + 0,04)(1 + erro_ganho) + offset,
 *     P = rho g h, com ruido eletrico branco, interferencia de 60 Hz e
 *     turbulencia colorida (mais intensa quando ha entrada de agua)
 *   - falhas injetaveis: fonte seca, sensor aberto / em curto, rele colado
 *
 * O firmware "ve" apenas o pino ADC0, a USART e os pinos de E/S, como no
 * hardware. Saidas: <cenario>_verdade.csv (grandezas fisicas a 10 Hz),
 * <cenario>_serial.log (tudo o que o firmware enviou, com carimbo de
 * tempo) e <cenario>_resumo.txt (ocupacao da CPU etc.).
 *
 * Uso: tanque_sim <firmware.elf> <cenario> <dir_saida>
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include <simavr/sim_avr.h>
#include <simavr/sim_elf.h>
#include <simavr/avr_adc.h>
#include <simavr/avr_uart.h>
#include <simavr/avr_ioport.h>

#define F_CPU_HZ 16000000.0
#define DT_MODELO 0.01                 /* integracao a 100 Hz */

/* ------------------------------------------------------------------ */
/* Gerador de numeros aleatorios reprodutivel (xorshift64 + Box-Muller) */
/* ------------------------------------------------------------------ */
static uint64_t rng = 0x9E3779B97F4A7C15ull;
static double uniforme(void)
{
    rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17;
    return ((rng >> 11) + 0.5) * (1.0 / 9007199254740992.0);
}
static double gauss(void)
{
    return sqrt(-2.0 * log(uniforme())) * cos(2.0 * M_PI * uniforme());
}

/* ------------------------------------------------------------------ */
/* Modelo do reservatorio                                              */
/* ------------------------------------------------------------------ */
typedef enum { SENS_NORMAL = 0, SENS_ABERTO, SENS_CURTO_VCC } sensor_modo_t;

static struct {
    double area_m2;          /* secao transversal                         */
    double qp_lmin;          /* vazao nominal da bomba                     */
    double topo_mm;          /* altura do extravasor (transbordamento)     */
    double h_mm;             /* nivel real                                 */
    double qout_lmin;        /* consumo atual                              */
    double rot;              /* rotacao relativa da bomba (0..1)           */
    bool   fonte_ok;
    bool   rele_colado;
    bool   cmd_bomba;        /* pino PB0 */
    bool   buzzer;           /* pino PB1 */
    bool   led_alarme;       /* pino PB2 */
    sensor_modo_t sens;
    double off_mv, ganho_err;
    double sig_elet_mv;      /* ruido eletrico branco                      */
    double turb_mm;          /* estado da turbulencia (AR(1))              */
    double qin_lmin;
    double transbordado_l;
} T;

static double t_sim;

static double sensor_mv(void)
{
    if (T.sens == SENS_ABERTO)    return 0.0;      /* pull-down de 100 k */
    if (T.sens == SENS_CURTO_VCC) return 5000.0;
    double h = T.h_mm + T.turb_mm;
    if (h < 0) h = 0;
    double p_kpa = 1000.0 * 9.81 * (h / 1000.0) / 1000.0;
    double v = 5.0 * (0.09 * p_kpa + 0.04) * (1.0 + T.ganho_err);
    double mv = v * 1000.0 + T.off_mv
              + T.sig_elet_mv * gauss()
              + 2.0 * sin(2.0 * M_PI * 60.0 * t_sim);   /* 60 Hz */
    if (mv < 0) mv = 0;
    if (mv > 5000) mv = 5000;
    return mv;
}

static void passo_modelo(double dt)
{
    bool ligada = T.cmd_bomba || T.rele_colado;
    double alvo = ligada ? 1.0 : 0.0;
    T.rot += (alvo - T.rot) * (dt / 2.0);                /* tau = 2 s */
    T.qin_lmin = T.fonte_ok ? T.qp_lmin * T.rot : 0.0;

    double qout = (T.h_mm > 0.5) ? T.qout_lmin : 0.0;    /* vazio: sem consumo */
    double dh = ((T.qin_lmin - qout) / 60.0) / T.area_m2 * dt;  /* mm */
    T.h_mm += dh;
    if (T.h_mm < 0) T.h_mm = 0;
    if (T.h_mm > T.topo_mm) {
        T.transbordado_l += (T.h_mm - T.topo_mm) * T.area_m2;  /* mm*m2 = L */
        T.h_mm = T.topo_mm;
    }

    /* turbulencia: ruido colorido, tau = 0,2 s; desvio de 3 mm com
     * entrada de agua e 0,5 mm sem ela */
    double sig = 0.5 + 2.5 * (T.qin_lmin / T.qp_lmin);
    double a = exp(-dt / 0.2);
    T.turb_mm = a * T.turb_mm + sqrt(1.0 - a * a) * sig * gauss();
}

/* ------------------------------------------------------------------ */
/* Cenarios                                                            */
/* ------------------------------------------------------------------ */
typedef enum {
    EV_CMD, EV_CONSUMO, EV_FONTE, EV_SENSOR, EV_RELE, EV_BOTAO,
    EV_NIVEL, EV_RESET, EV_FIM
} ev_tipo_t;

typedef struct {
    double     t;
    ev_tipo_t  tipo;
    double     v;
    const char *s;
} evento_t;

typedef struct {
    const char *nome;
    double h0, area, qp, topo, off_mv, ganho_err;
    const evento_t *ev;
} cenario_t;

#define CMD(t, s)   { t, EV_CMD, 0, s }
#define CONS(t, q)  { t, EV_CONSUMO, q, NULL }
#define FONTE(t, b) { t, EV_FONTE, b, NULL }
#define SENS(t, m)  { t, EV_SENSOR, m, NULL }
#define RELE(t, b)  { t, EV_RELE, b, NULL }
#define BOTAO(t)    { t, EV_BOTAO, 0, NULL }
#define NIVEL(t, h) { t, EV_NIVEL, h, NULL }
#define RESET(t)    { t, EV_RESET, 0, NULL }
#define FIM(t)      { t, EV_FIM, 0, NULL }

static const evento_t ev_normal[] = {
    CONS(0, 2.0), CMD(0.5, "SET TELEM 1"),
    CONS(600, 4.0), CONS(1200, 0.0), CONS(1500, 3.0),
    CMD(2390, "STATUS"),
    FIM(2400)
};
static const evento_t ev_hist_a[] = {
    CONS(0, 3.0), CMD(0.5, "SET TELEM 1 TCONF 0 TMINOFF 0 LIGA 300 DESLIGA 301"), FIM(900)
};
static const evento_t ev_hist_b[] = {
    CONS(0, 3.0), CMD(0.5, "SET TELEM 1 LIGA 300 DESLIGA 301"), FIM(900)
};
static const evento_t ev_hist_c[] = {
    CONS(0, 3.0), CMD(0.5, "SET TELEM 1 LIGA 280 DESLIGA 320"), FIM(900)
};
static const evento_t ev_abast[] = {
    CONS(0, 2.0), CMD(0.5, "SET TELEM 1 TRETRY 60"),
    FONTE(60, 0),                 /* cisterna seca */
    CMD(330, "STATUS"),
    FONTE(500, 1),                /* fonte restabelecida */
    BOTAO(550),                   /* operador reconhece no painel */
    FIM(900)
};
static const evento_t ev_sensor[] = {
    CONS(0, 3.0), CMD(0.5, "SET TELEM 1"),
    SENS(150, SENS_ABERTO), SENS(200, SENS_NORMAL),
    SENS(350, SENS_CURTO_VCC), SENS(370, SENS_NORMAL),
    FIM(600)
};
static const evento_t ev_manual[] = {
    CONS(0, 1.0), CMD(0.5, "SET TELEM 1"),
    CMD(5, "MODO MANUAL"), CMD(6, "BOMBA LIGA"),
    CMD(130, "BOMBA LIGA"),       /* recusado: critico alto */
    CMD(200, "STATUS"),
    CMD(250, "MODO AUTO"),
    FIM(300)
};
static const evento_t ev_rele[] = {
    CONS(0, 1.0), CMD(0.5, "SET TELEM 1"),
    RELE(50, 1),                  /* contato soldado (invisivel ao firmware) */
    CMD(300, "ACK"),
    FIM(400)
};
static const evento_t ev_config[] = {
    CONS(0, 0.0),
    CMD(0.3, "MODO MANUAL"),
    CMD(1, "VERSAO"),
    CMD(2, "STATUS"),             /* sem calibracao, tanque vazio */
    NIVEL(3, 400), CMD(4, "STATUS"),
    NIVEL(5, 0),   CMD(6, "CAL ZERO"),
    NIVEL(7, 400), CMD(8, "CAL 400"),
    NIVEL(9, 250), CMD(10, "STATUS"),
    CMD(11, "SET LIGA 450"),      /* viola a ordem: rejeitado */
    CMD(12, "SET LIGA 180 DESLIGA 400"),
    CMD(13, "SET TMAX 99999"),    /* fora da faixa */
    CMD(14, "SALVAR"),
    RESET(15),
    CMD(17, "PARAMS"),
    CMD(19, "STATUS"),
    CMD(20, "AJUDA"),
    FIM(22)
};

static const cenario_t CENARIOS[] = {
    /* nome            h0   area  qp  topo off  ganho  eventos */
    { "normal",        150, 0.05, 6, 520, 0,  0,     ev_normal },
    { "hist_a",        310, 0.05, 6, 520, 0,  0,     ev_hist_a },
    { "hist_b",        310, 0.05, 6, 520, 0,  0,     ev_hist_b },
    { "hist_c",        310, 0.05, 6, 520, 0,  0,     ev_hist_c },
    { "abastecimento", 250, 0.05, 6, 520, 0,  0,     ev_abast  },
    { "sensor",        300, 0.05, 6, 520, 0,  0,     ev_sensor },
    { "manual",        300, 0.05, 6, 520, 0,  0,     ev_manual },
    { "rele",          150, 0.05, 6, 520, 0,  0,     ev_rele   },
    { "config",          0, 0.05, 6, 520, 60, 0.04,  ev_config },
};

/* ------------------------------------------------------------------ */
/* Ligacao com o simavr                                                */
/* ------------------------------------------------------------------ */
static avr_t *avr;
static FILE *f_serial;
static char lbuf[512];
static size_t lpos;

static void cb_uart(struct avr_irq_t *irq, uint32_t v, void *p)
{
    (void)irq; (void)p;
    char c = (char)v;
    if (c == '\r') return;
    if (c == '\n' || lpos >= sizeof lbuf - 1) {
        lbuf[lpos] = 0;
        fprintf(f_serial, "[%9.3f] %s\n", t_sim, lbuf);
        lpos = 0;
        return;
    }
    lbuf[lpos++] = c;
}

static void cb_adc_trigger(struct avr_irq_t *irq, uint32_t v, void *p)
{
    (void)irq; (void)v; (void)p;
    avr_raise_irq(avr_io_getirq(avr, AVR_IOCTL_ADC_GETIRQ, ADC_IRQ_ADC0),
                  (uint32_t)lround(sensor_mv()));
}

static void cb_pb(struct avr_irq_t *irq, uint32_t v, void *p)
{
    (void)irq;
    *(bool *)p = v != 0;
}

static void envia_uart(const char *s)
{
    avr_irq_t *in = avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_INPUT);
    fprintf(f_serial, "[%9.3f] >>> %s\n", t_sim, s);
    while (*s) avr_raise_irq(in, (uint8_t)*s++);
    avr_raise_irq(in, '\r');
}

/* simavr, por padrao, dorme em tempo real enquanto a CPU simulada esta
 * em SLEEP; aqui o tempo simulado avanca sem esperar. */
static void sono_instantaneo(avr_t *a, avr_cycle_count_t quanto)
{
    (void)a; (void)quanto;
}

static void botao(int pressionado)
{
    avr_raise_irq(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('D'), 2), pressionado ? 0 : 1);
}

int main(int argc, char **argv)
{
    if (argc < 4) {
        fprintf(stderr, "uso: %s firmware.elf cenario dir_saida\n", argv[0]);
        return 2;
    }
    const cenario_t *cn = NULL;
    for (size_t i = 0; i < sizeof CENARIOS / sizeof CENARIOS[0]; i++)
        if (!strcmp(CENARIOS[i].nome, argv[2])) cn = &CENARIOS[i];
    if (!cn) { fprintf(stderr, "cenario desconhecido: %s\n", argv[2]); return 2; }

    char path[512];
    snprintf(path, sizeof path, "%s/%s_serial.log", argv[3], cn->nome);
    f_serial = fopen(path, "w");
    snprintf(path, sizeof path, "%s/%s_verdade.csv", argv[3], cn->nome);
    FILE *f_csv = fopen(path, "w");
    if (!f_serial || !f_csv) { perror("fopen"); return 1; }
    fprintf(f_csv, "t,h_mm,cmd_bomba,rotacao,qin_lmin,qout_lmin,sensor_mv,buzzer,led_alarme,transbordado_l\n");

    /* modelo */
    memset(&T, 0, sizeof T);
    T.area_m2 = cn->area; T.qp_lmin = cn->qp; T.topo_mm = cn->topo;
    T.h_mm = cn->h0; T.fonte_ok = true; T.sens = SENS_NORMAL;
    T.off_mv = cn->off_mv; T.ganho_err = cn->ganho_err; T.sig_elet_mv = 4.0;

    /* MCU */
    elf_firmware_t fw;
    memset(&fw, 0, sizeof fw);
    if (elf_read_firmware(argv[1], &fw)) { fprintf(stderr, "erro lendo %s\n", argv[1]); return 1; }
    avr = avr_make_mcu_by_name("atmega328p");
    if (!avr) { fprintf(stderr, "MCU nao suportado\n"); return 1; }
    avr_init(avr);
    avr->log = LOG_WARNING;
    avr_load_firmware(avr, &fw);
    avr->frequency = (uint32_t)F_CPU_HZ;
    avr->vcc = avr->avcc = avr->aref = 5000;
    avr->sleep = sono_instantaneo;

    uint32_t flags = 0;
    avr_ioctl(avr, AVR_IOCTL_UART_GET_FLAGS('0'), &flags);
    flags &= ~AVR_UART_FLAG_STDIO;
    avr_ioctl(avr, AVR_IOCTL_UART_SET_FLAGS('0'), &flags);

    avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_UART_GETIRQ('0'), UART_IRQ_OUTPUT), cb_uart, NULL);
    avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_ADC_GETIRQ, ADC_IRQ_OUT_TRIGGER), cb_adc_trigger, NULL);
    avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('B'), 0), cb_pb, &T.cmd_bomba);
    avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('B'), 1), cb_pb, &T.buzzer);
    avr_irq_register_notify(avr_io_getirq(avr, AVR_IOCTL_IOPORT_GETIRQ('B'), 2), cb_pb, &T.led_alarme);
    botao(0);

    double t_modelo = 0, t_log = 0, t_soltar = -1;
    uint64_t ciclos_ativos = 0, ciclos_ativos_regime = 0;
    double t_regime = 30.0;            /* ocupacao medida apos o boot */
    uint64_t ciclo_regime = 0;
    size_t iev = 0;
    double t_fim = 1e9;
    for (const evento_t *e = cn->ev; ; e++) if (e->tipo == EV_FIM) { t_fim = e->t; break; }

    int estado = cpu_Running;
    while (t_sim < t_fim) {
        int antes = avr->state;
        avr_cycle_count_t c0 = avr->cycle;
        estado = avr_run(avr);
        /* no simavr, a chamada que executa SLEEP ja soma o tempo dormido:
         * so conta ciclos de chamadas em que a CPU continuou acordada */
        if (antes == cpu_Running && avr->state == cpu_Running) {
            ciclos_ativos += avr->cycle - c0;
            if (t_sim >= t_regime) ciclos_ativos_regime += avr->cycle - c0;
        }
        if (estado == cpu_Done || estado == cpu_Crashed) {
            fprintf(stderr, "CPU parou (estado %d) em t=%.3f\n", estado, t_sim);
            break;
        }
        t_sim = (double)avr->cycle / F_CPU_HZ;
        if (!ciclo_regime && t_sim >= t_regime) ciclo_regime = avr->cycle;

        while (t_modelo + DT_MODELO <= t_sim) {
            passo_modelo(DT_MODELO);
            t_modelo += DT_MODELO;
        }
        if (t_sim >= t_log) {
            fprintf(f_csv, "%.2f,%.3f,%d,%.3f,%.3f,%.3f,%.1f,%d,%d,%.3f\n",
                    t_sim, T.h_mm, T.cmd_bomba, T.rot, T.qin_lmin,
                    (T.h_mm > 0.5 ? T.qout_lmin : 0.0), sensor_mv(),
                    T.buzzer, T.led_alarme, T.transbordado_l);
            t_log += 0.1;
        }
        if (t_soltar > 0 && t_sim >= t_soltar) { botao(0); t_soltar = -1; }

        while (cn->ev[iev].tipo != EV_FIM && t_sim >= cn->ev[iev].t) {
            const evento_t *e = &cn->ev[iev++];
            switch (e->tipo) {
            case EV_CMD:     envia_uart(e->s); break;
            case EV_CONSUMO: T.qout_lmin = e->v;
                             fprintf(f_serial, "[%9.3f] *** consumo = %.1f L/min\n", t_sim, e->v); break;
            case EV_FONTE:   T.fonte_ok = e->v != 0;
                             fprintf(f_serial, "[%9.3f] *** fonte %s\n", t_sim, T.fonte_ok ? "restabelecida" : "SECA"); break;
            case EV_SENSOR:  T.sens = (sensor_modo_t)(int)e->v;
                             fprintf(f_serial, "[%9.3f] *** sensor %s\n", t_sim,
                                     T.sens == SENS_NORMAL ? "normal" : T.sens == SENS_ABERTO ? "ABERTO" : "EM CURTO COM VCC"); break;
            case EV_RELE:    T.rele_colado = e->v != 0;
                             fprintf(f_serial, "[%9.3f] *** rele %s\n", t_sim, T.rele_colado ? "COLADO" : "normal"); break;
            case EV_BOTAO:   botao(1); t_soltar = t_sim + 0.2;
                             fprintf(f_serial, "[%9.3f] *** botao pressionado\n", t_sim); break;
            case EV_NIVEL:   T.h_mm = e->v;
                             fprintf(f_serial, "[%9.3f] *** nivel ajustado para %.0f mm\n", t_sim, e->v); break;
            case EV_RESET:   avr_reset(avr);
                             /* o reset zera PIND, mas a IRQ do simavr guarda o
                              * ultimo valor e filtra repeticoes: forca 0 -> 1
                              * (pull-up do botao solto) */
                             botao(1); botao(0);
                             fprintf(f_serial, "[%9.3f] *** RESET do microcontrolador\n", t_sim); break;
            default: break;
            }
        }
    }

    double dur_regime = (double)(avr->cycle - ciclo_regime) / F_CPU_HZ;
    snprintf(path, sizeof path, "%s/%s_resumo.txt", argv[3], cn->nome);
    FILE *f_res = fopen(path, "w");
    fprintf(f_res, "cenario=%s\n", cn->nome);
    fprintf(f_res, "tempo_simulado_s=%.3f\n", t_sim);
    fprintf(f_res, "ciclos_totais=%llu\n", (unsigned long long)avr->cycle);
    fprintf(f_res, "ciclos_ativos=%llu\n", (unsigned long long)ciclos_ativos);
    fprintf(f_res, "ocupacao_cpu_total_pct=%.4f\n", 100.0 * ciclos_ativos / (double)avr->cycle);
    if (dur_regime > 0)
        fprintf(f_res, "ocupacao_cpu_regime_pct=%.4f\n",
                100.0 * ciclos_ativos_regime / (dur_regime * F_CPU_HZ));
    fprintf(f_res, "transbordado_l=%.3f\n", T.transbordado_l);
    fprintf(f_res, "estado_final_cpu=%d\n", estado);
    fclose(f_res);
    fclose(f_csv);
    fclose(f_serial);
    avr_terminate(avr);
    return 0;
}
