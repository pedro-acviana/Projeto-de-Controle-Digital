/*
 * proj1cdig.c (versão AUTO) — firmware do Prj1CDig para o proj1cdig_auto.py
 *
 * Igual ao da pasta proj1cdig, mais comandos em MAIÚSCULA que mudam parâmetros em tempo
 * real (U, T, C, W). Assim o script ajusta calibração, saturação, cooler e Ts sem regravar.
 * Os valores voltam aos do proj1cdig_config.h se a placa reiniciar; o script os reenvia.
 *
 * ENE0167 Controle Digital · UnB · ESP32-WROOM-32 (ADC interno) + 2x LM35 + transistores
 *
 * Um único firmware para todas as fases. O modo é escolhido por comando no serial
 * (115200 8N1, uma linha por comando). Digite "h" para ver a lista.
 *
 * Saída: linhas CSV começando com "D," — lidas pelo proj1cdig_logger.py.
 *   D,t_s,k,modo,u,w,tp,ta,ref,aw
 *     t_s  tempo desde o boot [s]         k    índice da amostra no modo atual
 *     modo IDLE|SENS|OPEN|PRBS|CTRL       u    aquecedor [contagens 0..1023]
 *     w    cooler [contagens]             tp,ta temperaturas [°C]
 *     ref  referência [°C] (só CTRL)      aw   anti-windup 1/0
 * Linhas que começam com "#" são mensagens.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_random.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "proj1cdig_config.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ======================================================================= */
/* Estado compartilhado (protegido por g_mtx)                              */
/* ======================================================================= */
typedef enum { M_IDLE, M_SENS, M_OPEN, M_PRBS, M_CTRL } mode_t_;
static const char *MODE_NAME[] = { "IDLE", "SENS", "OPEN", "PRBS", "CTRL" };

typedef enum { W_OFF, W_CONST, W_SIN, W_TRI } wmode_t_;

static struct {
    mode_t_  mode;
    bool     restart;      /* comando novo: reinicia contadores do modo */
    int      u_open;       /* OPEN: duty fixo */
    int      prbs_u0, prbs_du, prbs_duracao_bit_s, prbs_tempo_aquecimento_s;
    float    ref;          /* CTRL */
    bool     aw;           /* anti-windup ligado */
    float    K, z0;        /* PI */
    wmode_t_ wmode;
    int      w_const;
    int64_t  w_t0_us;      /* início da perturbação periódica */
    bool     ensaio;       /* ensaio do roteiro (comando e) em andamento */
    float    e_lo, e_hi;   /* níveis da onda quadrada de referência */
    int      e_meio;       /* meio período da onda quadrada [s] */
} g = {
    .mode = M_IDLE, .restart = true,
    .prbs_u0 = PRBS_DEFAULT_U0, .prbs_du = PRBS_DEFAULT_DU,
    .prbs_duracao_bit_s = PRBS_DEFAULT_BIT_S, .prbs_tempo_aquecimento_s = PRBS_DEFAULT_T_AQUEC_S,
    .ref = GROUP_PO_C, .aw = true, .K = PI_K, .z0 = PI_Z0,
    .wmode = W_OFF, .e_meio = ENSAIO_MEIO_PERIODO_S,
};
static SemaphoreHandle_t g_mtx;
#define LOCK()   xSemaphoreTake(g_mtx, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(g_mtx)

static volatile int g_w_now = 0;   /* duty atual do cooler, para o log */

/* Parâmetros ajustáveis em tempo real (comandos U, T, C, W) */
static volatile int   s_umax  = U_MAX;
static volatile int   s_ts_ms = CTRL_TS_MS;
static volatile float s_cal[4] = { CAL_TP_GAIN, CAL_TP_OFFSET, CAL_TA_GAIN, CAL_TA_OFFSET };
static volatile int   s_w_off = COOLER_OFFSET, s_w_amp = COOLER_AMP;
static volatile float s_w0 = GROUP_W0_RAD_S;

/* ======================================================================= */
/* PWM                                                                     */
/* ======================================================================= */
static void pwm_init(void)
{
    ledc_timer_config_t th = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_16_BIT, .freq_hz = HEATER_FREQ_HZ,   /* aquecedor: 16 bits */
        .clk_cfg = LEDC_AUTO_CLK };
    ESP_ERROR_CHECK(ledc_timer_config(&th));
    ledc_timer_config_t tc = th;
    tc.timer_num = LEDC_TIMER_1; tc.freq_hz = COOLER_FREQ_HZ;
    tc.duty_resolution = LEDC_TIMER_10_BIT;                                  /* cooler: 10 bits a 25 kHz */
    ESP_ERROR_CHECK(ledc_timer_config(&tc));

    ledc_channel_config_t ch = {
        .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0, .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PIN_PWM_HEATER, .duty = 0, .hpoint = 0 };
    ESP_ERROR_CHECK(ledc_channel_config(&ch));
    ch.channel = LEDC_CHANNEL_1; ch.timer_sel = LEDC_TIMER_1; ch.gpio_num = PIN_PWM_COOLER;
    ESP_ERROR_CHECK(ledc_channel_config(&ch));
}

static int clampi(int v, int lo, int hi) { return v < lo ? lo : (v > hi ? hi : v); }

static void heater_set(int u)
{
    u = clampi(u, U_MIN, s_umax);             /* limite físico, sempre */
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, u);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void cooler_set(int w)
{
    w = clampi(w, 0, PWM_FULL);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, w);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    g_w_now = w;
}

/* ======================================================================= */
/* ADC interno da ESP32 (ADC1: continua funcionando com o WiFi ligado)      */
/* ======================================================================= */
/* O "filtro" da bancada (75 Ω + 1 µF) é o damper da Fig. 13 do datasheet do LM35: protege o
 * sensor contra a capacitância do fio, mas quase não filtra (canto em ~2 kHz). A filtragem é
 * digital: cada amostra é a média de ADC_JANELA_MS ms de leituras. 400 ms contém um número
 * inteiro de ciclos de 50 e de 60 Hz, que se cancelam na média; e a fase de cada rajada é
 * sorteada para o PWM do aquecedor não ficar "congelado" sempre no mesmo ponto.            */
static adc_oneshot_unit_handle_t s_adc;
static adc_channel_t s_ch_tp, s_ch_ta;
static float s_mv_a = 1250.0f / 4095.0f, s_mv_b = 0.0f;   /* mV = a·raw + b */
static bool s_adc_ok = false;
static esp_err_t adc_init(void)
{
    adc_unit_t u1, u2;
    esp_err_t e = adc_oneshot_io_to_channel(PIN_ADC_TP, &u1, &s_ch_tp);
    if (e == ESP_OK) e = adc_oneshot_io_to_channel(PIN_ADC_TA, &u2, &s_ch_ta);
    if (e != ESP_OK) return e;
    if (u1 != ADC_UNIT_1 || u2 != ADC_UNIT_1) return ESP_ERR_INVALID_ARG;   /* ADC2 conflita com WiFi */
    adc_oneshot_unit_init_cfg_t uc = { .unit_id = ADC_UNIT_1, .ulp_mode = ADC_ULP_MODE_DISABLE };
    if ((e = adc_oneshot_new_unit(&uc, &s_adc)) != ESP_OK) return e;
    adc_oneshot_chan_cfg_t cc = { .atten = ADC_ATENUACAO, .bitwidth = ADC_BITWIDTH_12 };
    if ((e = adc_oneshot_config_channel(s_adc, s_ch_tp, &cc)) != ESP_OK) return e;
    if ((e = adc_oneshot_config_channel(s_adc, s_ch_ta, &cc)) != ESP_OK) return e;

    /* Calibração de fábrica (eFuse), que no ESP32 é uma reta: extraímos a e b para aplicar
     * na MÉDIA em ponto flutuante, sem perder a resolução que a média ganhou. */
    bool fabrica = false;
#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    adc_cali_line_fitting_config_t lc = {
        .unit_id = ADC_UNIT_1, .atten = ADC_ATENUACAO, .bitwidth = ADC_BITWIDTH_12,
#if CONFIG_IDF_TARGET_ESP32
        .default_vref = 1100,
#endif
    };
    adc_cali_handle_t cal = NULL;
    if (adc_cali_create_scheme_line_fitting(&lc, &cal) == ESP_OK) {
        int v0 = 0, v1 = 0;
        adc_cali_raw_to_voltage(cal, 500, &v0);
        adc_cali_raw_to_voltage(cal, 3500, &v1);
        s_mv_a = (v1 - v0) / 3000.0f;
        s_mv_b = v0 - s_mv_a * 500.0f;
        adc_cali_delete_scheme_line_fitting(cal);
        fabrica = true;
    }
#endif
    if (fabrica) printf("#ADC calibracao de fabrica: mV = %.5f*raw %+.2f\n", s_mv_a, s_mv_b);
    else         printf("#ADC sem calibracao de fabrica: reta nominal (a calibracao de 2 pontos corrige)\n");
    s_adc_ok = true;
    return ESP_OK;
}

static void read_temps(float *tp, float *ta)
{
    int64_t sp = 0, sa = 0;
    int np = 0, na = 0;
    if (s_adc_ok) {
        TickType_t t = xTaskGetTickCount();
        for (int i = 0; i < ADC_JANELA_MS; i++) {
            esp_rom_delay_us(esp_random() % 850);          /* fase sorteada ao longo do período de 1 ms do PWM de 1 kHz */
            for (int j = 0; j < ADC_LEITURAS_POR_MS; j++) {
                int r;
                if (adc_oneshot_read(s_adc, s_ch_tp, &r) == ESP_OK) { sp += r; np++; }
                if (adc_oneshot_read(s_adc, s_ch_ta, &r) == ESP_OK) { sa += r; na++; }
            }
            xTaskDelayUntil(&t, pdMS_TO_TICKS(1));
        }
    }
    float vp = np ? s_mv_a * ((float)sp / np) + s_mv_b : NAN;
    float va = na ? s_mv_a * ((float)sa / na) + s_mv_b : NAN;
    *tp = s_cal[0] * (vp / 10.0f) + s_cal[1];
    *ta = s_cal[2] * (va / 10.0f) + s_cal[3];
}

/* ======================================================================= */
/* PRBS — LFSR de 9 bits, x^9 + x^5 + 1, sequência-m de 511 bits           */
/* ======================================================================= */
static uint16_t s_lfsr = 0x1A5;
static int prbs_next_bit(void)
{
    int b = ((s_lfsr >> 0) ^ (s_lfsr >> 4)) & 1;
    s_lfsr = (s_lfsr >> 1) | (b << 8);
    return s_lfsr & 1;
}

/* ======================================================================= */
/* PI discreto anti-windup                                                  */
/*   C(z) = K (z - z0)/(z - 1) = K + K(1 - z0)/(z - 1)                       */
/*   u(k)   = K e(k) + I(k)                                                  */
/*   I(k+1) = I(k) + K(1 - z0) e(k)     (congelado se saturado e AW ligado)  */
/* ======================================================================= */
typedef struct { float I; } pi_t;

static int pi_step(pi_t *pi, float e, float K, float z0, bool aw, bool *sat)
{
    float u = K * e + pi->I;
    bool hi = u > s_umax, lo = u < U_MIN;
    *sat = hi || lo;
    /* integração condicional: só para de integrar se o erro empurra mais para a saturação */
    bool freeze = aw && ((hi && e > 0) || (lo && e < 0));
    if (!freeze) pi->I += K * (1.0f - z0) * e;
    if (hi) u = s_umax;
    if (lo) u = U_MIN;
    return (int)lroundf(u);
}

#define W_PERT_MAX (s_w_off + s_w_amp)   /* perturbação constante do ensaio = pico da triangular */

/* ======================================================================= */
/* Tarefa do cooler: atualiza a perturbação a cada 200 ms, independente de Ts */
/* ======================================================================= */
static void cooler_task(void *arg)
{
    for (;;) {
        LOCK();
        wmode_t_ m = g.wmode; int wc = g.w_const; int64_t t0 = g.w_t0_us;
        UNLOCK();
        double t = (esp_timer_get_time() - t0) / 1e6;
        double th = s_w0 * t;
        int w = 0;
        switch (m) {
        case W_OFF:   w = 0; break;
        case W_CONST: w = wc; break;
        case W_SIN:   w = s_w_off + (int)lround(s_w_amp * sin(th)); break;
        case W_TRI:   w = s_w_off + (int)lround(s_w_amp * (2.0 / M_PI) * asin(sin(th))); break;
        }
        cooler_set(w);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

/* ======================================================================= */
/* Tarefa principal: amostragem, modos, log                                 */
/* ======================================================================= */
static void control_task(void *arg)
{
    pi_t pi = { 0 };
    int u = 0, k = 0, prbs_u = 0, prbs_bit = 0;
    int64_t t_ens0 = 0; int fase = -1; float ise = 0; bool ise_ok = false;   /* ensaio do roteiro */
    TickType_t last = xTaskGetTickCount();

    for (;;) {
        LOCK();
        mode_t_ m = g.mode;
        bool restart = g.restart; g.restart = false;
        int u_open = g.u_open;
        int u0 = g.prbs_u0, du = g.prbs_du, duracao_bit_s = g.prbs_duracao_bit_s, tempo_aquecimento_s = g.prbs_tempo_aquecimento_s;
        float ref = g.ref, K = g.K, z0 = g.z0; bool aw = g.aw;
        bool ensaio = g.ensaio; float e_lo = g.e_lo, e_hi = g.e_hi; int e_meio = g.e_meio;
        UNLOCK();

        int period_ms = (m == M_CTRL) ? s_ts_ms : 1000;   /* Ti = 1 s; Ts no controle */

        if (restart) {
            k = 0;
            last = xTaskGetTickCount();
            if (m == M_CTRL) pi.I = (float)u;   /* partida sem solavanco: começa no u atual */
            if (m == M_PRBS) { prbs_bit = 1; prbs_u = u0; }
            if (m == M_CTRL && ensaio) { t_ens0 = esp_timer_get_time(); fase = -1; ise = 0; ise_ok = false; }
        }

        float tp, ta;
        read_temps(&tp, &ta);

        switch (m) {
        case M_IDLE:
        case M_SENS:
            u = 0; break;
        case M_OPEN:
            u = u_open; break;
        case M_PRBS: {
            int t = k;                           /* k em segundos (período 1 s) */
            if (t < tempo_aquecimento_s) prbs_u = u0;
            else if ((t - tempo_aquecimento_s) % duracao_bit_s == 0) {
                prbs_bit = prbs_next_bit();
                prbs_u = prbs_bit ? u0 + du : u0 - du;
            }
            if (t >= tempo_aquecimento_s + 511 * duracao_bit_s) {     /* sequência completa: volta ao P.O. */
                prbs_u = u0;
                if (t == tempo_aquecimento_s + 511 * duracao_bit_s) printf("#PRBS concluido em k=%d\n", k);
            }
            u = prbs_u;
            break; }
        case M_CTRL: {
            bool sat;
            double te = -1;
            if (ensaio) {                        /* ensaio do roteiro: referência e cooler pelo relógio */
                te = (esp_timer_get_time() - t_ens0) / 1e6;
                if (te >= ENSAIO_T_FIM_S) {
                    ref = e_lo;
                    LOCK(); g.ensaio = false; g.ref = e_lo; g.wmode = W_OFF; UNLOCK();
                    printf("#ENSAIO concluido: ISE(%d-%d s) = %.1f\n", ISE_T0_S, ISE_T1_S, ise);
                    ensaio = false;
                } else {
                    ref = (((int)(te / e_meio)) % 2 == 0) ? e_lo : e_hi;
                    int f = te < ENSAIO_T_CONST1_S ? 0 : te < ENSAIO_T_TRI_S ? 1 : te < ENSAIO_T_CONST2_S ? 2 : 3;
                    LOCK();
                    g.ref = ref;
                    if (f != fase) {
                        g.w_t0_us = esp_timer_get_time();
                        if (f == 0) g.wmode = W_OFF;
                        else if (f == 2) g.wmode = W_TRI;
                        else { g.wmode = W_CONST; g.w_const = W_PERT_MAX; }
                    }
                    UNLOCK();
                    if (f != fase) {
                        static const char *NOME[] = { "cooler desligado", "perturbacao constante",
                                                      "perturbacao triangular", "perturbacao constante" };
                        printf("#ENSAIO t=%.0f s: %s\n", te, NOME[f]);
                        fase = f;
                    }
                }
            }
            if (isnan(tp)) break;                /* sem leitura: mantém u */
            float e = ref - tp;
            if (ensaio && te >= ISE_T0_S && te < ISE_T1_S) ise += e * e * (period_ms / 1000.0f);
            if (ensaio && te >= ISE_T1_S && !ise_ok) {
                ise_ok = true;
                printf("#ISE(%d-%d s) = %.1f\n", ISE_T0_S, ISE_T1_S, ise);
            }
            u = pi_step(&pi, e, K, z0, aw, &sat);
            break; }
        }

        /* proteção térmica */
        if (!isnan(tp) && tp > T_TRIP_C && u > 0) {
            u = 0;
            LOCK(); g.mode = M_IDLE; g.restart = true; UNLOCK();
            printf("#TRIP Tp=%.2f > %.1f: aquecedor desligado, modo IDLE\n", tp, T_TRIP_C);
        }
        heater_set(u);

        if (m != M_IDLE)
            printf("D,%.3f,%d,%s,%d,%d,%.3f,%.3f,%.2f,%d\n",
                   esp_timer_get_time() / 1e6, k, MODE_NAME[m], clampi(u, U_MIN, s_umax),
                   g_w_now, tp, ta, (m == M_CTRL) ? ref : NAN, aw ? 1 : 0);
        k++;
        xTaskDelayUntil(&last, pdMS_TO_TICKS(period_ms));
    }
}

/* ======================================================================= */
/* Comandos                                                                 */
/* ======================================================================= */
static void help(void)
{
    printf("#--- proj1cdig: comandos (uma linha cada) ---\n"
           "# x                      para tudo (aquecedor e cooler em 0)\n"
           "# s                      so le sensores, aquecedor em 0 (log 1 s)\n"
           "# o <u>                  malha aberta, aquecedor fixo em u (0..%d)\n"
           "# p [u0 du duracao_bit_s tempo_aquecimento_s]\n"
           "#                        PRBS: fica tempo_aquecimento_s em u0, depois alterna\n"
           "#                        u0+du / u0-du, cada nivel durando duracao_bit_s\n"
           "# c [ref]                malha fechada PI, Ts=%d ms (ref padrao %.1f)\n"
           "# r <ref>                degrau de referencia (em CTRL)\n"
           "# a <0|1>                anti-windup desligado/ligado\n"
           "# e [r_baixa r_alta meio_periodo_s]\n"
           "#                        ensaio do roteiro (Fig. 5): onda quadrada (padrao P.O. / P.O.+10,\n"
           "#                        400 s) + perturbacao constante e triangular, 3600 s, calcula o ISE\n"
           "# k <K> <z0>             troca o PI sem regravar\n"
           "# w0 | wc <w> | ws | wt  cooler: off | constante | senoide | triangular\n"
           "# --- ajustes em tempo real (usados pelo proj1cdig_auto.py) ---\n"
           "# U <umax>               saturacao do aquecedor (1..%d)\n"
           "# T <ms>                 periodo do controle Ts (200..20000 ms)\n"
           "# C <gp> <op> <ga> <oa>  calibracao: T = g*(mV/10) + o, para Tp e Ta\n"
           "# W <offset> <amp>       senoide/triangular do cooler\n"
           "# F <w0>                 frequencia da perturbacao [rad/s]\n"
           "# h                      esta ajuda\n",
           U_MAX, CTRL_TS_MS, GROUP_PO_C, U_MAX_HARD);
}

static void print_state(void)
{
    LOCK();
    printf("#ESTADO modo=%s ref=%.2f K=%.4f z0=%.5f aw=%d prbs=[%d %d %d %d] wmode=%d ens=%d "
           "umax=%d ts=%d cal=[%.5f %.4f %.5f %.4f] wcfg=[%d %d] w0=%.4f\n",
           MODE_NAME[g.mode], g.ref, g.K, g.z0, g.aw, g.prbs_u0, g.prbs_du,
           g.prbs_duracao_bit_s, g.prbs_tempo_aquecimento_s, g.wmode, g.ensaio,
           s_umax, s_ts_ms, s_cal[0], s_cal[1], s_cal[2], s_cal[3], s_w_off, s_w_amp, s_w0);
    UNLOCK();
}

static void handle(char *line)
{
    char c = line[0], c2 = line[1];
    char *args = line + 1;
    LOCK();
    if (c && strchr("xsopcrw", c)) g.ensaio = false;   /* qualquer outro modo/ajuste manual encerra o ensaio */
    switch (c) {
    case 'x': g.mode = M_IDLE; g.restart = true; g.wmode = W_OFF; break;
    case 's': g.mode = M_SENS; g.restart = true; break;
    case 'o': g.u_open = clampi(atoi(args), 0, s_umax); g.mode = M_OPEN; g.restart = true; break;
    case 'p': {
        int a[4] = { g.prbs_u0, g.prbs_du, g.prbs_duracao_bit_s, g.prbs_tempo_aquecimento_s };
        sscanf(args, "%d %d %d %d", &a[0], &a[1], &a[2], &a[3]);
        if (a[0] + a[1] > s_umax || a[0] - a[1] < 0 || a[2] < 1 || a[3] < 0) {
            printf("#ERRO PRBS fora da faixa 0..%d\n", s_umax); break;
        }
        g.prbs_u0 = a[0]; g.prbs_du = a[1]; g.prbs_duracao_bit_s = a[2]; g.prbs_tempo_aquecimento_s = a[3];
        g.mode = M_PRBS; g.restart = true;
        printf("#PRBS u0=%d du=%d duracao_bit=%d s tempo_aquecimento=%d s, duracao total %.2f h\n",
               a[0], a[1], a[2], a[3], (a[3] + 511.0 * a[2]) / 3600.0);
        break; }
    case 'c': { float r; if (sscanf(args, "%f", &r) == 1) g.ref = r;
                g.mode = M_CTRL; g.restart = true; break; }
    case 'r': { float r; if (sscanf(args, "%f", &r) == 1) g.ref = r; break; }
    case 'a': g.aw = atoi(args) != 0; break;
    case 'e': {
        float a[3] = { GROUP_PO_C, GROUP_PO_C + ENSAIO_AMPL_C, ENSAIO_MEIO_PERIODO_S };
        sscanf(args, "%f %f %f", &a[0], &a[1], &a[2]);
        if (a[2] < 10) { printf("#ERRO meio periodo menor que 10 s\n"); break; }
        g.e_lo = a[0]; g.e_hi = a[1]; g.e_meio = (int)a[2]; g.ref = a[0];
        g.ensaio = true; g.wmode = W_OFF; g.mode = M_CTRL; g.restart = true;
        printf("#ENSAIO iniciado: ref %.1f/%.1f C, meio periodo %d s, %d s no total, ISE entre %d e %d s\n",
               a[0], a[1], (int)a[2], ENSAIO_T_FIM_S, ISE_T0_S, ISE_T1_S);
        break; }
    case 'k': { float K, z; if (sscanf(args, "%f %f", &K, &z) == 2) { g.K = K; g.z0 = z; } break; }
    case 'w':
        g.w_t0_us = esp_timer_get_time();
        if (c2 == '0') g.wmode = W_OFF;
        else if (c2 == 'c') { g.w_const = clampi(atoi(line + 2), 0, PWM_FULL); g.wmode = W_CONST; }
        else if (c2 == 's') g.wmode = W_SIN;
        else if (c2 == 't') g.wmode = W_TRI;
        break;
    case 'U': { int v = atoi(args);
        if (v >= 1 && v <= U_MAX_HARD) s_umax = v;
        else printf("#ERRO U fora de 1..%d (U_MAX_HARD no config.h)\n", U_MAX_HARD);
        break; }
    case 'T': { int v = atoi(args);
        if (v >= 200 && v <= 20000) { s_ts_ms = v; } else printf("#ERRO T fora de 200..20000 ms\n");
        break; }
    case 'C': { float a[4];
        if (sscanf(args, "%f %f %f %f", &a[0], &a[1], &a[2], &a[3]) == 4)
            for (int i = 0; i < 4; i++) s_cal[i] = a[i];
        else printf("#ERRO C precisa de 4 numeros\n");
        break; }
    case 'W': { int o, a;
        if (sscanf(args, "%d %d", &o, &a) == 2 && o - a >= 0 && o + a <= PWM_FULL) { s_w_off = o; s_w_amp = a; }
        else printf("#ERRO W: offset-amp >= 0 e offset+amp <= %d\n", PWM_FULL);
        break; }
    case 'F': { float v = atof(args);
        if (v > 0.001f && v < 1.0f) s_w0 = v; else printf("#ERRO F fora de 0,001..1 rad/s\n");
        break; }
    case 'h': case '?': UNLOCK(); help(); return;
    default: UNLOCK(); printf("#? comando desconhecido: %s (h = ajuda)\n", line); return;
    }
    UNLOCK();
    print_state();
}

static void uart_task(void *arg)
{
    char line[64]; int n = 0; uint8_t ch;
    for (;;) {
        if (uart_read_bytes(UART_NUM_0, &ch, 1, pdMS_TO_TICKS(100)) == 1) {
            if (ch == '\r' || ch == '\n') {
                if (n) { line[n] = 0; handle(line); n = 0; }
            } else if (n < (int)sizeof(line) - 1) line[n++] = (char)ch;
        }
    }
}

void app_main(void)
{
    g_mtx = xSemaphoreCreateMutex();
    pwm_init();
    heater_set(0); cooler_set(0);
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));

    printf("#proj1cdig iniciado (AUTO). U_MAX=%d Ts=%d ms PO=%.1f w0=%.3f\n",
           U_MAX, CTRL_TS_MS, GROUP_PO_C, GROUP_W0_RAD_S);
    esp_err_t e = adc_init();
    if (e != ESP_OK)
        printf("#ERRO ADC: os GPIO %d e %d precisam ser do ADC1 (32 a 39) (%s)\n",
               PIN_ADC_TP, PIN_ADC_TA, esp_err_to_name(e));
    else
        printf("#ADC ok: Tp no GPIO%d, Ta no GPIO%d\n", PIN_ADC_TP, PIN_ADC_TA);
    printf("#H,t_s,k,modo,u,w,tp,ta,ref,aw\n");
    help();

    xTaskCreate(control_task, "ctrl", 4096, NULL, 5, NULL);
    xTaskCreate(cooler_task, "cooler", 3072, NULL, 4, NULL);
    xTaskCreate(uart_task, "uart", 3072, NULL, 3, NULL);
}
