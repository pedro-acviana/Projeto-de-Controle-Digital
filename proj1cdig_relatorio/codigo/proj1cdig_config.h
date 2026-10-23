/*
 * proj1cdig_config.h — tudo o que muda de grupo para grupo ou depois de cada etapa.
 * Prj1CDig · ENE0167 Controle Digital · UnB
 *
 * Unidade do atuador: CONTAGENS do PWM de 16 bits do aquecedor (0..65535 = 0..12 V médio),
 * como pede o roteiro atualizado (passo 2: "PRBS implementado via PWM 16 bits").
 * Kp identificado (21/10/2026): parte rápida Kp·Tz/Tp1 ≈ 0,0017 °C/contagem; PI ≈ 2000 contagens/°C.
 * O cooler continua em 10 bits (0..1023): ele é perturbação, não precisa de resolução.
 */
#pragma once
#include "hal/adc_types.h"

/* ---------------- Pinos (ESP32-WROOM-32 DevKit, os "Dxx" da placa) ---------------- */
#define PIN_ADC_TP         35     /* D35 = ADC1_CH7: LM35 do processo (colado nos resistores) */
#define PIN_ADC_TA         34     /* D34 = ADC1_CH6: LM35 do ambiente */
#define PIN_PWM_HEATER     25     /* D25 → transistor do aquecedor   <<< CONFIRA ONDE ESTÁ LIGADO */
#define PIN_PWM_COOLER     26     /* D26 → transistor do cooler */
/* Não use para PWM: 34–39 (só entrada), 6–11 (flash), 0/2/5/12/15 (definem o boot), 1/3 (USB-serial). */

/* ---------------- ADC interno ---------------- */
#define ADC_ATENUACAO       ADC_ATTEN_DB_2_5  /* faixa útil ~100–1250 mV = 10–125 °C (0 dB iria só até ~95 °C) */
#define ADC_JANELA_MS       400    /* média de 400 ms por amostra: cancela 50 e 60 Hz */
#define ADC_LEITURAS_POR_MS 4      /* × 2 canais → ~1600 leituras de cada sensor por amostra */

/* Calibração de dois pontos (preencher na Fase 0, passo 0.6):
 *   T = CAL_GAIN * (V_mV / 10) + CAL_OFFSET   [°C]
 * Deixe 1.0 / 0.0 até calibrar. */
#define CAL_TP_GAIN        1.0f
#define CAL_TP_OFFSET      0.0f
#define CAL_TA_GAIN        1.0f
#define CAL_TA_OFFSET      0.0f

/* ---------------- Atuadores ---------------- */
#define HEATER_BITS        16     /* aquecedor: 16 bits exige f <= 80 MHz / 65536 = 1,22 kHz */
#define HEATER_FULL        65535
#define HEATER_FREQ_HZ     1000
#define PWM_BITS           10     /* cooler */
#define PWM_FULL           1023
#define COOLER_FREQ_HZ     25000

/* Limite do aquecedor. 3 × 150 Ω em paralelo a 12 V: 0,96 W em cada resistor a 100 %.
 * Ensaios de 21/10: U_MAX dentro da "janela" do anti-windup:
 *   - acima do u de regime no pior caso (60 °C com o cooler em 680: ~18000–19000; com 17000
 *     o ensaio parou em 55,9 °C), para a referência ser alcançável;
 *   - abaixo do u de pico que o PI pede no degrau de 10 °C (~30000–32000), para saturar nos
 *     transitórios e o efeito do anti-windup aparecer (com 34000 nunca saturava).
 * 21000 = 32 % de duty = 0,31 W por resistor (1,23 × a potência nominal de 1/4 W).
 * Este é o "Sat" REAL da bancada: use o MESMO valor em Sat no MATLAB/Simulink. */
#define U_MAX              21000  /* 32,0 % de 65535 */
#define U_MIN              0

/* Segurança: desliga o aquecedor se Tp passar disto. */
#define T_TRIP_C           95.0f

/* ---------------- Grupo (tabela do roteiro, §1) ---------------- */
/* Grupo 2: P.O. 50 °C, Mp 10 %, Tp 25 s, w0 0,065 rad/s (período 96,7 s). */
#define GROUP_PO_C         50.0f
#define GROUP_W0_RAD_S     0.065f

/* ---------------- Perturbação (cooler) ---------------- */
/* Escala 0..1023 (10 bits). Zona morta medida (Etapa E): o motor parte em ~530 (52 %).
 * A senoide/triangular é COOLER_OFFSET ± COOLER_AMP, nunca cruzando a zona morta. */
#define COOLER_OFFSET      620    /* medido em 20/10: w = 680 derruba Tp ~5 °C a 50 °C em malha aberta */
#define COOLER_AMP         60     /* triangular 560..680; perturbação constante do ensaio = 680 */

/* ---------------- Identificação (PRBS) ---------------- */
#define PRBS_DEFAULT_U0    10200  /* u0 medido na Etapa F (20/10): Tp ≈ 49,8 °C (15,6 % de duty) */
#define PRBS_DEFAULT_DU    2000   /* ±5 °C em regime (ganho local ~0,0025); coleta de 20–21/10: 45,9–56,1 °C */
#define PRBS_DEFAULT_BIT_S 60     /* duração de cada nível; LFSR de 9 bits → 2^9-1 = 511 bits = 8,5 h */
#define PRBS_DEFAULT_T_AQUEC_S 2000 /* tempo em u0 antes do PRBS, para sair do ambiente e
                                       estabilizar no P.O. (= transit = 2000 do script do professor) */

/* ---------------- Controlador PI discreto ---------------- */
/* C(z) = PI_K * (z - PI_Z0) / (z - 1), período CTRL_TS_MS. Projeto do proj1cdig_m.m (21/10/2026)
 * sobre o modelo identificado (Tp − Ta): Kp = 0,00320 °C/cont, Tz = 821,9 s,
 * Tp1 = 1553,9 s, Tp2 = 34,46 s, Td = 5,257 s (fit 86 %). LGR discreto (ZOH, atraso de 2 amostras)
 * + refino: Mp = 10,1 %, tp = 23,7 s em simulação. */
#define CTRL_TS_MS         2629   /* Ts = Td/2 */
#define PI_K               2039.9595f /* contagens/°C */
#define PI_Z0              0.94216f

/* ---------------- Ensaio do roteiro (passo 8, Figura 5) — comando "e" ----------------
 * Referência em onda quadrada entre P.O. e P.O. + ENSAIO_AMPL_C, meio período ENSAIO_MEIO_PERIODO_S.
 * Cooler: desligado até T_CONST1, constante (máximo da senoide) até T_TRI, triangular até
 * T_CONST2, constante até T_FIM. ISE = soma de e²·Ts entre ISE_T0 e ISE_T1 (roteiro: 2100–3000 s). */
#define ENSAIO_AMPL_C          10.0f
#define ENSAIO_MEIO_PERIODO_S  400
#define ENSAIO_T_CONST1_S      1000
#define ENSAIO_T_TRI_S         1800
#define ENSAIO_T_CONST2_S      2600
#define ENSAIO_T_FIM_S         3600
#define ISE_T0_S               2100
#define ISE_T1_S               3000
