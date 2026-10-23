/*
 * proj1cdig_config.h (versão AUTO) — valores INICIAIS. O proj1cdig_auto.py ajusta
 * U_MAX, calibração, cooler, Ts e PI em tempo real e reescreve este arquivo no fim.
 * Prj1CDig · ENE0167 Controle Digital · UnB
 *
 * Unidade do atuador: CONTAGENS do PWM de 16 bits do aquecedor (0..65535 = 0..12 V médio),
 * como pede o roteiro atualizado (passo 2: "PRBS implementado via PWM 16 bits").
 * O Kp identificado sai em °C/contagem (~0,003) e o ganho do PI em contagens/°C (~2000).
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

/* Limite físico do aquecedor. 3 × 150 Ω em paralelo a 12 V: 0,96 W em cada resistor a 100 %.
 * Com 1/4 W, o máximo é 26 % de duty; 16000 contagens = 24,4 % (0,23 W por resistor).
 * Este é o "Sat" REAL da bancada: use o MESMO valor em Sat no MATLAB/Simulink.
 * Se a fonte do aquecedor for 5 V (e não 12 V), cada resistor dissipa só 0,17 W a 100 %: pode usar 65535. */
#define U_MAX              16000  /* 24,4 % de 65535 */
/* Teto absoluto para o comando U em tempo real. Só aumente se os resistores aguentarem
 * (1 W → pode ir a 65535). O script nunca consegue passar deste valor. */
#define U_MAX_HARD         U_MAX
#define U_MIN              0

/* Segurança: desliga o aquecedor se Tp passar disto. */
#define T_TRIP_C           95.0f

/* ---------------- Grupo (tabela do roteiro, §1) ---------------- */
/* Grupo 2: P.O. 50 °C, Mp 10 %, Tp 25 s, w0 0,065 rad/s (período 96,7 s). */
#define GROUP_PO_C         50.0f
#define GROUP_W0_RAD_S     0.065f

/* ---------------- Perturbação (cooler) ---------------- */
/* Escala 0..1023 (10 bits). O motor só parte acima de ~30 %: meça na Etapa E e ajuste.
 * A senoide/triangular é COOLER_OFFSET ± COOLER_AMP, nunca cruzando a zona morta. */
#define COOLER_OFFSET      460    /* ~45 % */
#define COOLER_AMP         256    /* ±25 % */

/* ---------------- Identificação (PRBS) ---------------- */
#define PRBS_DEFAULT_U0    8320   /* ponto médio que leva ao P.O. — achar na Etapa F (chute: ~13 % de duty) */
#define PRBS_DEFAULT_DU    1680   /* excursão que dá ~±5 °C — achar na Etapa G */
#define PRBS_DEFAULT_BIT_S 60     /* duração de cada nível; LFSR de 9 bits → 2^9-1 = 511 bits = 8,5 h */
#define PRBS_DEFAULT_T_AQUEC_S 2000 /* tempo em u0 antes do PRBS, para sair do ambiente e
                                       estabilizar no P.O. (= transit = 2000 do script do professor) */

/* ---------------- Controlador PI discreto ---------------- */
/* C(z) = PI_K * (z - PI_Z0) / (z - 1), período CTRL_TS_MS.
 * Valores que saem do proj1cdig_m.m (Fase 3). Os abaixo são só placeholders
 * calculados com o modelo de referência do roteiro, reescalado para o ganho esperado desta
 * bancada (Kp ≈ 0,003 °C por contagem de 16 bits, 3 × 150 Ω em 12 V) — NÃO use no experimento. */
#define CTRL_TS_MS         2330   /* Ts = Td/2 (modelo de referência: Td = 4,66 s) */
#define PI_K               2140.0f /* contagens/°C  (ref.: Mp 10 %, tp 25,6 s em simulação) */
#define PI_Z0              0.9447f

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
