# Controle Digital 2026.2 — Projeto 1 (Prj1CDig)

Controle PI digital com anti-windup de um processo térmico, implementado em uma ESP32.
Disciplina ENE0167 Controle Digital (UnB), Prof. Adolfo Bauchspiess. Grupo 2.

O processo é um sensor LM35 aquecido por três resistores acionados por PWM, com um cooler como
perturbação. O projeto cobre a identificação do processo por PRBS, o projeto do PI pelo lugar das
raízes discreto, a simulação com saturação e os ensaios experimentais com e sem anti-windup.

| Especificação (grupo 2) | Valor |
|---|---|
| Ponto de operação | 50 °C |
| Sobrepasso Mp | 10 % |
| Tempo de pico tp | 25 s |
| Período de amostragem Ts | Td/2 |
| Frequência da perturbação ω0 | 0,065 rad/s |

## Resultados principais

| Item | Resultado |
|---|---|
| Modelo P2DZ (`procest`, saída Tp − Ta) | Kp = 0,00320 °C/contagem, Tz = 821,9 s, Tp1 = 1553,9 s, Tp2 = 34,46 s, Td = 5,257 s; fit de 86,4 % na validação |
| PI discreto | C(z) = K (z − z0)/(z − 1), K = 2039,96 contagens/°C, z0 = 0,94216, Ts = 2,629 s |
| Saturação do aquecedor | U_MAX = 21000 contagens (32 % de duty) |
| Ensaio da Fig. 5 do roteiro, com anti-windup | ISE (2100–3000 s) = 2998 °C²·s; sobrepasso < 1 % nos degraus de 10 °C |
| Mesmo ensaio, sem anti-windup | ISE = 2938 °C²·s; sobrepasso de 13,5 a 21,6 % |

## Estrutura

```
.
├── proj1cdig/                  versão manual (a usada nos ensaios)
│   ├── firmware/               firmware ESP-IDF da ESP32
│   │   └── main/
│   │       ├── proj1cdig.c         modos, PRBS, PI anti-windup, ensaio da Fig. 5
│   │       └── proj1cdig_config.h  pinos, PWM, U_MAX, PI, perturbação
│   ├── proj1cdig_logger.py     serial → CSV, com gráfico em tempo real
│   ├── proj1cdig_plot.py       figuras a partir dos CSV (tela e relatório)
│   ├── proj1cdig_m.m           MATLAB: identificação, projeto do PI e análise dos ensaios
│   ├── dados/                  CSV, logs e PNG de cada etapa
│   └── outputs/                figuras do MATLAB (D = versão final)
├── proj1cdig_auto/             versão automatizada do mesmo fluxo (com simulador)
├── proj1cdig_relatorio/        relatório em LaTeX (modelo IFAC da disciplina)
├── proj1cdig_relatorio_v1.pdf  relatório compilado
├── Lista-de-Compras-e-Dimensionamento.md
├── Lista-de-Materiais-CDig.xlsx
└── Projeto-de-Controle-Digital/  submódulo: repositório de referência de Cristiano (CristianoCTS)
```

## Bancada

| Item | Ligação |
|---|---|
| ESP32-WROOM-32 DevKit | ESP-IDF v6.1 |
| LM35 do processo (Tp) | GPIO35 (ADC1_CH7) |
| LM35 do ambiente (Ta) | GPIO34 (ADC1_CH6), ~15 cm do aquecedor, fora do fluxo do cooler |
| Aquecedor: 3 × 150 Ω 1/4 W em paralelo, encostados no LM35 | PWM no GPIO25, 16 bits, 1 kHz, 12 V |
| Cooler 12 V | PWM no GPIO26, 10 bits, 25 kHz |
| Damper RC na saída de cada LM35 | 75 Ω (2 × 150 Ω em paralelo) + 1 µF |

A filtragem do sinal é digital: cada amostra é a média de 400 ms de conversões do ADC1
(atenuação de 2,5 dB e calibração de fábrica da placa).

## Como usar (versão manual)

**1. Gravar o firmware.** No terminal do ESP-IDF, de preferência a partir de um caminho curto e
sem espaços (por exemplo, uma cópia em `C:\cdig\fw`):

```
idf.py set-target esp32
idf.py -p COM7 flash monitor
```

**2. Coletar dados.** Na pasta `proj1cdig`, com `pip install pyserial matplotlib`:

```
python proj1cdig_logger.py COM7 --tag <nome> --cmd "<comando>"
```

| Comando | Função |
|---|---|
| `s` | só lê os sensores |
| `o <u>` | malha aberta com o aquecedor fixo em u |
| `p <u0> <du> <bit_s> <aquec_s>` | PRBS (ex.: `p 10200 2000 60 2000`, 9,07 h) |
| `c <ref>` / `r <ref>` | malha fechada / nova referência |
| `a 0` / `a 1` | anti-windup desligado / ligado |
| `e` | ensaio da Figura 5 do roteiro (3600 s), com cálculo do ISE |
| `wc <w>`, `ws`, `wt`, `w0` | cooler constante, senoidal, triangular, desligado |
| `x` | desliga tudo |

Cada execução grava `dados/proj1cdig_<tag>_<data>.csv`, um `.log` com as mensagens da placa
e, ao fechar, um `.png`.

**3. MATLAB.** No `proj1cdig_m.m`, ajuste `ARQ_PRBS` e `U0_MEDIDO` na seção 0 e rode o script.
Ele identifica o modelo, projeta o PI, simula com saturação e imprime as linhas para o
`proj1cdig_config.h`. A seção 9 analisa um ensaio em malha fechada (`ARQ_MF`).

## O que não está no repositório

- **`Referencias/`**: material da disciplina (roteiros, modelos Simulink, template do Overleaf)
  e livros com direitos autorais. Os roteiros e o template estão na página da disciplina:
  https://www.ene.unb.br/adolfo/Lectures/CDig.html
- **Vídeos dos ensaios** (`*.mp4`, mais de 1 GB cada) e arquivos `.zip` de backup.
