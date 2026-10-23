# proj1cdig_auto — Projeto 1 automatizado

Versão automática do passo a passo de `../proj1cdig/proj1cdig_plano.md`. A pasta `proj1cdig`
continua valendo para fazer tudo à mão. As duas não se misturam: **cada uma tem o seu firmware**.

| Arquivo | Função |
|---|---|
| `proj1cdig_auto.py` | O orquestrador. Faz as perguntas iniciais e executa as etapas em sequência, parando só quando algo sai do critério. |
| `firmware/` | Cópia do firmware com comandos extras (`U`, `T`, `C`, `W`, `F`) que mudam saturação, Ts, calibração e cooler **em tempo real**. Grava-se uma vez só. |
| `proj1cdig_auto_projeto.m` | Função MATLAB sem interface: identificação + PI discreto + simulação, salva `dados/projeto.json` e figuras. |
| `simulador.py` | Bancada virtual para ensaiar o fluxo inteiro sem hardware (`--simular`). |
| `estado.json` | Criado na primeira execução: respostas e progresso. Apagar = começar do zero. |
| `dados/` | CSVs de cada etapa, log, `projeto.json`, `relatorio_auto.md` e gráficos. |

Os CSVs têm o mesmo formato do logger manual: dá para abrir qualquer um no `proj1cdig_m.m`.

## Montagem considerada (19/10/2026)

| Item | Valor no firmware |
|---|---|
| Placa | ESP32-WROOM-32 DevKit (alvo `esp32`) |
| LM35 do processo | **D35** (ADC1_CH7), ADC interno |
| LM35 do ambiente | **D34** (ADC1_CH6), ADC interno |
| Aquecedor | 3 × 150 Ω **1/4 W**, PWM de **16 bits** (0–65535) a 1 kHz no **D25** — confira; **U_MAX = 16000** (24 %) |
| Cooler | PWM no **D26** |
| "Filtro" | 75 Ω (2 × 150 Ω em paralelo) + 1 µF: é o damper do datasheet, quase não filtra; a média é feita no firmware (400 ms por amostra) |

Pinos e limites ficam em `firmware/main/proj1cdig_config.h`.

## Antes de rodar

1. `pip install pyserial matplotlib`
2. Bancada montada e fixada (Etapas A e E do plano manual — isso o script não faz).
3. **Feche o logger manual e qualquer monitor serial.**
4. Recomendado: abrir o terminal do **ESP-IDF** nesta pasta. Assim o script grava o firmware sozinho.
   Se o MATLAB estiver no PATH (`matlab` funciona no terminal), a identificação também roda sozinha.

## Ensaio geral sem hardware (recomendado antes da primeira vez)

```
python proj1cdig_auto.py --simular
```

A bancada virtual roda ~600× mais rápido: as 9 h de PRBS viram cerca de 1 min e o fluxo inteiro, uns 3 min. Na etapa `projeto`, se o
MATLAB não estiver no PATH, o script mostra o comando para rodar no MATLAB sobre o CSV simulado —
bom treino da identificação. O progresso simulado fica em `estado_simulado.json`, separado do real.

## Rodar de verdade

```
python proj1cdig_auto.py
```

### Perguntas iniciais

| Pergunta | O que responder |
|---|---|
| Porta serial | A COM da ESP32 (Gerenciador de Dispositivos → Portas COM) |
| Grupo | 2 |
| U_MAX | **16000** (de 65535) com resistores de 1/4 W em 12 V. Acima disso o firmware recusa, a menos que `U_MAX_HARD` seja aumentado no `config.h` e o firmware regravado |
| Calibração | **1** digitar ganho/offset já conhecidos · **2** assistida · **3** pular |
| Zona morta do cooler | **1** digitar · **2** assistida · **3** não sei (usa 300) |
| Chute de PWM para o P.O. | 130 se não tiver ideia |
| Rodar o MATLAB sozinho? | Só aparece se o `matlab` estiver no PATH |
| Parar antes da coleta de 9 h? | **s** se quiser escolher a hora de começar a coleta noturna |

Na **calibração assistida**, prenda o termômetro de referência junto ao LM35 do processo. O script
estabiliza em ambiente, pede as leituras do termômetro (processo e ambiente), aquece com `o 150`,
estabiliza e pede a leitura do processo de novo. Ta recebe só offset.

Na **zona morta assistida**, o script sobe o cooler de 50 em 50 e pergunta se ele está girando.

### Etapas

| Etapa | O que o script faz | Para e pergunta se… | Duração típica |
|---|---|---|---|
| `calibracao` | (assistida) dois pontos contra o termômetro | ganho fora de 0,9–1,1 | 30–45 min |
| `zona_morta` | (assistida) sobe o cooler até girar | não girar nem a 100 % | 1 min |
| `sensores` | espera o repouso, mede σ(Tp) sem e com cooler | σ ≥ 0,1 °C (resolução do roteiro) ou leituras `nan` | 5–30 min |
| `u0` | `o u`, espera ESTÁVEL, corrige por secante até Tp = P.O. ± 0,5 °C | não convergir em 7 tentativas; faltar potência | 1–2 h |
| `du` | degrau de +30, mede o ganho estático, du = 5 °C ÷ ganho | du passar dos limites | 20–40 min |
| `prbs_teste` | PRBS de 35 min; ajusta u0/du se a excursão não ficar em ±2–6,5 °C em torno do P.O. | não acertar em 3 tentativas | 35–105 min |
| `prbs` | coleta de 2000 s + 511 × 60 s | >1 % de leituras inválidas ou buracos | 9,1 h |
| `projeto` | MATLAB (sozinho ou pedindo para você rodar), aplica Ts, K, z0 na placa | Td fora de 3–9 s, Tz ≤ 0, fit < 85 % | 2–10 min |
| `ensaios` | passo 8 do roteiro (Figura 5): malha fechada em 50 °C e depois o comando `e` duas vezes — `ensaio_aw1` (com anti-windup) e `ensaio_aw0` (sem), **um CSV cada**. Cada um: referência em onda quadrada 50/60 °C (400 s), cooler constante a partir de 1000 s, triangular de 1800 a 2600 s, constante até 3600 s; o firmware calcula o ISE entre 2100 e 3000 s | o firmware não avisar o fim do ensaio | ~2,5 h |
| `relatorio` | `dados/relatorio_auto.md`: Mp e tp no 1º degrau, ISE 2100–3000 s, efeito das perturbações e tempo saturado, com e sem anti-windup lado a lado; gráficos; grava os valores finais no `firmware/main/proj1cdig_config.h` desta pasta | — | segundos |

"ESTÁVEL" = média dos últimos 30 s difere menos de 0,1 °C da média de 5 min atrás. Todas as esperas usam o
relógio da placa, não o do PC.

### Pontos de decisão

Quando um critério falha, o script mantém a bancada como está, mostra o problema e pergunta:
`[c] continuar mesmo assim · [r] repetir a etapa · [a] abortar`.

### Interromper e continuar

- **Ctrl+C** desliga aquecedor e cooler e salva o progresso. `python proj1cdig_auto.py` continua da etapa
  interrompida (a etapa em andamento recomeça do início dela).
- Se a ESP32 reiniciar (queda de energia, USB), o script para com aviso: rode de novo.
- `--de u0` refaz a partir de uma etapa; `--reconfigurar` refaz as perguntas; `--novo` começa do zero
  (o `estado.json` antigo é guardado com data).

## O que continua sendo seu

- Montagem, fixação e o termômetro na calibração.
- **sisotool e Simulink** (passos 4–7 do roteiro): o professor espera que você justifique a posição do zero e
  do ganho. Use o `proj1cdig_m.m` da pasta manual com o CSV `dados/proj1cdig_auto_prbs_….csv` — o resultado
  deve bater com o `projeto.json`. As figuras de `dados/figs_projeto/` ajudam no vídeo.
- Decidir o que fazer quando o script para num ponto de decisão.
- O vídeo.
