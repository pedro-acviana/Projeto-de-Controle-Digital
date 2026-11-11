# Instrução — sequência de testes do Projeto 1 (Prj1CDig, grupo 2)

Roteiro passo a passo para refazer, do zero, todos os ensaios da bancada: sensores, cooler, ponto de operação,
PRBS de 9 h, projeto no MATLAB, limite do aquecedor e ensaios com e sem anti-windup.
Os comandos e tempos abaixo são os que geraram os dados finais em `proj1cdig/dados/` (20 e 21/10/2026).

---

## Resumo do que foi feito

| Etapa | Resultado final |
|---|---|
| Bancada | ESP32-WROOM; LM35 do processo no GPIO35 e do ambiente no GPIO34 (ADC interno, média de 400 ms); aquecedor 3 × 150 Ω 1/4 W em paralelo, 12 V, PWM 16 bits a 1 kHz no GPIO25; cooler 12 V, PWM 10 bits a 25 kHz no GPIO26; damper 75 Ω + 1 µF em cada LM35; sem isopor/caixa |
| Sensores | σ(Tp) = 0,047 °C parado e 0,036 °C com o cooler ligado (requisito: < 0,1 °C). Calibração mantida em ganho 1, offset 0 |
| Cooler | Zona morta: parte em w ≈ 530. Perturbações usam 560–680 (constante 680; triangular 620 ± 60) |
| Ponto de operação | u0 = 10200 contagens → Tp ≈ 50 °C. Ganho local ≈ 0,0025 °C/contagem → Δu = 2000 para ±5 °C |
| PRBS | `p 10200 2000 60 2000`: 2000 s de aquecimento + 511 bits de 60 s = 9,07 h |
| Modelo (MATLAB) | P2DZ com y = Tp − Ta: Kp = 0,00320, Tz = 822 s, Tp1 = 1554 s, Tp2 = 34,5 s, Td = 5,26 s, fit 86,4 % |
| PI discreto | Ts = Td/2 = 2,629 s, K = 2039,96, z0 = 0,94216 (Mp 10,1 %, tp 23,7 s em simulação) |
| Limite do aquecedor | U_MAX = 21000 (32 % de duty, 0,31 W por resistor). 17000 não chega a 60 °C com cooler; 34000 nunca satura |
| Ensaio da Figura 5 | ISE 2998 com anti-windup e 2938 sem; sobrepasso ≤ 0,9 % com AW e 13,5–21,6 % sem |

O que **não** fazer de novo: identificar com o isopor (dados antigos em `dados/com_isopor/`, fora do relatório),
`procest` sem chute inicial (cai num mínimo local com Tz = 0) e U_MAX = 17000 (`dados/umax17000/`).

---

## Antes de começar

**Onde rodar cada coisa**

| O quê | Onde | Terminal |
|---|---|---|
| Gravar firmware | `C:\cdig\fw` (caminho curto, sem espaços: o ESP-IDF falha com o caminho longo) | ESP-IDF (PowerShell do ESP-IDF v5.3 ou v6.1) |
| Logger | `C:\Users\Pedro\Controle Digital 26_2\proj1cdig` | PowerShell/cmd comum, com `pip install pyserial matplotlib` |
| Identificação e projeto | `proj1cdig_m.m`, Current Folder = `proj1cdig` | MATLAB (System Identification e Control System Toolboxes) |

**Formato do logger**

```
python proj1cdig_logger.py COMx --tag NOME --cmd "COMANDO" [--cmd "OUTRO"]
```

- `COMx` = porta da ESP no Gerenciador de Dispositivos.
- Cada execução gera `dados/proj1cdig_NOME_data-hora.csv`, `.log` e `.png` (salvo ao fechar).
- Com o logger aberto, **digite os comandos no terminal** + Enter (não na janela do gráfico).
- **Ctrl+C** fecha o arquivo. Reabrir o logger **não reseta** a placa: ela continua no modo em que estava.
- Feche o `idf.py monitor` antes de abrir o logger (os dois não usam a porta ao mesmo tempo).

**Gatilho "ESTÁVEL"**: o título do gráfico mostra o selo ESTÁVEL quando a média dos últimos 30 s difere menos de
0,1 °C da média de 5 min atrás. Em malha aberta isso leva **15–30 min** depois de mudar u; em malha fechada, 5–15 min.

**PC**: na tomada, suspensão e "suspensão seletiva USB" desligadas, longe de sol e ar-condicionado.

---

## Etapa 0 — Gravar o firmware (uma vez, antes dos ensaios)

1. Confira em `C:\cdig\fw\main\proj1cdig_config.h`: `U_MAX 21000`, pinos 35/34/25/26, `PRBS_DEFAULT_U0 10200`,
   `PRBS_DEFAULT_DU 2000`. Para uma bancada nova, o PI (`CTRL_TS_MS`, `PI_K`, `PI_Z0`) pode ficar com o valor
   antigo: ele só é usado a partir da Etapa 6.
2. No terminal do ESP-IDF:
   ```
   cd C:\cdig\fw
   idf.py set-target esp32        ← só na primeira vez
   idf.py -p COMx flash
   ```
   "Failed to connect" → segure **BOOT** enquanto a gravação começa.
3. **Gatilho para seguir**: `Hard resetting via RTS pin...` no fim. Feche o terminal (ou Ctrl+] se abriu o monitor).

`C:\cdig\fw` e `proj1cdig\firmware` têm de ser cópias idênticas: toda mudança no `config.h` vale para as duas.

---

## Etapa 1 — Sensores e ruído (~25 min)

```
python proj1cdig_logger.py COMx --tag sensores --cmd s
```

| Passo | Comando | Gatilho para o próximo |
|---|---|---|
| 1.1 | (já enviado) `s` | Aparece `#ADC ok: Tp no GPIO35, Ta no GPIO34`. Espere **10 min**: Tp ≈ Ta ≈ ambiente e **σ(Tp) < 0,1 °C** no título |
| 1.2 | `wc 700` | Cooler girando forte. Espere **5 min**: σ(Tp) continua < 0,1 °C (captação do motor) |
| 1.3 | `w0` | Cooler parado → siga para a Etapa 2 **no mesmo logger** |

Se σ ≥ 0,1 °C: trançar os fios dos sensores, terra comum curto, conferir o damper. Não avance sem passar.
Leitura perto de 0 °C ou fixa = fio solto ou pino errado.

---

## Etapa 2 — Zona morta do cooler (~10 min, mesmo logger)

| Passo | Comando | Gatilho |
|---|---|---|
| 2.1 | `wc 500` | Espere **10 s** olhando/ouvindo o cooler |
| 2.2 | `wc 510`, `wc 520`, `wc 530`… (passos de **10**) | A cada **~10 s**, até o motor **partir sozinho** do repouso. Na bancada final: ≈ 530 |
| 2.3 | `w0`, depois `wc <valor achado + 30>` | Confirme que ele parte de novo do repouso (560 parte, fraco) |
| 2.4 | `x` → Ctrl+C | — |

Com o valor achado, ajuste no `config.h`: a perturbação tem de ficar toda **acima** da zona morta.
Valores finais: `COOLER_OFFSET 620`, `COOLER_AMP 60` (faixa 560–680), constante = 680 (máximo da faixa).
Mudou? Regrave (Etapa 0).

---

## Etapa 3 — Ponto de operação u0 e excursão Δu (~1 h)

```
python proj1cdig_logger.py COMx --tag u0 --cmd "o 11000"
```

| Passo | Comando | Gatilho |
|---|---|---|
| 3.1 | (já enviado) `o 11000` | **ESTÁVEL** (≈ 15–20 min). Anote Tp (na bancada: 52,1 °C) |
| 3.2 | `o 10000` | ESTÁVEL. Anote Tp (49,6 °C) |
| 3.3 | `o <interpolado>` | Interpole para 50 °C: cada **~400 contagens ≈ 1 °C**. Na bancada: `o 10200` → 49,8 °C. Se ESTÁVEL a menos de ±0,3 °C de 50, este é o **u0** |
| 3.4 | `wc 680` | Teste do efeito da perturbação: espere **~6 min** (Tp caiu de 49,7 para 45,3 °C, tendendo a −5 °C) |
| 3.5 | `w0` | Tp volta a subir. Pode ir direto para a Etapa 4 (o PRBS tem aquecimento próprio) |

**Contas**: ganho local = ΔTp/Δu entre 3.1 e 3.2 (≈ 2,5 °C / 1000 = 0,0025 °C/contagem).
Δu = 5 °C / ganho ≈ **2000**. Confira que u0 + Δu ≤ U_MAX (12200 ≤ 21000), senão o firmware recusa o PRBS.

---

## Etapa 4 — Coleta PRBS de 9 h (à noite)

```
python proj1cdig_logger.py COMx --tag prbs --cmd "p 10200 2000 60 2000"
```

| Passo | O que aparece / fazer | Gatilho |
|---|---|---|
| 4.1 | `#PRBS u0=10200 du=2000 duracao_bit=60 s tempo_aquecimento=2000 s, duracao total 9.07 h` | Confirme os números. Não mexa na bancada nem no PC |
| 4.2 | 2000 s em u0, depois u alterna entre 8200 e 12200 (patamares de 60 a 540 s) | Tp oscila ±3–5 °C em torno de 50 |
| 4.3 | `#PRBS concluido em k=32660` | u volta a u0. Envie `x` |
| 4.4 | Ctrl+C | Confira no `.log` que não houve `#TRIP` e no gráfico que não há buracos |

**Guarde o CSV**: é a base do Projeto 1 e do Projeto 2.

---

## Etapa 5 — MATLAB: identificação e projeto (seções 0 a 8)

`proj1cdig_m.m`, uma seção por vez (Ctrl+Enter).

| Seção | O que editar / conferir |
|---|---|
| 0 | `ARQ_PRBS = 'dados/proj1cdig_prbs_<data-hora>.csv';` · `Sat = 21000;` · `U0_MEDIDO = 10200;` |
| 1 | Coleta plotada sem buracos |
| 2 | `USAR_TP_MENOS_TA = true` e o chute inicial já no código. Critérios: **Td 3–8 s, Tz > 0, fit > 85 %** |
| 3 | Variáveis dos `.slx` (opcional: `sim Termico_FP` para a figura do vídeo) |
| 4 | ζ = 0,591, ωn = 0,156 rad/s, s_d = −0,092 ± j0,126 |
| 5 | PI contínuo (ponto de partida para o sisotool e o Simulink, itens 4–5 do roteiro) |
| 6 | PI discreto: Ts = Td/2, LGR em z, refino → **Ts, K, z0** |
| 7 | Simula degraus de 5 e 10 °C com saturação em `Sat`, com e sem anti-windup |
| 8 | Imprime as linhas para o firmware |

Copie a saída da seção 8 para **os dois** `proj1cdig_config.h` (`C:\cdig\fw\main` e `proj1cdig\firmware\main`):

```
#define CTRL_TS_MS 2629
#define PI_K       2039.9595f
#define PI_Z0      0.94216f
```

Regrave (Etapa 0). **Ts só muda regravando**. K e z0 podem ser trocados sem regravar com `k <K> <z0>`, mas só valem
até a placa reiniciar.

---

## Etapa 6 — Limite do aquecedor U_MAX (pior caso, ~15 min)

U_MAX tem de ficar numa janela: **acima** do u que mantém 60 °C com o cooler em 680 e **abaixo** do pico que o PI pede
num degrau de 10 °C (≈ u0 + K·10 ≈ 30000). Abaixo da janela, 60 °C não é alcançado; acima, o atuador nunca satura
e o anti-windup não aparece. Antes do ensaio, a seção 7 do MATLAB já mostra se há saturação nos degraus.

```
python proj1cdig_logger.py COMx --tag teste_saturacao --cmd "a 1" --cmd "c 50"
```

| Passo | Comando | Gatilho |
|---|---|---|
| 6.1 | (já enviado) `a 1`, `c 50` | ESTÁVEL em 50 °C (5–15 min; partindo de frio, use antes `o 10200` por ~5 min) |
| 6.2 | `wc 680` | Espere **~3 min**: Tp volta a 50 com u maior |
| 6.3 | `r 60` | Espere **~5 min**. **Critério**: u encosta em U_MAX só no transitório (≈ 26 s) e depois fica abaixo dele (na bancada: u ≈ 19560, 7 % de folga) com Tp = 60 ± 0,1 °C |
| 6.4 | `r 50`, depois `w0` | Ctrl+C |

- u **preso** em U_MAX e Tp parando abaixo de 60 → U_MAX baixo demais: aumente, regrave, repita.
- u **nunca** encosta em U_MAX no `r 60` → U_MAX alto demais: diminua, regrave, repita.
- Limite de potência: P por resistor = duty × 0,96 W. 21000 = 32 % = 0,31 W (acima do nominal de 1/4 W, aceito
  porque só ocorre em transitórios; numa montagem definitiva, usar resistores de 1/2 W).

---

## Etapa 7 — Ensaio da Figura 5, com e sem anti-windup (~2 h 15 min)

O firmware executa o perfil sozinho (comando `e`): referência alternando 50 ↔ 60 °C a cada 400 s; cooler desligado
até 1000 s, constante em 680 até 1800 s, triangular até 2600 s, constante até 3600 s; ISE calculado entre 2100 e
3000 s. **Durante o ensaio, não digite `x`, `s`, `o`, `p`, `c`, `r` nem `w…`**: qualquer um desses encerra o perfil.

**7.1 Preparo (~10–15 min)**

```
python proj1cdig_logger.py COMx --tag preparo --cmd "o 10200"
```
Aquecendo de frio: espere ~4 min, envie `a 1` e `c 50` → **ESTÁVEL** em 50 °C → Ctrl+C. A placa continua em
malha fechada.

**7.2 Com anti-windup (60 min + 10 min)**

```
python proj1cdig_logger.py COMx --tag ensaio_aw1 --cmd "a 1" --cmd "e"
```

| Tempo | Mensagem esperada |
|---|---|
| 0 s | `#ENSAIO iniciado: ref 50.0/60.0 C, meio periodo 400 s, 3600 s no total, ISE entre 2100 e 3000 s` |
| ~1000 s | `#ENSAIO t=1002 s: perturbacao constante` |
| ~1800 s | `#ENSAIO t=1801 s: perturbacao triangular` |
| ~2600 s | `#ENSAIO t=2600 s: perturbacao constante` |
| ~3000 s | `#ISE(2100-3000 s) = …` (faixa típica 2000–4000; na bancada: 2997,7) |
| 3600 s | `#ENSAIO concluido: ISE(2100-3000 s) = …` → a placa volta a 50 °C em malha fechada |

**Gatilho**: depois de `#ENSAIO concluido`, espere **ESTÁVEL** em 50 °C (~3–10 min) → Ctrl+C.

**7.3 Sem anti-windup (60 min)**

```
python proj1cdig_logger.py COMx --tag ensaio_aw0 --cmd "a 0" --cmd "e"
```
Mesmas mensagens (na bancada: ISE 2937,6). Depois de `#ENSAIO concluido`: `a 1` → `x` → Ctrl+C.

---

## Etapa 8 — MATLAB: análise dos ensaios (seção 9)

Para cada ensaio: seção 0 com `ARQ_MF = 'dados/proj1cdig_ensaio_aw1_<data-hora>.csv';` → seção 9.
Repita com o `ensaio_aw0`. A seção 9 imprime o ISE (tem de bater com o do firmware), o tempo saturado e Mp/tp do
1º degrau, e gera a figura "Malha fechada". Valores da bancada:

| | Com AW | Sem AW |
|---|---|---|
| ISE (2100–3000 s) | 2997,8 | 2937,6 |
| Tempo saturado em U_MAX | 92 s | 216 s |
| Mp / tp (1º degrau) | 2,9 %* / 312,9 s | 14,3 % / 39,4 s |

\* pico de convecção a +313 s; nos primeiros 150 s o sobrepasso fica abaixo de 1 %.

---

## Etapa 9 (pendente) — Degrau sem saturação (~25 min)

Para comparar Mp e tp experimentais com o projeto (10 %, 25 s), sem a saturação mascarar a resposta
(pico de u ≈ 14300 < 21000):

```
python proj1cdig_logger.py COMx --tag degrau_pequeno --cmd "a 1" --cmd "c 50"
```
ESTÁVEL em 50 °C → `r 52` → espere **400 s** → `r 50` → ESTÁVEL → `x` → Ctrl+C.
Mp = (pico − 52)/2 e tp = tempo do degrau até o pico.

---

## Mensagens de alerta

| Mensagem | Significa | O que fazer |
|---|---|---|
| `#TRIP Tp=… > 95.0` | Tp passou de 95 °C; aquecedor desligado | Conferir sensor e fixação antes de religar |
| `#ERRO PRBS fora da faixa 0..21000` | u0 ± Δu fora de 0..U_MAX | Diminuir Δu |
| `#? comando desconhecido: …` | Digitação errada | `h` lista os comandos |
| `[logger] conexão caiu; reconectando…` | USB caiu | O logger reabre sozinho, sem resetar a placa |
| `#proj1cdig iniciado` no meio do ensaio | A placa reiniciou | Ensaio perdido: refaça a partir do preparo |
| Gráfico parado após `x` | Em IDLE a placa não manda dados | Normal |

## Comandos do firmware

| Comando | Faz |
|---|---|
| `h` | Lista os comandos |
| `x` | Para tudo (aquecedor e cooler em 0), modo IDLE |
| `s` | Só lê os sensores, aquecedor desligado |
| `o <u>` | Malha aberta, aquecedor fixo em u (0–21000) |
| `p <u0> <du> <duracao_bit_s> <tempo_aquecimento_s>` | PRBS |
| `c [ref]` | Malha fechada PI (padrão 50 °C), partida sem solavanco |
| `r <ref>` | Degrau de referência |
| `a 0` / `a 1` | Anti-windup desligado / ligado |
| `k <K> <z0>` | Troca o PI sem regravar (até reiniciar) |
| `e [baixa alta meio_periodo_s]` | Ensaio da Figura 5 (padrão 50/60 °C, 400 s), calcula o ISE |
| `w0` / `wc <w>` / `ws` / `wt` | Cooler: desligado / constante (0–1023) / senoide / triangular |
