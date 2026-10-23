# proj1cdig — passo a passo completo do Projeto 1

Prj1CDig · Controle PI Anti-Windup de Processo Térmico · ENE0167 · 2/2026 · **Grupo 2**

| Especificação do grupo 2 | Valor |
|---|---|
| Ponto de operação (P.O.) | **50 °C** |
| Sobrepasso Mp | **10 %** |
| Tempo de pico Tp | **25 s** |
| Frequência da perturbação ω0 | **0,065 rad/s** (período de 96,7 s) |
| Faixa da coleta PRBS | **45–55 °C** (50 ± 5) |
| Degrau de referência nos ensaios | **50 → 55 °C** |

## O que mudou no roteiro (versão nova, recebida em 19/10/2026)

| Mudança | O que fizemos |
|---|---|
| Passo 2: "PRBS implementado via **PWM 16 bits**"; tabela de variáveis com U [0–32768] e Us = "excursão do número de bits do PWM" | Aquecedor agora em **16 bits** (0–65535, 1 kHz). Todos os valores de u, du, U_MAX, Sat e do ganho do PI passaram para essa escala. O cálculo do PI é em ponto flutuante, o que cobre o "#bits → evitar propagação de erro" |
| Passo 8: "mostrar em tempo real (tela PC) o seguimento de **referência onda quadrada**, com rejeição de perturbações" (Figura 5: 3600 s, ref P.O./P.O.+10, perturbação constante e triangular) | Novo comando **`e`** no firmware, que roda exatamente esse ensaio (Etapa L). O gráfico do logger é a "tela PC" |
| Tabela de variáveis: **ISE** entre t = 2100 e 3000 s, típico 2000–4000 | O firmware calcula e imprime o ISE ao fim do ensaio; a seção 9 do `.m` também calcula |
| Objetivo: rejeitar perturbações constantes e **triangulares** (antes: senoidais) | O ensaio usa constante + triangular; `ws` continua disponível para o Prj2 |
| Passos 9 e 10 (verificar e gravar vídeo) foram removidos; o passo 8 inclui "verificar as especificações" | Etapa N virou opcional — confirme com o professor se ainda há vídeo |
| Tabela de variáveis: Ts típico 3–4 s, Td 3–8 s, Kp típico 0,05–0,1 °C/"excursão PWM" | Ts = Td/2 continua no texto (passo 6). O Kp da tabela está na escala de 10 bits do exemplo; em 16 bits o seu sai ~64× menor (~0,003) |

O roteiro ainda manda o ADC em 0 dB; mantivemos **2,5 dB**: em 0 dB a leitura satura em ~95 °C, justamente onde o
firmware precisa enxergar para desligar o aquecedor por segurança.

## Montagem (atualizada em 19/10/2026)

| Item | Ligação | No firmware (`proj1cdig_config.h`) |
|---|---|---|
| Placa | ESP32-WROOM-32 DevKit | alvo `esp32` |
| LM35 do processo (colado nos resistores) | **D35** | `PIN_ADC_TP 35` (ADC1_CH7) |
| LM35 do ambiente | **D34** | `PIN_ADC_TA 34` (ADC1_CH6) |
| Leitura | ADC interno, atenuação 2,5 dB (até ~1,25 V = 125 °C) | `ADC_ATENUACAO`, `ADC_JANELA_MS` |
| Aquecedor | 3 × 150 Ω **1/4 W** via transistor, PWM de **16 bits** a 1 kHz no **D25** | `PIN_PWM_HEATER 25`, `U_MAX 16000` |
| Cooler | via transistor, PWM de 10 bits (0–1023) a 25 kHz no **D26** | `PIN_PWM_COOLER 26` |
| "Filtro" de cada sensor | 1 µF eletrolítico em série com 2 × 150 Ω em paralelo (75 Ω) | — |

**Confira na bancada antes de ligar:**

- **Pino do aquecedor.** Ele não foi informado; o firmware usa o **D25**. Ligue o transistor do aquecedor no D25 ou troque
  `PIN_PWM_HEATER`. O cooler ficou no **D26**. Não use para PWM: D34–D39 (só entrada), D6–D11 (flash),
  D0/D2/D5/D12/D15 (definem o boot) e D1/D3 (USB-serial).
- **Transistor.** Os 3 × 150 Ω em 12 V puxam 240 mA durante o pulso. Se for o BC548 do roteiro (máx. 100 mA), ele aquece e a
  corrente deriva no meio da coleta; um TIP120/TIP122 ou um MOSFET logic-level (IRLZ44N) resolve.
- **Fonte do aquecedor = 12 V?** O limite de 16000 foi calculado para 12 V. Em 5 V cada resistor dissipa só 0,17 W a 100 % e o
  limite pode subir para 65535 — mas o aquecimento fica bem mais fraco.
- **Eletrolítico.** O lado + do capacitor vai para o lado da saída do LM35.

**Sobre o filtro:** 75 Ω + 1 µF em série, da saída ao GND, é o **damper** da Figura 13 do datasheet do LM35. Ele protege o
sensor contra a capacitância do fio e contra o ruído do motor, mas quase não filtra (o canto fica em ~2 kHz). A filtragem
de verdade é feita no firmware: cada amostra é a **média de 400 ms** de leituras (~1600 por sensor). 400 ms contém um número
inteiro de ciclos de 50 e de 60 Hz, que se cancelam na média, e a fase de cada rajada é sorteada em relação ao PWM do aquecedor.

---

## Onde cada coisa acontece

Há **três lugares** onde você digita coisas. Não misture:

| Lugar | O que você faz lá | Exemplo |
|---|---|---|
| **Terminal do ESP-IDF** (ou do VS Code com a extensão ESP-IDF), na pasta `proj1cdig\firmware` | Gravar o firmware na placa | `idf.py -p COM5 flash` |
| **Terminal comum** (PowerShell/cmd), na pasta `proj1cdig`, com o logger rodando | Mandar comandos para a ESP32 | `o 8320`, `e`, `k 2140 0.945` |
| **MATLAB** — Editor (`proj1cdig_m.m`) e Command Window | Identificar, projetar, simular, analisar | Ctrl+Enter numa seção; `sim Prj1CDig` |

## Os arquivos

| Arquivo | Função |
|---|---|
| `firmware/` | Programa da ESP32. Lê Tp (D35) e Ta (D34) no ADC interno e converte para °C, aciona aquecedor e cooler, **gera o PRBS** internamente, executa o PI anti-windup. Obedece comandos de texto. |
| `firmware/main/proj1cdig_config.h` | Todos os números ajustáveis do firmware. Mudou algo aqui → regravar. |
| `proj1cdig_logger.py` | Ponte PC ↔ ESP32: grava cada amostra em `dados/proj1cdig_<tag>_<data>.csv` **no instante em que chega**, plota ao vivo, mostra σ e o indicador **ESTÁVEL**, e repassa o que você digita. |
| `proj1cdig_m.m` | MATLAB: identificação (seções 1–3), projeto do PI (4–6), simulação com/sem anti-windup (7), valores para o firmware (8), análise dos ensaios (9). |

**Sobre o PRBS:** o professor deu o *gerador* (`prbs.m`) e uma coleta de exemplo da bancada dele
(`ma31500.mat`), não uma sequência para a sua bancada. O firmware gera o PRBS sozinho, com o mesmo
gerador de 9 bits que o `prbs.m` usa para b = 9: equivale a `prbs(N, 9, 60)`. Não precisa gerar nada no MATLAB.

---

## Conceitos usados no guia

- **LM35 + ADC interno:** o LM35 dá 10 mV/°C. O ADC da ESP32 lê a tensão, o firmware tira a média de 400 ms, aplica a
  calibração de fábrica do chip (eFuse) e converte para °C — o CSV já sai em °C. O ADC da ESP32 erra alguns % de ganho
  mesmo assim: **a calibração de dois pontos (Etapa D) não é opcional nesta montagem**.
- **u (contagens):** PWM de 16 bits do aquecedor, 0 a 65535 (= 0 a 12 V médio). O firmware nunca passa de `U_MAX` = 16000.
  O cooler usa outra escala, 0 a 1023 (10 bits): ele é só a perturbação.
- **U_MAX / Sat = 16000 (24 %):** limite nosso, não do professor. 3 × 150 Ω em 12 V dissipam 0,96 W por resistor a 100 %;
  resistores de 1/4 W aguentam até ~26 %. O roteiro fala em saturação 0–12 V (PWM cheio). É a saturação real da sua bancada — use o
  mesmo valor no MATLAB/Simulink e explique no vídeo. Com tão pouca folga, a saturação vai aparecer bem nos ensaios.
- **u0:** PWM que mantém Tp parada em 50 °C. **du:** quanto somar/subtrair de u0 para mover ~5 °C.
  Estimativa: o grupo de 1/2026 chegou a ~52 °C com 14 % de duty → u0 ≈ 8320, du ≈ 1680 (ganho ≈ 0,003 °C/contagem).
- **Estabilizado:** o logger compara a média dos últimos 30 s com a de 5 min atrás; diferença < 0,1 °C
  → **ESTÁVEL**. Na prática leva 20–30 min (polo lento ≈ 350 s; 3 constantes de tempo = 95 %, 5 = 99 %).
- **PRBS:** alterna u0 + du (nível alto) e u0 − du (nível baixo) em instantes pseudoaleatórios. Excita uma faixa
  larga de frequências sem correlação com o ruído — é o que permite identificar a planta.
- **511:** um gerador de 9 bits passa por 2⁹ − 1 = 511 estados antes de repetir (o "tudo zero" é proibido).
  A sequência é quase balanceada (256 altos, 255 baixos) e tem no máximo 9 bits iguais seguidos.
- **duracao_bit_s = 60 s:** um pouco acima do polo rápido (~50 s) — cada troca aparece acima do ruído;
  até 9 bits iguais seguidos = 9 min, que excitam o polo lento; 511 × 60 s = 8,5 h, o "tipicamente 8 h" do roteiro.
- **tempo_aquecimento_s:** tempo parado em u0 antes do PRBS começar, para sair do ambiente e estabilizar no P.O.
  2000 s na coleta longa (o `transit = 2000` do script do professor, descartado na identificação); 300 s no ensaio
  curto, que já começa quente.

---

## Etapa A — Preparar o PC (uma vez)

1. Instalar o **ESP-IDF v5.3** (o mais fácil: extensão "ESP-IDF" do VS Code).
2. Instalar Python e as bibliotecas do logger: `pip install pyserial matplotlib`.
3. Driver USB-serial da DevKit (CP2102 ou CH340), se a porta COM não aparecer no Gerenciador de Dispositivos.
4. MATLAB com **System Identification Toolbox**, **Control System Toolbox** e **Simulink**.
5. Copiar `Referencias\Prj1CDig\Prj1CDig\Prj1CDig.slx` e `Termico_FP.slx` **para `proj1cdig`** (não editar os originais).

## Etapa B — Gravar o firmware

1. Confira os pinos no `proj1cdig_config.h` (seção "Montagem" acima).
2. No terminal do ESP-IDF, em `proj1cdig\firmware`:

```
idf.py set-target esp32          ← só na primeira vez
idf.py -p COM5 flash
```

Se aparecer "Failed to connect", segure o botão **BOOT** da placa enquanto a gravação começa.
Feche qualquer `idf.py monitor` antes de abrir o logger: os dois não podem usar a porta ao mesmo tempo.

**Formato do logger** (terminal comum, na pasta `proj1cdig`):

```
python proj1cdig_logger.py COM5 --tag NOME --cmd "COMANDO"
```

- `--tag` vai no nome do arquivo; `--cmd` é o primeiro comando, enviado ao abrir (opcional).
- Com o logger aberto, **digite no terminal** (não na janela do gráfico) e Enter → vai para a ESP32.
- **Ctrl+C** encerra e fecha o CSV. Reabrir o logger **não reseta** a placa: o processo continua onde estava.
- A ESP32 responde a cada comando com `#ESTADO …` — use para conferir o que está valendo.

## Etapa C — Validar os sensores

```
python proj1cdig_logger.py COM5 --tag sensores --cmd s
```

1. Ao ligar deve aparecer `#ADC ok: Tp no GPIO35, Ta no GPIO34` e a linha da calibração de fábrica.
2. Tp e Ta devem ficar perto do termômetro (até ±2–3 °C antes de calibrar: LM35 e ADC somam erros).
   Leitura perto de 0 °C ou fixa = fio solto ou pino errado.
3. **Critério: σ(Tp) < 0,1 °C** no título do gráfico, com tudo desligado (é a resolução que o roteiro exige; o ideal é < 0,05).
4. Digite `wc 700` (cooler ligado): σ tem que continuar < 0,1 °C. Se piorar → captação de ruído:
   trançar os fios até os sensores, terra comum e curto, conferir o damper. Se não bastar, um RC de verdade no pino do ADC
   (ex.: 1 kΩ em série + 10 µF cerâmico ao GND) ajuda. Depois `w0`.

**Não avance sem passar nos itens 3 e 4** — foi o ruído do ADC interno que estragou a identificação do grupo de 1/2026.

## Etapa D — Calibração (obrigatória nesta montagem)

1. Com `s`, anote Tp e Ta lidos vs. termômetro de referência em dois pontos (ambiente e ~45–50 °C;
   para aquecer use `o 7700` e espere ESTÁVEL). Ta só dá para calibrar no ponto ambiente (offset).
2. Calcule ganho e offset: `T_real = GAIN · T_lido + OFFSET`.
3. Preencha `CAL_TP_GAIN`, `CAL_TP_OFFSET`, `CAL_TA_GAIN`, `CAL_TA_OFFSET` no `config.h` e regrave.

## Etapa E — Fixação definitiva e cooler

1. Resistores colados no LM35, sensor Ta a ~35 cm **fora do vento do cooler**, cooler preso.
   **Fotografe.** Nada pode mudar daqui até o fim do Prj2 — senão o modelo perde validade.
2. Zona morta do cooler (escala 0–1023): `wc 200`, `wc 250`, `wc 300`… até o motor partir. Anote o valor.
3. No `config.h`: `COOLER_OFFSET` e `COOLER_AMP` com `OFFSET − AMP` acima do valor de partida. Regrave.

## Etapa F — Achar o u0 (PWM que dá 50 °C)

```
python proj1cdig_logger.py COM5 --tag po --cmd "o 8320"
```

1. Espere aparecer **ESTÁVEL** (20–30 min).
2. Abaixo de 50 °C → `o 9600`; acima → `o 7400`. Repita até estabilizar em ~50 °C (cada 640 contagens ≈ 2 °C).
3. **u0 = o último valor de `o` que estabilizou em 50 °C.** Anote. (Exemplo: 8320.)
4. Se nem com `o 16000` chegar a 50 °C, falta potência: confira a fonte de 12 V e o transistor (seção "Montagem").

## Etapa G — Achar o du (quanto dá ±5 °C)

Ainda no mesmo logger, com Tp estável em 50 °C:

1. Digite `o` seguido de **u0 + 1920 já somado** (com u0 = 8320 → `o 10240`). Espere ESTÁVEL.
2. Ganho = (T_final − 50) / 1920. Ex.: (55,7 − 50) / 1920 = 0,00297 °C/contagem.
3. **du = 5 / ganho.** Ex.: 5 / 0,00297 ≈ 1680.
4. Confira: u0 + du ≤ 16000. Volte para `o <u0>` e Ctrl+C.

## Etapa H — Ensaio curto de PRBS (30 min)

```
python proj1cdig_logger.py COM5 --tag prbs_teste --cmd "p 8320 1680 60 300"
```

(`p u0 du duracao_bit_s tempo_aquecimento_s` — troque 8320 e 1680 pelos seus.)

1. Veja se Tp oscila em torno de 50 °C, sem passar muito de 45–55 °C. A excursão real fica um pouco menor que
   ±5 °C, porque cada bit é curto perto do polo lento — normal.
2. Se a oscilação for pequena demais (< ±2 °C), aumente du; se passar de 55 °C com folga, diminua.
3. Após 30 min, digite `x` e Ctrl+C.

## Etapa I — Coleta PRBS (~9,1 h, à noite)

1. Notebook na tomada, suspensão desligada, "suspensão seletiva USB" desligada, longe de sol e ar-condicionado.
2. Comece com a bancada **fria** (em ambiente):
   ```
   python proj1cdig_logger.py COM5 --tag prbs --cmd "p 8320 1680 60 2000"
   ```
   = 2000 s de aquecimento + 511 × 60 s de PRBS = 9,1 h. Ao fim aparece `#PRBS concluido`; a placa volta a u0.
3. De manhã: Ctrl+C. Conferir no `.log` que não houve `#TRIP` e no gráfico que não há buracos ou `nan`.
4. **Guarde o CSV** — tudo do Prj1 e do Prj2 sai dele.

## Etapa J — MATLAB: identificação e projeto (seções 0 a 8)

No MATLAB, **Current Folder = `proj1cdig`**. Abra `proj1cdig_m.m` no Editor.
Rodar uma seção = clicar dentro dela e **Ctrl+Enter** (Ctrl+Shift+Enter roda e avança). Não use F5 na primeira vez.

**Seção 0** — edite só isto e rode:
```matlab
GRUPO    = 2;
ARQ_PRBS = 'dados/proj1cdig_prbs_2026-xx-xx_xx-xx-xx.csv';
ARQ_MF   = '';
Sat      = 16000;
ABRIR_SISOTOOL = false;
```

| Seção | O que faz | Critério / ação |
|---|---|---|
| 1 | Carrega e plota a coleta | Sem buracos; Ta registrada o tempo todo |
| 2 | Identifica Kp, Tz, Tp1, Tp2, Td (`procest` P2DZ), testando 19 partições | **Td entre 4 e 8 s, Tz > 0, fit > 85 %**. Se Td ≈ 0 ou Tz < 0 → ruído/fixação: volte à Etapa C |
| 3 | Cria as variáveis dos `.slx` | Na Command Window: `sim Termico_FP` → modelo caixa-cinza vs. Tp medido (validação; boa figura para o vídeo) |
| 4 | Mp, Tp → ζ ≈ 0,59, ωn ≈ 0,156 rad/s, polo dominante desejado | — |
| 5 | PI contínuo pelo LGR (condição de ângulo e módulo) | Ajuste no sisotool (`ABRIR_SISOTOOL = true`) olhando degrau de referência, de perturbação e amplitude de u. Depois `sim Prj1CDig` = passo 5 do roteiro (saturação 0–Sat, AW ligado/desligado) |
| 6 | PI discreto: Ts = Td/2, ZOH, LGR em z, refino de z0 e K | Justificar no sisotool discreto (passo 6 do roteiro) |
| 7 | Simula degrau 50 → 55 °C com saturação, **com e sem** anti-windup | Diferença clara de sobrepasso |
| 8 | Imprime as linhas para o firmware | Copiar |

Passo 7 do roteiro: faça uma **cópia** do `Prj1CDig.slx`, troque o PI contínuo pelo discreto
(Ts, saturação 0–Sat, chave anti-windup) e simule. A seção 7 do `.m` serve de conferência.

> Referência de ordem de grandeza (modelo do roteiro reescalado para Kp ≈ 0,003 °C/contagem de 16 bits, grupo 2, Sat = 16000):
> Ts = 2,33 s, z0 ≈ 0,945, K ≈ 2140 cont/°C → Mp 10 %, tp 25,6 s em simulação linear. Degrau 50 → 55 °C com saturação:
> ~14 % de sobrepasso **sem** AW, ~0 % **com** AW. Seus números vão diferir.

## Etapa K — Levar o PI para a ESP32

1. Cole as linhas da seção 8 em `firmware/main/proj1cdig_config.h` (`CTRL_TS_MS`, `PI_K`, `PI_Z0`).
2. Regrave (`idf.py -p COM5 flash`). **O Ts só muda regravando.**
3. Ajuste fino de K e z0 sem regravar: no terminal do logger, `k <K> <z0>` (ex.: `k 2140 0.945`).
   Vale até a placa reiniciar; os valores finais vão para o `config.h`.

## Etapa L — Ensaio do passo 8: onda quadrada + perturbações (Figura 5)

O roteiro novo pede para **mostrar em tempo real, na tela do PC, o seguimento de uma referência em onda quadrada com
rejeição de perturbações**. O comando `e` do firmware faz o ensaio inteiro da Figura 5 sozinho, em 3600 s:

| Tempo | Referência | Cooler |
|---|---|---|
| 0–1000 s | onda quadrada 50 ↔ 60 °C, troca a cada 400 s (começa em 50) | desligado |
| 1000–1800 s | continua | **constante** (pico da triangular) |
| 1800–2600 s | continua | **triangular** (ω0 do grupo) |
| 2600–3600 s | continua | constante |
| fim | volta a 50 °C e continua em malha fechada | desligado |

O firmware avisa cada mudança (`#ENSAIO t=… s: …`), imprime o **ISE de 2100 a 3000 s** quando chega lá e, no fim,
`#ENSAIO concluido: ISE(2100-3000 s) = …`. Para mudar os níveis e o meio período: `e <r_baixa> <r_alta> <meio_periodo_s>`.

**Faça duas vezes, uma com e outra sem anti-windup, cada uma num arquivo** — é a comparação da Figura 5 do roteiro:

1. **Preparar** (sem arquivo importante): `python proj1cdig_logger.py COM5 --tag preparo --cmd "a 1" --cmd "c 50"`.
   Espere ESTÁVEL em 50 °C e dê Ctrl+C. O processo continua em malha fechada.
2. **Com anti-windup:** `python proj1cdig_logger.py COM5 --tag ensaio_aw1 --cmd "a 1" --cmd "e"`.
   **Grave a tela** com o gráfico do logger: essa é a demonstração em tempo real do passo 8.
   Espere `#ENSAIO concluido` (1 h), espere ESTÁVEL em 50 °C e dê Ctrl+C.
3. **Sem anti-windup:** `python proj1cdig_logger.py COM5 --tag ensaio_aw0 --cmd "a 0" --cmd "e"`.
   Espere `#ENSAIO concluido`, digite `a 1` e depois `x`, e dê Ctrl+C.

Não digite outros comandos durante o ensaio: `x`, `s`, `o`, `p`, `c`, `r` e `w…` encerram o perfil (só `a` e `k` não).

Com U_MAX = 16000 e u0 ≈ 8320, a folga até a saturação é de ~7700 contagens (~23 °C em regime). O degrau de 10 °C
da onda quadrada satura o aquecedor por dezenas de segundos — sem anti-windup, isso vira um sobrepasso bem maior,
que é o efeito que o projeto pede para mostrar. Na simulação com o modelo de exemplo: Mp ≈ 1 % com AW contra ≈ 29 % sem.

Ensaios avulsos continuam disponíveis se quiser medir algo isolado: degrau (`c 50`, depois `r 55`), perturbação
constante (`wc 600`), senoidal (`ws`) e triangular (`wt`), cada um com `--tag` próprio.

Se Mp ou tp fugirem muito da especificação (o passo 8 pede para verificar): ajuste com `k` e repita o ensaio com AW;
se não resolver, volte à Etapa J.

## Etapa M — MATLAB: análise dos ensaios (seção 9)

Para cada arquivo de ensaio:
1. Seção 0: `ARQ_MF = 'dados/proj1cdig_ensaio_aw1_….csv';` → Ctrl+Enter.
2. Seção 9 → Ctrl+Enter. Imprime o **ISE (2100–3000 s)**, o tempo com o aquecedor saturado, e Mp e tp no primeiro degrau
   da onda quadrada (50 → 60 °C, em t = 400 s), e plota Tp, ref, u e cooler.
3. Repita com `ensaio_aw0` e compare.

| Momento | Seções do `.m` |
|---|---|
| Depois da coleta de 9 h | 0 → 8 (+ `sim Termico_FP` e `sim Prj1CDig`) |
| Depois de cada ensaio | 0 (trocando `ARQ_MF`) e 9 |

## Etapa N — Vídeo (opcional: saiu do roteiro novo, confirme com o professor)

1. Montagem física e esquema — 30 s
2. Coleta PRBS e modelo identificado, com o fit de validação — 1 min
3. Projeto: polo desejado a partir de Mp/Tp, LGR contínuo e discreto, Ts = Td/2 — 1 min 30 s
4. Simulação com saturação, com e sem anti-windup (e por que Sat = 16000) — 1 min
5. Ensaio da Figura 5 com e sem AW, lado a lado com a simulação — 1 min 30 s
6. Rejeição de perturbação constante e triangular em tempo real, e o ISE — 1 min
7. Conclusões — 30 s

Há um modelo IFAC em `Referencias/OverleafCDig.zip`; o roteiro do Prj1 só exige o vídeo — confirmar com o professor.

---

## Referência rápida

### Comandos do firmware (digitar no terminal do logger)

| Comando | Faz |
|---|---|
| `h` | Lista os comandos |
| `x` | Para tudo (aquecedor e cooler em 0) |
| `s` | Só lê sensores, aquecedor desligado |
| `o <u>` | Malha aberta, aquecedor fixo em u (0–16000, escala de 16 bits) |
| `p <u0> <du> <duracao_bit_s> <tempo_aquecimento_s>` | PRBS |
| `c [ref]` | Malha fechada PI (padrão 50 °C) |
| `r <ref>` | Degrau de referência |
| `a 0` / `a 1` | Anti-windup desligado / ligado |
| `e [r_baixa r_alta meio_periodo_s]` | Ensaio do passo 8 (Figura 5): onda quadrada + perturbações, 3600 s, calcula o ISE |
| `k <K> <z0>` | Troca o PI sem regravar |
| `w0` / `wc <w>` / `ws` / `wt` | Cooler (0–1023): off / constante / senoide / triangular |

Proteção: se Tp passar de 95 °C, o firmware desliga o aquecedor e avisa `#TRIP`.

### Onde alterar

| O quê | Onde |
|---|---|
| P.O. e ω0 | `config.h`: `GROUP_PO_C` (50), `GROUP_W0_RAD_S` (0,065) |
| Grupo no MATLAB | `proj1cdig_m.m`: `GRUPO` (2) |
| Pinos | `config.h`: `PIN_ADC_TP` (35), `PIN_ADC_TA` (34), `PIN_PWM_HEATER` (25), `PIN_PWM_COOLER` (26) |
| Leitura do ADC | `config.h`: `ADC_ATENUACAO` (2,5 dB), `ADC_JANELA_MS` (400), `ADC_LEITURAS_POR_MS` |
| Saturação | `config.h`: `U_MAX` (16000) **e** `proj1cdig_m.m`: `Sat` (16000) — sempre iguais |
| Ensaio do passo 8 | `config.h`: `ENSAIO_AMPL_C` (10), `ENSAIO_MEIO_PERIODO_S` (400), tempos das fases, `ISE_T0_S`/`ISE_T1_S` |
| Calibração | `config.h`: `CAL_TP_GAIN/OFFSET`, `CAL_TA_GAIN/OFFSET` |
| Cooler | `config.h`: `COOLER_OFFSET`, `COOLER_AMP` |
| PI e Ts | `config.h`: `CTRL_TS_MS`, `PI_K`, `PI_Z0` (ou `k` ao vivo) |
| u0, du do PRBS | no próprio comando `p` |
| Arquivos de dados | `proj1cdig_m.m`: `ARQ_PRBS`, `ARQ_MF` |

Qualquer mudança no `config.h` → regravar (`idf.py -p COM5 flash`).

### Riscos

- **Ruído do ADC interno acima do critério da Etapa C** → identificação ruim. Não comece a Etapa I sem passar.
- **Resistores de 1/4 W** → nunca tire o limite de 16000 sem trocar os resistores ou baixar a tensão do aquecedor.
- **Transistor subdimensionado** (BC548 com 240 mA) → ganho do aquecedor muda durante a coleta.
- **Montagem que muda depois da Etapa I** → modelo inválido para o Prj1 e o Prj2.
- **Reset da placa durante a coleta** (queda de energia, Windows suspendendo o USB) → a placa fica parada com o
  aquecedor desligado e a coleta precisa recomeçar.
- **Sat diferente entre firmware e simulação** → a comparação simulação × experimento perde sentido.
