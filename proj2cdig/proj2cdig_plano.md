# proj2cdig — Plano do Projeto 2 (Prj2CDig)

**Controle PI anti-windup com observador de perturbações senoidais**, ENE0167 Controle Digital, UnB
(Prof. Adolfo Bauchspiess), grupo 2.

Este arquivo tem dois papéis:
1. Explicar ao grupo **o que** está sendo desenvolvido e **por quê**, passo a passo.
2. Servir de **contexto para o Claude Code**: caminhos, convenções, números já definidos e regras do repositório.

Atualize a seção [Status](#10-status) conforme as fases forem concluídas.

---

## 1. Objetivo em uma frase

Manter o processo térmico do Prj1 em 50 °C com resposta de 1ª ordem (Tr = 30 s, sem sobrepasso) e
**zerar o efeito da perturbação senoidal do cooler**, que o PI do Prj1 só atenuava (sobrou ±0,4 a 0,8 °C).

Fonte: `Referencias/Prj2CDig/Prj2CDig.pdf` (roteiro), `Prj2CDig_m.m` e `Prj2CDig.slx` (modelos do professor).

## 2. Especificações do grupo 2

| Item | Valor | Origem |
|---|---|---|
| Ponto de operação (P.O.) | 50 °C | tabela de grupos |
| Resposta à referência | 1ª ordem, Mp = 0 % | §3 do roteiro |
| Tempo de subida Tr (10–90 %) | **30 s** | tabela de grupos |
| Frequência da senoide ω0 | **0,065 rad/s** (período 96,7 s) | tabela de grupos |
| Período de amostragem | **Ts = Td** (≈ 5,49 s; no Prj1 era Td/2) | §3 do roteiro |
| Rejeição | perturbações constantes por partes e senoidais em ω0 | §3 do roteiro |
| Resolução de temperatura | ≤ 0,1 °C | §3 do roteiro |
| Ensaio | 1 h com Ta registrada | passo 7 |
| Entregas | relatório (template IFAC), vídeo de 5–7 min, apresentação presencial | roteiro |

## 3. O que vem pronto do Prj1

| Item | Valor / onde está |
|---|---|
| Bancada | ESP32-WROOM; LM35 do processo no GPIO35, ambiente no GPIO34 (ADC interno, média de 400 ms); aquecedor 3 × 150 Ω 1/4 W em paralelo, 12 V, PWM 16 bits a 1 kHz no GPIO25; cooler 12 V, PWM 10 bits a 25 kHz no GPIO26 |
| Unidade do atuador | **contagens** de 16 bits (0–65535), limitadas a **U_MAX = 21000** |
| Ponto de operação | u0 = 10200 contagens → Tp ≈ 50 °C |
| Modelo (PRBS de 12/11/2026, `proj1cdig/outputs/E/modelo_identificado.mat`) | y = Tp − Ta; **Kp = 0,0022006 °C/cont., Tz = 321,05 s, Tp1 = 441,96 s, Tp2 = 32,77 s, Td = 5,486 s**; fit 92,8 % (MATLAB) e 91,3 % em todos os dados |
| Firmware | `proj1cdig/firmware/` (ESP-IDF). Modos IDLE/SENS/OPEN/PRBS/CTRL, ensaio `e`, cooler `w0/wc/ws/wt` |
| Logger | `proj1cdig/proj1cdig_logger.py` (+ `proj1cdig_plot.py`), CSV `t_s,k,modo,u,w,tp,ta,ref,aw,pc_time` |
| Sequência de ensaios | `instrução.md` na raiz |

Modelo:

```
G(s) = Tp−Ta / U = Kp (1 + Tz s) / ((1 + Tp1 s)(1 + Tp2 s)) · e^(−Td s)
```

---

## 4. A ideia, explicada

### 4.1 Por que o PI do Prj1 não basta
Um controlador só zera em regime o erro causado por sinais que ele "sabe gerar" internamente (**princípio
do modelo interno**). O integrador do PI gera degraus, então rejeita perturbação constante. Uma senoide
em ω0 precisa de um modelo que oscile em ω0, e o PI não tem.

### 4.2 A estrutura do Prj2 (Figura 2 do roteiro)

```
                 ┌──────────── canal PI ────────────┐
  r ──┬── Nb ───►(+)                                 │
      │           ▲                                  ▼
      └─►(+)─ e ─►∫─ xi ─► Ki ─►(+)── u_PI ──►(+)── u ──► [sat 0..U_MAX] ──► processo ──┬── y
          ▲ −                                    ▲ −  ▲ −                                │
          │                                      │    │                                  │
          y                               K·x̂ ───┘    └── d̂ (perturbação estimada)       │
                                                 ▲         ▲                             │
                                                 └── observador aumentado ◄── u, y ──────┘
                                                    (modelo do processo + gerador de senoide)
```

- **Canal PI**: proporcional aplicado à **referência** (ganho Nb) e integral aplicado ao **erro**.
  Assim a malha não ganha zeros indesejados e o projeto sai "por inspeção".
- **Realimentação de estados** K·x̂: posiciona os polos de malha fechada.
- **Observador aumentado**: estima os estados do processo e a senoide da perturbação, d̂. O controle
  subtrai d̂, cancelando a perturbação antes que ela apareça em y.

### 4.3 Espaço de estados do processo (forma modal, como no script do professor)

```
A = [−1/Tp1   0    ]    B = [Kp]    C = [a/Tp1   b/Tp2]
    [  0    −1/Tp2 ]        [Kp]

a = (Tz − Tp1)/(Tp2 − Tp1) = 0,29549     b = 1 − a = 0,70451
```

Cada estado é uma das duas "constantes de tempo" do processo. O atraso Td fica de fora no contínuo
e entra no discreto (Fase 2).

### 4.4 Controlador aumentado (canal I)
Acrescenta-se o integrador do erro, xi' = r − y, e escolhem-se 3 polos com `acker`:

| Polo | Valor | Por quê |
|---|---|---|
| p1 | −2,2/(3·Tr) = −0,02444 | será **cancelado** pelo zero do canal PI, com Nb = −Ki/p1 |
| p2 | −1/Tz = −0,003115 | fica **sobre o zero do processo** e se cancela com ele |
| p3 | 3·p1 = −2,2/Tr = −0,07333 | é o **único polo que sobra**: define Tr |

Resultado: Y/R = 1/(τ s + 1) com τ = Tr/2,2 = 13,6 s, ou seja, 1ª ordem e Mp = 0.

**Pré-cálculo contínuo, sem atraso** (Python, conferir no MATLAB):

```
K  = [1,309  29,64]     Ki = 36,75     Nb = 1503,3
Tr = 29,9 s  ·  Mp = 0 %  ·  ganho DC = 1
u logo após um degrau de r: Nb = 1503 contagens por °C
  → degrau de 5 °C: u0 + 7500 ≈ 17700 (abaixo de U_MAX)
  → degrau de 10 °C: ≈ 25200 (satura: o anti-windup tem papel)
```

### 4.5 Observador aumentado
O modelo do observador junta o processo e um **gerador de senoide**:

```
As = [  0    1 ]      Cs = [1 0]        (a saída do gerador soma na entrada do processo)
     [−ω0²   0 ]

Ao = [A   B·Cs]       Co = [C   0 0]
     [0    As ]
```

Os ganhos L saem de `acker(Ao', Co', polos)'`. O professor usa polos em 12× os do processo
(`12*[−1/Tp1, −1/Tp2, −1/Tp1, −1/Tp1]`): é o ponto de partida, a ser ajustado. Polos rápidos demais
amplificam o ruído do sensor; lentos demais demoram a "aprender" a senoide.
O primeiro estado do gerador é **d̂**, a perturbação estimada, que é subtraída de u.

### 4.6 Discretização (o que o roteiro chama de "não trivial")
- **Ts = Td**: o atraso vira **exatamente uma amostra**, representada por um estado extra
  `u_atrasado(k) = u(k−1)`, conhecido (é o que foi aplicado).
- Processo e gerador discretizados por ZOH; o gerador vira uma rotação exata
  `Φs = expm(As·Ts)`.
- Polos do controlador e do observador levados para z por `z = e^(s·Ts)`; o polo extra do atraso
  em z = 0 (resposta mais rápida possível).
- Integrador discreto: `xi(k+1) = xi(k) + Ts·(r − y(k))`.

### 4.7 Anti-windup
Mesmo critério do Prj1 (integração condicional): o integrador **congela** quando u passa de U_MAX
com erro positivo, ou fica abaixo de 0 com erro negativo. **O observador recebe sempre o u saturado**
(o que de fato foi aplicado). Senão ele "acha" que o aquecedor fez mais do que fez e a estimativa
diverge.

---

## 5. Fases

### Fase 0 — Ajustar a perturbação na bancada (~1 h, pode ser em paralelo com a Fase 1)
Depois da remontagem de novembro, w = 680 quase não afeta Tp (Δu em regime de só ~1300 contagens;
em outubro eram ~4750). Nos testes de 10–11/11: w = 700–900 tirou ~0,5 °C/min e w = 1023 derrubou 13 °C
em 2 min.

1. Malha aberta, `o 10200` até ESTÁVEL em 50 °C.
2. `wc 700`, `wc 800`, `wc 900`, `wc 1000` e `wc 1023`, **5 min cada** e `w0` + ESTÁVEL entre eles. Anotar a queda de Tp.
3. Escolher `COOLER_OFFSET ± COOLER_AMP` para que `ws` cause **±1–2 °C** em malha aberta, sem cruzar a zona morta.
4. **Critério**: com essa faixa, a oscilação aparece claramente no ensaio com o observador desligado.

### Fase 1 — Projeto contínuo no MATLAB (passo 1 do roteiro)
- Criar `proj2cdig/proj2cdig_m.m` a partir de `Referencias/Prj2CDig/Prj2CDig_m.m` (copiar, nunca editar o original).
- Substituir: modelo da tabela da seção 3 (ou carregar `modelo_identificado.mat`), `umax = 21000`, `w0 = 0.065`,
  `PO = 50`, Ta medida, polos da seção 4.4.
- Copiar `Prj2CDig.slx` para `proj2cdig/` e simular.
- **Critério**: Tr ≈ 30 s, Mp = 0, perturbação senoidal rejeitada em regime, d̂ acompanhando a senoide.
- **Entrega**: figuras da simulação contínua, com observador ligado e desligado.

### Fase 2 — Projeto discreto (passos 2 a 5)
- No mesmo `.m`, seção nova: c2d (ZOH) com Ts = Td, estado do atraso, gerador discreto, K/Ki/Nb e L no plano z.
- Simulação discreta **com atraso e anti-windup** (Simulink ou laço em MATLAB/Python), incluindo saturação em 0..21000.
- Refinar polos do controlador e do observador, amplitude da perturbação e P.O.
- **Critério**: os mesmos da Fase 1, agora com atraso e saturação; u dentro de 0..21000 em regime.
- **Entrega**: matrizes discretas finais impressas no formato `#define` (como a seção 8 do `proj1cdig_m.m`).

### Fase 3 — Firmware (passo 6)
- Criar `proj2cdig/firmware/` como **cópia** de `proj1cdig/firmware/` (o Prj1 não pode mudar).
- Novo modo (sugestão `OBS`) com período `OBS_TS_MS = round(Td·1000)`, mantendo todos os modos do Prj1.
- Lei por amostra, em variáveis de desvio (y = Tp − Ta − y0, u em torno de u0):

  ```
  1. lê Tp, Ta → y(k)
  2. observador:   x̂(k+1) = Φo·x̂(k) + Γo·u_ap(k−1) + L·(y(k) − Co·x̂(k))
  3. integrador:   se não (u>U_MAX e e>0) e não (u<0 e e<0): xi += Ts·(r − y)
  4. controle:     u = u0 + Nb·r_desvio + Ki·xi − K·x̂_proc − obs_on·d̂
  5. u_ap = sat(u, 0, U_MAX) → PWM;  guardar u_ap para o próximo passo
  ```

- Comandos novos: `b 0|1` (observador da senoide desligado/ligado; sugestão de letra) e um perfil automático de 1 h
  no estilo do `e` (degrau de perturbação constante em 1000 s, senoide `ws` a partir de 1800 s, Fig. 4 do roteiro).
- CSV com colunas extras: `dhat` (perturbação estimada) e `obs` (0/1). Atualizar o logger para aceitá-las.
- Gravar de um caminho curto, `C:\cdig\fw2` (o ESP-IDF falha com o caminho longo com espaços).

### Fase 4 — Ensaios (passo 7, ~3 h de bancada)
1. **Preparo**: `o 10200`, depois o modo OBS em 50 °C até ESTÁVEL.
2. **Ensaio 1 h com observador desligado** (`b 0`): deve parecer a Fig. 3 do roteiro, com a senoide atenuada mas presente.
3. **Ensaio 1 h com observador ligado** (`b 1`): deve parecer a Fig. 4, com a senoide rejeitada e d̂ acompanhando.
4. **Degrau de referência** de 50 para 52 °C sem saturação: medir Tr e confirmar Mp = 0.
- Cada ensaio num CSV próprio em `proj2cdig/dados/`, com `--tag` descritivo.

### Fase 5 — Análise e entregas
- Script de análise: Tr, Mp, amplitude residual da senoide (RMS do erro na janela da senoide), d̂ × w aplicado, tempo saturado.
- Relatório no template IFAC, vídeo de 5–7 min e apresentação presencial.

---

## 6. Ordem prática

```
Fase 0 (bancada) ─┐
                  ├─► Fase 2 ─► Fase 3 ─► Fase 4 ─► Fase 5
Fase 1 (MATLAB) ──┘
```

---

## 7. Contexto para o Claude Code

### 7.1 Estrutura e caminhos

| Caminho | Conteúdo | Regra |
|---|---|---|
| `C:\Users\Pedro\Controle Digital 26_2\` | raiz do repositório `pedro-acviana/Projeto-de-Controle-Digital` | — |
| `proj1cdig/` | Prj1 completo (firmware, logger, MATLAB, dados, outputs A–E) | **não alterar** sem pedido explícito |
| `proj1cdig_relatorio/` | relatório do Prj1 (LaTeX, template IFAC) | idem |
| `proj2cdig/` | **tudo do Prj2 vai aqui** | nomes de arquivo com prefixo `proj2cdig` |
| `Projeto-de-Controle-Digital/` | submódulo: repositório do colega Cristiano (`CristianoCTS`) | **somente leitura**. Nunca editar, commitar ou mover nada dentro |
| `Referencias/` | roteiros, scripts e modelos do professor; livro | **somente leitura**; fora do Git (direitos autorais) |
| `C:\cdig\fw` | cópia de compilação do firmware do Prj1 | criar `C:\cdig\fw2` para o Prj2 |
| `instrução.md` | sequência de comandos dos ensaios do Prj1 | modelo para escrever a do Prj2 |

A branch `Prj2CDig` do repositório do Cristiano tem o script e os resultados dele da parte 2. Serve de referência para ler, sem copiar para o nosso repositório.

### 7.2 Convenções técnicas
- Atuador sempre em **contagens de 16 bits**; U_MAX = 21000. Cooler em 10 bits (0–1023), zona morta ≈ 530.
- Modelo identificado com **y = Tp − Ta** (variáveis de desvio em torno de u0 e da média).
- Firmware em **ESP-IDF v5.3/v6.1** (C, FreeRTOS); toda configuração em `*_config.h` com `#define`.
- Mensagens da placa começam com `#`; linhas de dados com `D,`. O logger descarta linhas inválidas.
- Comandos de uma letra + argumentos, terminados em `\n`. A placa responde `#ESTADO ...` a cada comando aceito.
- MATLAB organizado em seções `%%`, uma etapa por seção, com prints dos critérios.

### 7.3 Git
- Commits em nome de **Pedro Viana <pedroacviana@outlook.com>**, **sem** linhas de coautoria de IA.
- Commit e push **só quando pedido**.
- `.gitignore` já exclui `Referencias/`, `*.mp4`, `*.zip`, `build/`, `sdkconfig`.
- Outro colaborador (GitHub `Noites-Dias`) também envia commits: faça `git pull` antes de começar.

### 7.4 Ambiente
- Windows + PowerShell. Porta da ESP em `COMx` (Gerenciador de Dispositivos).
- Logger: `python proj1cdig_logger.py COMx --tag NOME --cmd "COMANDO"`, rodado dentro da pasta do projeto.
- Não abrir `idf.py monitor` e o logger ao mesmo tempo.

---

## 8. Decisões já tomadas

| Decisão | Motivo |
|---|---|
| Modelo de 12/11/2026 | bancada remontada; fit 92,8 %; Tp1 = 442 s, próximo da faixa do roteiro (200–400 s) |
| U_MAX = 21000 mantido | janela 16300 < U_MAX < ~28500 respeitada na bancada atual |
| Polo sobre −1/Tz e p1 cancelado por Nb | receita do roteiro (passo 3) → Y/R de 1ª ordem |
| Polo restante em −2,2/Tr | Tr = 30 s do grupo 2 |
| Observador recebe u saturado | evita divergência da estimativa durante a saturação |

## 9. Questões em aberto

- [ ] Faixa do cooler (`COOLER_OFFSET`/`COOLER_AMP`) para a senoide ficar visível (Fase 0).
- [ ] Onde a perturbação entra no modelo discreto: antes ou depois do atraso de uma amostra (verificar na simulação).
- [ ] Polos finais do observador (compromisso entre ruído e velocidade de convergência).
- [ ] Perfil exato do ensaio de 1 h (degraus de referência além das perturbações?): confirmar com a Fig. 4 do roteiro.

## 10. Status

| Fase | Situação |
|---|---|
| 0 — Perturbação na bancada | pendente |
| 1 — Projeto contínuo | pendente (pré-cálculo da seção 4.4 feito) |
| 2 — Projeto discreto | pendente |
| 3 — Firmware | pendente |
| 4 — Ensaios | pendente |
| 5 — Análise e entregas | pendente |
