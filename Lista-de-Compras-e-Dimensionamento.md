# Bancada Térmica ENE0167 — lista de compras e dimensionamento

Prj1CDig + Prj2CDig · Controle Digital · ENE/FT/UnB
Versão web, sempre a mais atual (inclui o anexo sobre usar o ADC interno da ESP32 e a
conferência contra o datasheet do LM35): https://claude.ai/code/artifact/e24f381e-e892-4b98-b08b-216fdb2786db

> Este .md é um resumo. Correções de 02/09/2026: INA128 (não INA126), damper R-C 75 Ω + 1 µF
> adicionado ao Bloco B, e fundos de escala reais do ESP32-S3 (12 dB = 3100 mV, 0 dB = 950 mV).

---

## 1. As sete armadilhas

### CRÍTICO — O ADC interno do ESP32 não entrega os 0,1 °C exigidos
Requisito: resolução ≤ 0,1 °C = 1 mV na saída do LM35.

**Evidência no próprio repositório:** em `Outputs/Output2026-05-20_22-49-55.txt` a temperatura
ambiente lê 23,53 → 26,75 → 20,39 °C em amostras consecutivas de 1 s (±3 °C de ruído).
Consequência: os modelos em `MatlabResults/` saíram com Td ≈ 0 s e Tz negativo (fit 69–89%),
quando o roteiro espera Td entre 4 e 8 s e Tz ≈ +250 s. O ruído contaminou a identificação inteira.

**Correção:** ADS1115 (16 bits, referência interna, PGA, I2C) → 62,5 µV/LSB = 0,006 °C. ~R$ 21.
Também: o `main.c` usa `ADC_ATTEN_DB_12` (FS 3,3 V) para um sinal que nunca passa de 1,0 V.

### CRÍTICO — O BC548 do roteiro não aguenta o aquecedor
3 × 150 Ω em paralelo = 50 Ω → 240 mA em 12 V. Ic máx do BC548 = 100 mA.
Ele esquenta, o ganho cai e a corrente deriva no meio da coleta.

**Correção:** MOSFET logic-level **IRLZ44N** nos dois canais (aquecedor e cooler).

### ALTO — Resistor de 1/4 W queima acima de 26% de duty
O próprio Prj2 avisa. Mas é preciso potência sobrando para o aquecimento vencer o cooler.

**Correção:** 150 Ω em **1/2 W** (duty utilizável vai de 26% para 52%, corpo quase igual).
Evite 2 W: massa térmica alta demais. Some um limite de duty em firmware.

### ALTO — LM35 alimentado em 3,3 V está fora do datasheet
A Figura 1 mostra 3,3 V, mas o datasheet especifica Vs de **4 V a 30 V**.

**Correção:** alimentar os dois LM35 pelo pino **5 V** do devkit. A saída em 100 °C continua
sendo 1,000 V, segura para o ADC. 100 nF de desacoplamento junto de cada sensor.

### ALTO — O ADC2 do ESP32 para de funcionar com o WiFi ligado
A Figura 1 mostra MQTT. O rádio toma posse do ADC2.

**Correção:** usar somente ADC1 — ou o ADS1115 por I2C, que é imune.

### MÉDIO — Cooler sem diodo de roda-livre / zona morta de partida
**Correção:** 1N4007 em antiparalelo (catodo no +12 V) + 100 µF e 100 nF no trilho de 12 V.
O motor só parte acima de ~30% de duty: a perturbação senoidal do Prj2 precisa de offset DC
(ex.: 45% ± 25%), não pode cruzar o zero.

### MÉDIO — Placa USB-C sem os resistores CC de 5,1 kΩ
Placas baratas omitem os resistores em CC1/CC2 e não ligam com cabo C→C.

**Correção:** DevKitC-1 oficial ou clone de vendedor conhecido + cabo A→C de dados como plano B.

---

## 2. Memória de cálculo

### 2.1 Aquecedor
```
Req = 150 / 3        = 50 Ω
I   = 12 / 50        = 240 mA
Ptot= 12 × 0,240     = 2,88 W    (100% de duty)
P/resistor           = 0,96 W    ← 3,8× o rating de 1/4 W
```

| Duty | P total | P/resistor | 1/4 W | 1/2 W | 1 W |
|-----:|--------:|-----------:|:------|:------|:----|
| 10%  | 0,29 W  | 0,096 W    | ok | ok | ok |
| 15%  | 0,43 W  | 0,144 W    | ok | ok | ok |
| 25%  | 0,72 W  | 0,240 W    | no limite | ok | ok |
| 40%  | 1,15 W  | 0,384 W    | QUEIMA | ok | ok |
| 50%  | 1,44 W  | 0,480 W    | QUEIMA | no limite | ok |
| 75%  | 2,16 W  | 0,720 W    | QUEIMA | QUEIMA | ok |
| 100% | 2,88 W  | 0,960 W    | QUEIMA | QUEIMA | no limite |

Duty máximo seguro: **26% com 1/4 W · 52% com 1/2 W · 104% com 1 W**.
O grupo de 1/2026 operou em ~14% e chegou a 52 °C → faixa de trabalho esperada 10–25%.

### 2.2 Driver PWM
```
ID = 240 mA (aquecedor) · ~250 mA (cooler) · VDS = 12 V
IRLZ44N @ VGS = 3,3 V → RDS(on) ≈ 0,04–0,10 Ω
   queda 10–24 mV · dissipação 2–6 mW · sem dissipador
Rgate      = 220 Ω  → pico de 15 mA no GPIO (limite 20 mA)
Rpull-down = 10 kΩ do gate ao GND (mantém desligado no boot)
```
ATENÇÃO: tem que ser IRL**Z**44N (logic-level). O IRF540N precisa de 10 V no gate.

### 2.3 Filtro RC na saída do LM35

| R · C           | τ       | fc      | Fase em ω0 | Rej. 60 Hz | Erro de ganho* |
|-----------------|--------:|--------:|-----------:|-----------:|---------------:|
| 1 kΩ · 10 µF    | 0,010 s | 15,9 Hz | 0,06°      | −12 dB     | 0,02% |
| **10 kΩ · 10 µF** | **0,100 s** | **1,59 Hz** | **0,56°** | **−32 dB** | **0,20%** |
| 22 kΩ · 10 µF   | 0,220 s | 0,72 Hz | 1,24°      | −38 dB     | 0,45% |
| 100 kΩ · 10 µF  | 1,000 s | 0,16 Hz | 5,6°       | −51 dB     | 2,0%  |

\* divisor com a impedância de entrada do ADS1115 (≈ 4,9 MΩ).

**Escolha: R = 10 kΩ + C = 10 µF cerâmico X7R** (NÃO eletrolítico — a corrente de fuga sobre
10 kΩ vira erro de offset de vários graus). Mais 100 nF direto no pino de saída do LM35.
Resto da filtragem é digital: sobreamostrar (ex. 64 leituras a 128 SPS) + mediana e EWMA
que já existem no `main.c`.

### 2.4 Resolução — LM35 a 10 mV/°C, faixa útil 25–95 °C = 250–950 mV

| Conversor            | Bits | FS      | LSB     | °C/LSB | Veredito |
|----------------------|-----:|--------:|--------:|-------:|----------|
| ESP32 ADC · 12 dB    | 12   | 3,3 V   | 806 µV  | 0,081  | no papel passa; na prática ±3 °C |
| ESP32 ADC · 0 dB     | 12   | 1,1 V   | 269 µV  | 0,027  | satura em 95 °C; INL ruim |
| **ADS1115 · ±2,048 V** | 16 | 2,048 V | 62,5 µV | 0,006  | **16× de folga sobre o requisito** |
| ADS1115 · ±1,024 V   | 16   | 1,024 V | 31,3 µV | 0,003  | só 2% de folga acima de 100 °C |

Usar PGA ±2,048 V. Endereço I2C 0x48 com ADDR no GND.

### 2.5 Fonte de 12 V
```
aquecedor 240 mA + cooler ~250 mA = ~490 mA → fonte de 12 V / 2 A (4× de folga)
ESP32 pelo USB. GND da fonte OBRIGATORIAMENTE comum com o GND do devkit.
Nunca alimentar aquecedor ou cooler pelo pino 5 V da placa.
```

---

## 3. Lista de compras

### Bloco A — Essencial (R$ 350–512)

| Item | Especificação | Qtd | R$ est. | Por quê |
|------|---------------|----:|--------:|---------|
| ESP32-S3-DevKitC-1 | N16R8 · 16 MB flash · 8 MB PSRAM · USB-C duplo | 1 | 95–120 | WiFi+BLE5, USB nativo, 45 GPIOs, PSRAM — rende em projetos pessoais depois |
| ADS1115 | Módulo I2C 16 bits, 4 canais, PGA | 1 | 21–30 | Resolve a armadilha nº 1 |
| LM35DZ | TO-92, 0–100 °C, 10 mV/°C | 4 | 48–72 | 2 em uso + 2 reservas (inverter a pinagem queima) |
| IRLZ44N | MOSFET N logic-level TO-220 | 4 | 32–56 | Aquecedor + cooler + 2 reservas |
| Resistor 150 Ω | 1/2 W filme de carbono ±5% | 10 | 3–6 | 3 formam o aquecedor |
| Cooler 12 V | 80 mm, 2 ou 3 fios, 0,15–0,25 A | 1 | 20–35 | Perturbação constante e senoidal |
| Fonte 12 V / 2 A | Chaveada, P4 + adaptador P4→borne | 1 | 35–55 | Saturação especificada em 0–12 V |
| 1N4007 | Diodo 1 A / 1000 V | 5 | 1–3 | Roda-livre do cooler |
| Protoboard 830 | Com trilhos de alimentação | 2 | 50–70 | Separar lógica de potência reduz ruído |
| Kit de jumpers | 120 un · M-M, M-F, F-F | 1 | 25–35 | — |
| Cabo USB-A → USB-C | De DADOS, 1,5 m | 1 | 20–30 | Plano B contra placas sem resistor CC |

### Bloco B — Passivos (R$ 19–35)

| Item | Especificação | Qtd | R$ | Onde entra |
|------|---------------|----:|---:|------------|
| Resistor 10 kΩ | 1/4 W | 10 | 2 | Filtro RC + pull-down dos gates |
| Resistor 220 Ω | 1/4 W | 10 | 2 | Gate dos MOSFETs |
| Resistor 4,7 kΩ | 1/4 W | 5 | 1 | Pull-up I2C, se o módulo não trouxer |
| Capacitor 10 µF | Cerâmico X7R ou filme 25 V — NÃO eletrolítico | 5 | 8–15 | Filtro RC dos sensores |
| Capacitor 100 nF | Cerâmico | 10 | 3 | Desacoplamento V+→GND de cada LM35 (§9 do datasheet), ADS1115 e trilho 12 V |
| **Resistor 75 Ω** | 1/4 W | 5 | 1 | **Damper R-C da Fig.13 do datasheet do LM35 — nos pinos do sensor** |
| **Capacitor 1 µF** | Cerâmico ou filme 25 V | 5 | 2–5 | **A outra metade do damper. Recomendado para fiação perto de 'motors with arcing brushes'** |
| Capacitor 100 µF | Eletrolítico 25 V | 3 | 3 | Reservatório do trilho de 12 V |
| Kit de resistores | 600 un 1/4 W sortidos (alternativa aos 3 primeiros) | 1 | 25–40 | Vale se fizer outros projetos |

### Bloco C — Montagem física e instrumentação (R$ 88–167)

| Item | Especificação | Qtd | R$ | Por quê |
|------|---------------|----:|---:|---------|
| Multímetro digital | Com escala de mV DC | 1 | 60–100 | Calibração do LM35; dá para pegar emprestado |
| Placa perfurada ilhada | 5×7 cm | 2 | 6–12 | O conjunto LM35+resistores tem que sair da protoboard (1,5 W deforma o plástico) |
| Cabo 4 vias blindado | Ou par trançado, 1 m | 1 | 10–20 | Sensor longe do PWM; cabo solto capta 60 Hz |
| Fita Kapton | 10 mm, resiste a 100 °C | 1 | 12–20 | Fita isolante comum descola no meio da coleta de 8 h |
| Abraçadeiras + caixa | Caixa de sapato ou plástica ~35 cm | 1 | 0–15 | Fixação relativa processo / cooler / sensor Ta |
| Ferro de solda + estanho | 30 W ponta fina | 1 | 0 | Emprestado |

### Bloco D — Opcionais

| Item | R$ | Vale a pena se… |
|------|---:|-----------------|
| Termômetro digital com sonda | 30–60 | …quiser calibrar offset e ganho dos LM35 contra referência (melhora o relatório) |
| ESP32-WROOM-32 DevKit reserva | 40–60 | …não quiser parar o projeto se queimar um GPIO na véspera |
| INA128P × 2 | 60–120 | …quiser seguir literalmente a nota da Figura 1 (RG = 12 kΩ cada). Em fonte única de 5 V a saída não desce abaixo de 1,4 V — exige fonte simétrica ou referência no pino REF. Rota ALTERNATIVA ao ADS1115, não complementar. |
| Fonte de bancada ajustável | 200+ | …for continuar em eletrônica |

**Totais: essencial+passivos R$ 369–547 · montagem completa R$ 457–714 · com opcionais R$ 612–994.**
Cortando multímetro (emprestado), a 2ª protoboard e as reservas: **~R$ 330**.

---

## 4. Mapa de pinos — ESP32-S3-DevKitC-1 N16R8

Escolhido para não colidir com nada e manter válidas as constantes `ADC_CHANNEL_0` e
`ADC_CHANNEL_1` que já estão no `main.c`.

| GPIO | Função | Liga em | Observação |
|-----:|--------|---------|------------|
| 8  | I2C SDA | ADS1115 SDA | Pull-up 4,7 kΩ (o módulo costuma ter) |
| 9  | I2C SCL | ADS1115 SCL | Endereço 0x48 com ADDR no GND |
| 5  | PWM aquecedor | Gate IRLZ44N #1 | LEDC ch0 · 1 kHz · 10 bits · via 220 Ω |
| 6  | PWM cooler | Gate IRLZ44N #2 | LEDC ch1 · 25 kHz (evita zumbido) |
| 1  | ADC1_CH0 (backup) | Saída filtrada LM35 Tp | Redundância ESP32 × ADS1115 |
| 2  | ADC1_CH1 (backup) | Saída filtrada LM35 Ta | Mesma leitura por dois caminhos |
| 5V | Alimentação | Vs dos LM35 + Vdd do ADS1115 | Resolve a armadilha nº 4 |
| GND| Terra comum | Devkit + fonte 12 V + sensores | Ponto de estrela único, perto da fonte |

**Pinos a EVITAR na S3 N16R8:** 26–32 (flash SPI), 33–37 (PSRAM octal — a N16R8 usa),
19 e 20 (USB nativo), 0/3/45/46 (strapping), 48 (LED RGB). Ligar carga aí trava o boot
ou corrompe a flash.

---

## 5. Ordem de montagem e teste

1. **Devkit sozinho.** Blink, confirmar que o USB-C enumera com o cabo que você tem, anotar o driver serial (CH340 ou CP2102).
2. **Um LM35 em 5 V, sem filtro.** Medir com o multímetro: à temperatura ambiente tem que dar 240–280 mV. Metade disso = sensor invertido, desligue já.
3. **ADS1115 no I2C.** Escanear o barramento (0x48), ler o LM35 e conferir contra o multímetro. Discrepância > 5 mV = problema de terra.
4. **Filtro RC.** 10 kΩ + 10 µF; medir o desvio-padrão de 100 leituras seguidas. Meta: **< 0,05 °C**. Este teste valida ou reprova toda a cadeia de medição.
5. **Aquecedor com a fonte de 12 V.** MOSFET + 3 × 150 Ω, duty fixo em 20%. Conferir 240 mA com o multímetro em série a 100% por poucos segundos, voltar para 20%, ver onde estabiliza.
6. **Fixação térmica definitiva.** Resistores colados nas faces do LM35 com Kapton, tudo na placa perfurada, sensor de Ta a ~35 cm, cooler preso. **Nada pode se mexer entre o Prj1 e o Prj2** — se mudar, os parâmetros identificados perdem validade.
7. **Cooler com diodo.** Varrer o duty de 0 a 100% e anotar em que % o motor parte. Esse valor vira o offset DC da perturbação senoidal do Prj2.
8. **Ensaio curto de PRBS (30 min)** antes da coleta de 8 h. Publicar por MQTT em paralelo com o serial: se o cabo cair às 3 da manhã, você não perde a noite.


---

## 6. Cadeia do sensor conferida contra o datasheet do LM35 (SNIS159H)

| Componente | Onde vai | O que o datasheet diz | Veredito |
|---|---|---|---|
| 100 nF cerâmico | V+ do LM35 → GND, **nos pinos do sensor** | §9: "TI recommends adding a 0.1 uF from V+ to GND" | OK, valor exato |
| **75 Ω + 1 µF** | Saída do LM35 → GND, **nos pinos do sensor** | §8.1.1 Fig.13: "75 ohm in series with 0.2 or 1 uF from output to ground", para fiação exposta a "motors with arcing brushes" | **FALTAVA — adicionado** |
| 10 kΩ | Saída do sensor → entrada do ADC | §8.1.1 Fig.12: isolar carga capacitiva com resistor série. O LM35 sozinho só aguenta 50 pF | OK, conservador (a figura usa 2 kΩ) |
| 10 µF X7R | Entrada do ADC → GND | Sem os 10 kΩ à frente isso oscilaria; com eles, é a topologia da Fig.12 | OK |

### Topologia por sensor

```
        NO CORPO DO SENSOR                  |  1 m cabo  |   NA PROTOBOARD
   5V --+---------- V+ ----+                |            |
        |                  |                |            |
      100nF            +---+---+            |            |
        |              | LM35  |            |            |
       GND             |  DZ   |            |            |
                       +---+---+            |            |
                    OUT  |  GND -- GND      |            |
                         +------------------+-- 10k -----+--> ADC / buffer
                         |                  |            |
                        75R                 |          10uF
                         |                  |            |
                        1uF                 |           GND
                         |
                        GND
                  damper (Fig.13)              filtro (Fig.12)
```

O damper fica na origem porque protege o LM35 da capacitância do cabo. O filtro fica no
destino porque a saída do LM35 tem ~0,1 Ω e desce o cabo com baixa impedância. O damper não
interfere: canto em 2,1 kHz, três décadas acima do filtro, e em DC o capacitor é aberto.

### Números que fecham o diagnóstico

- Ruído próprio do LM35, integrado na banda do filtro (2,5 Hz): **1,9 µV RMS = 0,0002 °C**. Irrelevante.
- Regulação de linha ±0,1 mV/V máx: 100 mV de ripple no 5 V viram **0,001 °C**. Irrelevante.
- Logo: os ±3 °C dos dados de maio não vieram do sensor nem da alimentação. Vieram do conversor e da captação.

### Exatidão não é resolução

O LM35D erra **±0,9 °C típico e ±2 °C máximo**, e o ganho é especificado entre **9,8 e 10,2 mV/°C (±2 %)**.
Nenhum filtro conserta isso. O requisito de 0,1 °C do roteiro é de resolução — e essa o filtro entrega.
Para a identificação o offset é absorvido pela subtração do ponto de operação, mas os **±2 % de ganho vão
direto para o Kp identificado**. É o argumento para o termômetro de referência e a calibração de dois pontos.

---

Fontes: roteiros Prj1CDig e Prj2CDig (Prof. Adolfo Bauchspiess, ENE/FT/UnB); repositório
`Projeto-de-Controle-Digital` (coletas e identificações de maio/2026). Preços verificados em
agosto/2026 na Curto Circuito (DevKitC-1 N16R8 R$ 94,30; ADS1115 R$ 20,90); demais estimados.
