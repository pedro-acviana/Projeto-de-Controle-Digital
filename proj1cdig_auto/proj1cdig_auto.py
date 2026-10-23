#!/usr/bin/env python3
"""
proj1cdig_auto.py — executa o Projeto 1 de Controle Digital de ponta a ponta.

Etapas (cada uma só começa quando a anterior passou no critério):
  calibracao → zona_morta → sensores → u0 → du → prbs_teste → prbs → projeto → ensaios → relatorio

Uso (terminal do ESP-IDF ou terminal comum, dentro desta pasta):
    python proj1cdig_auto.py                 começa (ou continua de onde parou)
    python proj1cdig_auto.py --simular       ensaio geral com bancada virtual (9 h viram ~1 min)
    python proj1cdig_auto.py --de prbs       refaz a partir de uma etapa
    python proj1cdig_auto.py --novo          descarta o progresso e começa do zero
    python proj1cdig_auto.py --reconfigurar  refaz as perguntas iniciais
    python proj1cdig_auto.py --sem-plot      sem gráfico ao vivo

O progresso fica em estado.json; os dados em dados/. Ctrl+C desliga aquecedor e cooler e
salva o progresso — rodar de novo continua da etapa interrompida.

Precisa do firmware desta pasta (proj1cdig_auto/firmware), não o da pasta proj1cdig.
Dependências: pip install pyserial matplotlib
"""
import argparse
import csv
import json
import math
import queue
import re
import shutil
import statistics
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path

AQUI = Path(__file__).resolve().parent
DADOS = AQUI / "dados"
FIRMWARE = AQUI / "firmware"
CONFIG_H = FIRMWARE / "main" / "proj1cdig_config.h"
FIELDS = ["t_s", "k", "modo", "u", "w", "tp", "ta", "ref", "aw"]

ETAPAS = ["calibracao", "zona_morta", "sensores", "u0", "du", "prbs_teste",
          "prbs", "projeto", "ensaios", "relatorio"]

# Tabela do §1 do roteiro (grupos 1–10; 11–20 repetem)
_W0 = [0.065] * 5 + [0.098] * 5
_PO = [45, 50, 55, 60, 45, 50, 55, 60, 45, 50]
_MP = [10, 10, 10, 25, 25, 30, 30, 30, 15, 15]
_TP = [25, 25, 25, 30, 25, 25, 30, 30, 25, 30]


def grupo_spec(g):
    i = (g - 1) % 10
    return dict(PO=_PO[i], Mp=_MP[i], Tp=_TP[i], w0=_W0[i])


# Critérios
SIGMA_MAX = 0.1           # °C, desvio-padrão de Tp = requisito de resolução do roteiro (ideal < 0,05)
ESTAVEL_DELTA = 0.1       # °C entre a média de agora e a de 5 min atrás
BIT_S = 60                # duração de cada nível do PRBS
U_FULL = 65535            # aquecedor: PWM de 16 bits (roteiro atualizado)
DEGRAU_DU = 1920          # degrau usado para medir o ganho estático (~3 % de duty)

# Ensaio do passo 8 (Figura 5 do roteiro) — os mesmos valores do proj1cdig_config.h
ENSAIO_AMPL_C = 10
ENSAIO_MEIO_S = 400
ENSAIO_T_CONST1_S, ENSAIO_T_TRI_S, ENSAIO_T_CONST2_S, ENSAIO_FIM_S = 1000, 1800, 2600, 3600
ISE_T0_S, ISE_T1_S = 2100, 3000
AQUEC_S = 2000            # tempo de aquecimento antes do PRBS (transit do professor)

STATUS = {"etapa": "-", "detalhe": ""}


def info(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


class Abortar(Exception):
    pass


# ============================================================================
# Perguntas
# ============================================================================
def perguntar(texto, padrao=None, tipo=str):
    while True:
        p = f" [{padrao}]" if padrao is not None else ""
        r = input(f"  {texto}{p}: ").strip()
        if tipo is not str:
            r = r.replace(",", ".")
        if not r and padrao is not None:
            return padrao
        try:
            return tipo(r)
        except ValueError:
            print("    valor inválido, tente de novo")


def sim_nao(texto, padrao=False):
    r = input(f"  {texto} [{'S/n' if padrao else 's/N'}]: ").strip().lower()
    return padrao if not r else r.startswith("s")


def decidir(msg, repetir=False):
    """Ponto de decisão: falhas sempre param e perguntam."""
    print("\n" + "!" * 70)
    print(msg)
    print("!" * 70)
    ops = "[c] continuar mesmo assim  " + ("[r] repetir a etapa  " if repetir else "") + "[a] abortar"
    while True:
        r = input(f"  {ops}: ").strip().lower()
        if r in ("c", "a") or (repetir and r == "r"):
            if r == "a":
                raise Abortar("abortado pelo usuário")
            return r


# ============================================================================
# Estado
# ============================================================================
def carregar_estado(path):
    if path.exists():
        return json.loads(path.read_text(encoding="utf-8"))
    return {"feitas": []}


def salvar_estado(st, path):
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(st, indent=2, ensure_ascii=False), encoding="utf-8")
    tmp.replace(path)


# ============================================================================
# Comunicação com a ESP32
# ============================================================================
class Bancada:
    def __init__(self, porta, log_path, simular=False):
        self.porta, self.simular = porta, simular
        self.ser = None
        self.wlock = threading.Lock()
        self.flock = threading.Lock()
        self.amostras = queue.Queue()
        self.msgs = queue.Queue()
        self.plot_buf = deque(maxlen=4000)
        self.csv = None
        self.log = open(log_path, "a", encoding="utf-8")
        self.parar = threading.Event()
        self.trip = False
        self.reiniciou = False
        self.configurado = False
        self.prbs_ok = False
        self.ensaio_ok = False
        self.ise = None
        self.ultimo_t = None
        self._abrir()
        threading.Thread(target=self._ler, daemon=True).start()

    def _abrir(self):
        if self.simular:
            from simulador import SerialSimulada
            self.ser = SerialSimulada()
            info("bancada SIMULADA conectada")
            return
        import serial
        while not self.parar.is_set():
            try:
                s = serial.Serial()
                s.port, s.baudrate, s.timeout = self.porta, 115200, 1
                s.dtr = False                 # não resetar a DevKit ao abrir
                s.rts = False
                s.open()
                # o CH340 manda um byte de lixo ao abrir a porta: um Enter sozinho descarta essa linha
                time.sleep(0.1)
                s.write(b"\n")
                time.sleep(0.2)
                self.ser = s
                info(f"conectado em {self.porta}")
                return
            except Exception as e:
                info(f"aguardando {self.porta}: {e}")
                time.sleep(2)

    def _ler(self):
        while not self.parar.is_set():
            try:
                raw = self.ser.readline()
            except Exception as e:
                info(f"conexão caiu ({e}); reconectando sem resetar a placa...")
                try:
                    self.ser.close()
                except Exception:
                    pass
                self._abrir()
                continue
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if line.startswith("D,"):
                parts = line[2:].split(",")
                # valida antes de gravar: uma linha corrompida não pode entrar no CSV
                try:
                    if len(parts) != len(FIELDS) or parts[2] not in ("IDLE", "SENS", "OPEN", "PRBS", "CTRL"):
                        raise ValueError
                    a = dict(t=float(parts[0]), k=int(parts[1]), modo=parts[2], u=float(parts[3]),
                             w=float(parts[4]), tp=float(parts[5]), ta=float(parts[6]), ref=float(parts[7]))
                    int(parts[8])
                except ValueError:
                    self.log.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} [descartada] {line}\n")
                    self.log.flush()
                    continue
                with self.flock:
                    if self.csv:
                        self.csv.write(",".join(parts) + f",{time.time():.3f}\n")
                if self.ultimo_t is not None and a["t"] < self.ultimo_t - 5:
                    self.reiniciou = True
                self.ultimo_t = a["t"]
                self.amostras.put(a)
                self.plot_buf.append(a)
            elif line:
                self.log.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} {line}\n")
                self.log.flush()
                if line.startswith("#TRIP"):
                    self.trip = True
                elif line.startswith("#proj1cdig iniciado") and self.configurado:
                    self.reiniciou = True
                elif line.startswith("#PRBS concluido"):
                    self.prbs_ok = True
                elif line.startswith("#ENSAIO concluido"):
                    m = re.search(r"=\s*([-\d.]+)", line)
                    self.ise = float(m.group(1)) if m else None
                    self.ensaio_ok = True
                if line.startswith(("#ESTADO", "#ERRO", "#?")):
                    self.msgs.put(line)
                if line.startswith(("#TRIP", "#ERRO", "#PRBS", "#ADC", "#ENSAIO", "#ISE")):
                    info(f"ESP32: {line}")

    def verificar(self):
        if self.parar.is_set():
            raise Abortar("interrompido (Ctrl+C)")
        if self.trip:
            raise Abortar("#TRIP: Tp passou de 95 °C e o firmware desligou o aquecedor")
        if self.reiniciou:
            raise Abortar("a ESP32 reiniciou (queda de energia ou USB). Rode o script de novo: "
                          "ele reenvia os parâmetros e refaz a etapa atual.")

    def enviar(self, cmd, timeout=5.0):
        while not self.msgs.empty():
            self.msgs.get_nowait()
        with self.wlock:
            self.ser.write((cmd + "\n").encode())
        self.log.write(f"{time.strftime('%Y-%m-%d %H:%M:%S')} >> {cmd}\n")
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                m = self.msgs.get(timeout=0.2)
            except queue.Empty:
                continue
            if m.startswith("#ESTADO"):
                self.limpar()            # amostras na fila são de antes do comando
                return m
            raise Abortar(f"a ESP32 recusou '{cmd}': {m}")
        raise Abortar(f"sem resposta da ESP32 para '{cmd}'. Firmware gravado? Porta certa?")

    def enviar_seguro(self, cmd):
        try:
            with self.wlock:
                self.ser.write((cmd + "\n").encode())
        except Exception:
            pass

    def arquivo(self, tag):
        """Fecha o CSV atual e abre outro (tag=None só fecha)."""
        with self.flock:
            if self.csv:
                self.csv.close()
            self.csv = None
            if tag is None:
                return None
            p = DADOS / f"proj1cdig_auto_{tag}_{time.strftime('%Y-%m-%d_%H-%M-%S')}.csv"
            self.csv = open(p, "w", encoding="utf-8")
            self.csv.write(",".join(FIELDS) + ",pc_time\n")
        info(f"gravando {p.name}")
        return p

    def limpar(self):
        while not self.amostras.empty():
            self.amostras.get_nowait()

    def proxima(self):
        avisou = False
        t0 = time.time()
        while True:
            self.verificar()
            try:
                return self.amostras.get(timeout=1)
            except queue.Empty:
                if time.time() - t0 > 30 and not avisou:
                    info("nenhuma amostra há 30 s — conferindo cabo/placa...")
                    avisou = True
                if time.time() - t0 > 300:
                    raise Abortar("nenhuma amostra da ESP32 há 5 min")

    def fechar(self):
        self.parar.set()
        self.arquivo(None)


# ============================================================================
# Esperas (no tempo da BANCADA, não do PC)
# ============================================================================
def validos(h, campo="tp"):
    return [a[campo] for a in h if not math.isnan(a[campo])]


def media(h, seg, campo="tp"):
    if not h:
        return float("nan")
    tf = h[-1]["t"]
    v = [a[campo] for a in h if a["t"] >= tf - seg and not math.isnan(a[campo])]
    return statistics.fmean(v) if v else float("nan")


def delta5(h):
    """Média dos últimos 30 s menos a média de 30 s terminando 5 min antes."""
    if not h or h[-1]["t"] - h[0]["t"] < 330:
        return None
    tf = h[-1]["t"]
    agora = [a["tp"] for a in h if a["t"] >= tf - 30 and not math.isnan(a["tp"])]
    antes = [a["tp"] for a in h if tf - 330 <= a["t"] <= tf - 300 and not math.isnan(a["tp"])]
    if not agora or not antes:
        return None
    return statistics.fmean(agora) - statistics.fmean(antes)


def esperar(b, dur, rotulo=None):
    """Consome amostras por 'dur' segundos de bancada; devolve o histórico."""
    a = b.proxima()
    t0, hist, ult = a["t"], [a], a["t"]
    while a["t"] - t0 < dur:
        a = b.proxima()
        hist.append(a)
        if rotulo and a["t"] - ult >= 600:
            ult = a["t"]
            STATUS["detalhe"] = f"{rotulo}: {(a['t'] - t0) / 60:.0f}/{dur / 60:.0f} min"
            info(f"  {STATUS['detalhe']}  Tp={a['tp']:.2f} °C  u={a['u']:.0f}")
    return hist


def esperar_estavel(b, minimo, maximo, rotulo):
    """Espera Tp estabilizar (Δ5min < 0,1 °C) após no mínimo 'minimo' s; desiste em 'maximo' s."""
    a = b.proxima()
    t0, ult = a["t"], a["t"]
    hist = deque([a])
    while True:
        a = b.proxima()
        hist.append(a)
        while hist and hist[0]["t"] < a["t"] - 400:
            hist.popleft()
        dt = a["t"] - t0
        d = delta5(hist)
        if a["t"] - ult >= 60:
            ult = a["t"]
            ds = f"{d:+.3f}" if d is not None else "…"
            STATUS["detalhe"] = f"{rotulo}: {dt / 60:.0f} min, Δ5min={ds} °C"
            info(f"  {STATUS['detalhe']}  Tp={a['tp']:.2f}  u={a['u']:.0f}")
        if dt >= minimo and d is not None and abs(d) < ESTAVEL_DELTA:
            info(f"  {rotulo}: ESTÁVEL em {media(hist, 60):.2f} °C após {dt / 60:.0f} min")
            return list(hist), True
        if dt >= maximo:
            info(f"  {rotulo}: não estabilizou em {maximo / 60:.0f} min")
            return list(hist), False


def sigma_sem_tendencia(v):
    n = len(v)
    if n < 3:
        return float("nan")
    xm, ym = (n - 1) / 2, statistics.fmean(v)
    sxx = sum((i - xm) ** 2 for i in range(n))
    b = sum((i - xm) * (y - ym) for i, y in enumerate(v)) / sxx
    res = [y - (ym + b * (i - xm)) for i, y in enumerate(v)]
    return statistics.pstdev(res)


# ============================================================================
# Configuração inicial
# ============================================================================
def configurar(st, args):
    print("\n=== proj1cdig_auto — configuração inicial ===\n")
    if not args.simular:
        st["porta"] = perguntar("Porta serial da ESP32 (Gerenciador de Dispositivos → Portas COM)", st.get("porta", "COM5"))
    st["grupo"] = perguntar("Número do grupo", st.get("grupo", 2), int)
    g = grupo_spec(st["grupo"])
    print(f"    → P.O. {g['PO']} °C, Mp {g['Mp']} %, Tp {g['Tp']} s, ω0 {g['w0']} rad/s")

    print("\n  Saturação do aquecedor (contagens de PWM, 65535 = 100 %).")
    print("  PWM do aquecedor em 16 bits (0..65535). Com 3 × 150 Ω de 1/4 W em 12 V, o máximo seguro")
    print("  é 16000 (24 %). Com 1/2 W, 32000; com 1 W, 65535.")
    st["umax"] = perguntar("U_MAX", st.get("umax", 16000), int)
    if st["umax"] > 16000:
        print("    ATENÇÃO: acima de 16000 o firmware recusa, a menos que U_MAX_HARD no config.h")
        print("    tenha sido aumentado e o firmware regravado.")

    print("\n  Calibração dos LM35  (T_real = ganho · T_lido + offset)")
    print("    1 = digitar ganho e offset que já tenho")
    print("    2 = assistida: o script mede e eu digito o termômetro de referência")
    print("    3 = pular (ganho 1, offset 0)")
    modo = perguntar("Opção", st.get("cal_modo", 2), int)
    st["cal_modo"] = modo
    cal = st.get("cal", [1.0, 0.0, 1.0, 0.0])
    if modo == 1:
        cal = [perguntar("Ganho Tp", cal[0], float), perguntar("Offset Tp [°C]", cal[1], float),
               perguntar("Ganho Ta", cal[2], float), perguntar("Offset Ta [°C]", cal[3], float)]
    elif modo == 3:
        cal = [1.0, 0.0, 1.0, 0.0]
    st["cal"] = cal

    print("\n  Zona morta do cooler (menor PWM em que o motor PARTE)")
    print("    1 = digitar o valor   2 = assistida (o script sobe o PWM e pergunta)   3 = não sei (usar 300)")
    zm = perguntar("Opção", st.get("dz_modo", 2), int)
    st["dz_modo"] = zm
    if zm == 1:
        st["dz"] = perguntar("Zona morta [contagens]", st.get("dz", 300), int)
        definir_cooler(st)
    elif zm == 3:
        st["dz"] = 300
        definir_cooler(st)

    st["u_chute"] = perguntar("Chute inicial de PWM para o P.O. (16 bits)", st.get("u_chute", 8320), int)

    tem_matlab = shutil.which("matlab") is not None
    if tem_matlab:
        st["matlab"] = sim_nao("MATLAB encontrado. Rodar a identificação/projeto automaticamente?", True)
    else:
        print("\n  MATLAB não está no PATH: na etapa 'projeto' o script vai mostrar o comando")
        print("  para você rodar no MATLAB e continua sozinho quando o resultado aparecer.")
        st["matlab"] = False
    st["pausar_prbs"] = sim_nao("Parar e pedir confirmação antes da coleta de 9 h?", st.get("pausar_prbs", False))

    # etapas assistidas que não serão feitas
    feitas = set(st.get("feitas", []))
    if modo != 2:
        feitas.add("calibracao")
    else:
        feitas.discard("calibracao")
    if zm != 2:
        feitas.add("zona_morta")
    else:
        feitas.discard("zona_morta")
    st["feitas"] = [e for e in ETAPAS if e in feitas]
    st["configurado"] = True


def definir_cooler(st):
    """Senoide do cooler sempre acima da zona morta (margem de 50)."""
    baixo = st["dz"] + 50
    amp = max(20, min(256, (1023 - baixo) // 2))
    st["woff"], st["wamp"] = baixo + amp, amp
    info(f"cooler: zona morta {st['dz']} → senoide {st['woff']} ± {st['wamp']}")


def gravar_firmware(st, args):
    if args.simular:
        return
    if sim_nao("O firmware da pasta proj1cdig_auto/firmware já está gravado na placa?", st.get("gravado", False)):
        st["gravado"] = True
        return
    if shutil.which("idf.py"):
        info("gravando o firmware (idf.py)...")
        if not (FIRMWARE / "sdkconfig").exists():
            subprocess.run("idf.py set-target esp32", cwd=FIRMWARE, shell=True, check=True)
        r = subprocess.run(f"idf.py -p {st['porta']} flash", cwd=FIRMWARE, shell=True)
        if r.returncode != 0:
            raise Abortar("falha ao gravar. Confira a porta e se nenhum monitor serial está aberto.")
    else:
        print("\n  idf.py não encontrado (rode o script no terminal do ESP-IDF para gravar sozinho).")
        print(f"  Grave manualmente:  cd \"{FIRMWARE}\"  e  idf.py -p {st['porta']} flash")
        input("  Enter quando terminar... ")
    st["gravado"] = True


def aplicar_parametros(b, st):
    e = b.enviar("x")
    if "umax=" not in e:
        raise Abortar("a placa está com o firmware da pasta proj1cdig (manual). "
                      "Grave o de proj1cdig_auto/firmware.")
    g = grupo_spec(st["grupo"])
    b.enviar(f"U {st['umax']}")
    b.enviar("C " + " ".join(f"{c:.6f}" for c in st["cal"]))
    if "woff" in st:
        b.enviar(f"W {st['woff']} {st['wamp']}")
    b.enviar(f"F {g['w0']}")
    p = st.get("projeto")
    if p:
        b.enviar(f"T {round(p['Ts'] * 1000)}")
        b.enviar(f"k {p['K']:.5f} {p['z0']:.6f}")
    b.configurado = True
    b.limpar()


# ============================================================================
# Etapas
# ============================================================================
def etapa_calibracao(b, st):
    print("\n  Calibração assistida. Prenda o termômetro de referência junto ao LM35 do PROCESSO.")
    print("  (Ta só recebe offset: sem aquecedor perto dele, não há segundo ponto.)")
    input("  Enter quando estiver pronto... ")
    b.enviar("C 1 0 1 0")
    b.enviar("s")
    info("ponto 1: ambiente, aquecedor desligado")
    h, _ = esperar_estavel(b, 300, 1800, "ponto 1")
    tp1, ta1 = media(h, 60), media(h, 60, "ta")
    print(f"    lido: Tp = {tp1:.2f} °C, Ta = {ta1:.2f} °C")
    r_tp1 = perguntar("Termômetro de referência no sensor do PROCESSO agora [°C]", tipo=float)
    r_ta1 = perguntar("Termômetro de referência no sensor do AMBIENTE agora [°C]", tipo=float)
    u = min(9600, st["umax"])
    info(f"ponto 2: aquecendo com o {u}")
    b.enviar(f"o {u}")
    h, _ = esperar_estavel(b, 900, 3600, "ponto 2")
    tp2 = media(h, 60)
    print(f"    lido: Tp = {tp2:.2f} °C")
    r_tp2 = perguntar("Termômetro de referência no sensor do PROCESSO agora [°C]", tipo=float)
    b.enviar("s")
    if abs(tp2 - tp1) < 5:
        decidir("Os dois pontos ficaram a menos de 5 °C: o ganho seria impreciso. Fica só o offset.")
        gp = 1.0
    else:
        gp = (r_tp2 - r_tp1) / (tp2 - tp1)
    op = r_tp1 - gp * tp1
    oa = r_ta1 - ta1
    if not 0.9 < gp < 1.1:
        decidir(f"Ganho calculado {gp:.4f} está fora de 0,9–1,1 (o LM35 erra no máximo ±2 %). "
                "Leitura errada do termômetro?", repetir=False)
    st["cal"] = [gp, op, 1.0, oa]
    b.enviar("C " + " ".join(f"{c:.6f}" for c in st["cal"]))
    info(f"calibração: Tp = {gp:.5f}·T {op:+.3f}   Ta = T {oa:+.3f}")


def etapa_zona_morta(b, st):
    print("\n  Zona morta do cooler: o script sobe o PWM de 50 em 50. Olhe/escute o cooler.")
    b.enviar("s")
    dz = None
    for v in range(100, 1024, 50):
        b.enviar(f"wc {v}")
        time.sleep(4)
        if sim_nao(f"PWM {v}: o cooler está girando?"):
            dz = v
            break
    b.enviar("w0")
    if dz is None:
        raise Abortar("o cooler não girou nem a 100 %: confira fonte de 12 V, MOSFET e diodo")
    st["dz"] = dz
    definir_cooler(st)
    b.enviar(f"W {st['woff']} {st['wamp']}")


def etapa_sensores(b, st):
    b.enviar("w0")
    b.enviar("s")
    info("aquecedor desligado: esperando o processo parar de esfriar antes de medir o ruído")
    esperar_estavel(b, 120, 3600, "repouso")
    info("medindo ruído com tudo desligado (100 s)")
    h = esperar(b, 100)
    s0 = sigma_sem_tendencia(validos(h))
    info(f"  σ(Tp) = {s0:.4f} °C   (Ta média {media(h, 100, 'ta'):.2f} °C)")
    wmax = st.get("woff", 460) + st.get("wamp", 256)
    b.enviar(f"wc {wmax}")
    info(f"medindo ruído com o cooler em {wmax} (100 s)")
    esperar(b, 60)
    h = esperar(b, 100)
    s1 = sigma_sem_tendencia(validos(h))
    b.enviar("w0")
    info(f"  σ(Tp) com cooler = {s1:.4f} °C")
    st["sigma"] = [s0, s1]
    nan = sum(1 for a in h if math.isnan(a["tp"]))
    if nan:
        decidir(f"{nan} leituras inválidas (nan): o ADC não está lendo. Confira os pinos D35/D34 no config.h.")
    if s0 >= SIGMA_MAX or s1 >= SIGMA_MAX:
        r = decidir(f"Ruído acima do critério: σ = {s0:.4f} / {s1:.4f} °C (limite {SIGMA_MAX}).\n"
                    "Com esse ruído a identificação tende a sair ruim (Td≈0, Tz<0).\n"
                    "Revise fios trançados até o sensor, terra comum e curto, e o capacitor do\n"
                    "damper (75 Ω + 1 µF). Se não bastar, um RC de verdade no pino do ADC ajuda\n"
                    "(ex.: 1 kΩ em série + 10 µF cerâmico ao GND).", repetir=True)
        if r == "r":
            return etapa_sensores(b, st)


def etapa_u0(b, st):
    PO = grupo_spec(st["grupo"])["PO"]
    u, pts = st.get("u_chute", 8320), []
    for it in range(1, 8):
        u = max(320, min(u, st["umax"]))
        info(f"u0 — tentativa {it}: o {u}")
        b.enviar(f"o {u}")
        h, ok = esperar_estavel(b, 600, 3600, f"o {u}")
        T, Ta = media(h, 60), media(h, 60, "ta")
        pts.append((u, T))
        info(f"  u={u} → Tp={T:.2f} °C (Ta={Ta:.2f})")
        if abs(T - PO) <= 0.5 and ok:
            st.update(u0=u, T0=T, Ta0=Ta)
            info(f"u0 = {u}")
            return
        if len(pts) >= 2 and abs(pts[-1][1] - pts[-2][1]) > 0.3:
            (ua, Tb_), (ub, Tc) = pts[-2], pts[-1]
            novo = ub + (PO - Tc) * (ub - ua) / (Tc - Tb_)          # secante
        elif T - Ta > 1:
            novo = u * (PO - Ta) / (T - Ta)                          # proporcional a partir do ambiente
        else:
            novo = u * 1.5
        novo = int(round(novo))
        if novo == u:
            novo += 64 if T < PO else -64
        if u >= st["umax"] and T < PO:
            raise Abortar(f"com U_MAX={st['umax']} o processo só chega a {T:.1f} °C: falta potência "
                          "(resistores, fonte de 12 V) ou o P.O. é alto demais para a montagem")
        u = novo
    decidir("u0 não convergiu em 7 tentativas. Pode haver deriva forte do ambiente.")
    st.update(u0=u, T0=pts[-1][1], Ta0=Ta)


def etapa_du(b, st):
    u0, T0 = st["u0"], st["T0"]
    u1 = min(u0 + DEGRAU_DU, st["umax"])
    info(f"degrau de {u0} para {u1}")
    b.enviar(f"o {u1}")
    h, ok = esperar_estavel(b, 600, 3600, f"o {u1}")
    T1 = media(h, 60)
    ganho = (T1 - T0) / (u1 - u0)
    if ganho * (u1 - u0) < 0.4:              # menos de 0,4 °C no degrau
        raise Abortar(f"ganho estático {ganho:.6f} °C/contagem: o degrau não aqueceu. Aquecedor ligado?")
    du = round(5 / ganho)
    lim = min(st["umax"] - u0, u0)
    if du > lim:
        decidir(f"du = {du} passaria dos limites (0..{st['umax']}); vou usar {lim}. "
                "A excursão do PRBS vai ficar menor que ±5 °C.")
        du = lim
    st.update(ganho=ganho, du=du)
    info(f"ganho estático = {ganho:.6f} °C/contagem → du = {du}")
    b.enviar(f"o {u0}")


def etapa_prbs_teste(b, st):
    PO = grupo_spec(st["grupo"])["PO"]
    for tent in range(1, 4):
        u0, du = st["u0"], st["du"]
        b.arquivo(f"prbs_teste{tent}")
        info(f"PRBS de teste {tent}: p {u0} {du} {BIT_S} 600 (35 min)")
        b.enviar(f"p {u0} {du} {BIT_S} 600")
        h = esperar(b, 600 + 1800, "PRBS de teste")
        b.arquivo(None)
        v = sorted(a["tp"] for a in h if a["k"] >= 900 and not math.isnan(a["tp"]))
        p5, p95 = v[int(0.05 * len(v))], v[int(0.95 * len(v)) - 1]
        amp, med = (p95 - p5) / 2, statistics.fmean(v)
        info(f"  excursão ±{amp:.2f} °C em torno de {med:.2f} °C")
        ajustou = False
        if abs(med - PO) > 2:
            st["u0"] = int(round(u0 + (PO - med) / st["ganho"]))
            info(f"  média longe do P.O.: u0 → {st['u0']}")
            ajustou = True
        if amp < 2:
            st["du"] = min(int(round(du * min(4 / max(amp, 0.5), 2))), st["umax"] - st["u0"], st["u0"])
            info(f"  excursão pequena: du → {st['du']}")
            ajustou = True
        elif amp > 6.5:
            st["du"] = int(round(du * 5 / amp))
            info(f"  excursão grande: du → {st['du']}")
            ajustou = True
        if not ajustou:
            st["prbs_amp"] = amp
            return
    decidir("Depois de 3 tentativas a excursão ainda não ficou em ±2–6,5 °C em torno do P.O.")


def etapa_prbs(b, st):
    u0, du = st["u0"], st["du"]
    total = AQUEC_S + 511 * BIT_S
    if st.get("pausar_prbs"):
        print(f"\n  Pronto para a coleta: p {u0} {du} {BIT_S} {AQUEC_S} ({total / 3600:.1f} h).")
        print("  Notebook na tomada, sem suspensão, longe de sol e ar-condicionado.")
        input("  Enter para começar... ")
    p = b.arquivo("prbs")
    b.prbs_ok = False
    b.enviar(f"p {u0} {du} {BIT_S} {AQUEC_S}")
    a, t0, ult, n, nan, gaps, tant = b.proxima(), None, 0, 0, 0, 0, None
    t0 = a["t"]
    while not (b.prbs_ok or a["k"] >= total):
        a = b.proxima()
        n += 1
        nan += math.isnan(a["tp"])
        if tant is not None and a["t"] - tant > 3:
            gaps += 1
        tant = a["t"]
        if a["t"] - ult >= 600:
            ult = a["t"]
            fr = a["k"] / total
            STATUS["detalhe"] = f"coleta PRBS {100 * fr:.0f} %"
            info(f"  {STATUS['detalhe']} — faltam {(total - a['k']) / 3600:.1f} h   Tp={a['tp']:.2f}  u={a['u']:.0f}")
    b.arquivo(None)
    b.enviar(f"o {u0}")                       # mantém o processo quente no P.O.
    st["arq_prbs"] = str(p.relative_to(AQUI)).replace("\\", "/")
    info(f"coleta concluída: {n} amostras, {nan} nan, {gaps} buracos → {p.name}")
    if nan > 0.01 * n or gaps > 0.01 * n:
        decidir(f"A coleta tem {nan} leituras inválidas e {gaps} buracos de tempo (>1 %).", repetir=False)


def etapa_projeto(b, st):
    arq = st["arq_prbs"]
    js = DADOS / "projeto.json"
    if js.exists():
        js.rename(js.with_name(f"projeto_antigo_{time.strftime('%H%M%S')}.json"))
    cmd = (f"cd('{AQUI.as_posix()}'); proj1cdig_auto_projeto('{arq}', {st['grupo']}, "
           f"{st['umax']}, 'dados/projeto.json')")
    if st.get("matlab"):
        info("rodando o MATLAB em modo batch (pode levar alguns minutos)...")
        r = subprocess.run(["matlab", "-batch", cmd], cwd=AQUI)
        if r.returncode != 0:
            decidir("O MATLAB terminou com erro (veja a saída acima).", repetir=False)
    else:
        print("\n  Rode no MATLAB (Command Window):\n")
        print(f"    {cmd}\n")
        print("  O script continua sozinho quando dados/projeto.json aparecer.")
    while not js.exists():
        b.limpar()
        b.verificar()
        time.sleep(2)
    time.sleep(1)
    p = json.loads(js.read_text(encoding="utf-8"))
    st["projeto"] = p
    info(f"modelo: Kp={p['Kp']:.6f} Tz={p['Tz']:.1f} Tp1={p['Tp1']:.1f} Tp2={p['Tp2']:.1f} "
         f"Td={p['Td']:.2f}  fit={p['fit']:.1f} %")
    info(f"PI: Ts={p['Ts']:.2f} s  K={p['K']:.3f}  z0={p['z0']:.5f}  → simulado Mp={p['Mp_sim']:.1f} % "
         f"tp={p['tp_sim']:.1f} s")
    probl = []
    if not 3 <= p["Td"] <= 9:
        probl.append(f"Td = {p['Td']:.2f} s (esperado 4–8 s)")
    if p["Tz"] <= 0:
        probl.append(f"Tz = {p['Tz']:.1f} (esperado > 0)")
    if p["fit"] < 85:
        probl.append(f"fit = {p['fit']:.1f} % (esperado > 85 %)")
    if probl:
        decidir("Identificação fora do esperado: " + "; ".join(probl) +
                "\nCostuma ser ruído ou montagem que mexeu durante a coleta.", repetir=False)
    b.enviar(f"T {round(p['Ts'] * 1000)}")
    b.enviar(f"k {p['K']:.5f} {p['z0']:.6f}")


def esperar_ensaio(b, limite, rotulo):
    """Consome amostras até o firmware avisar '#ENSAIO concluido' (ou estourar o limite)."""
    a = b.proxima()
    t0, ult = a["t"], a["t"]
    while not b.ensaio_ok and a["t"] - t0 < limite:
        a = b.proxima()
        if a["t"] - ult >= 300:
            ult = a["t"]
            STATUS["detalhe"] = f"{rotulo}: {(a['t'] - t0) / 60:.0f}/{ENSAIO_FIM_S / 60:.0f} min"
            info(f"  {STATUS['detalhe']}  ref={a['ref']:.1f}  Tp={a['tp']:.2f}  u={a['u']:.0f}  cooler={a['w']:.0f}")
    if not b.ensaio_ok:
        raise Abortar("o firmware não avisou o fim do ensaio. Firmware desatualizado (sem o comando e)?")


def etapa_ensaios(b, st):
    """Passo 8 do roteiro novo: onda quadrada de referência + perturbação constante e triangular
    (Figura 5), uma vez com anti-windup e outra sem, cada uma no seu CSV."""
    PO = grupo_spec(st["grupo"])["PO"]
    feitos = st.setdefault("ensaios", {})
    ise_fw = st.setdefault("ise_firmware", {})

    # ponto de partida: malha fechada estável no P.O. (também ao retomar)
    b.enviar("w0")
    b.enviar("a 1")
    b.enviar(f"c {PO}")
    esperar_estavel(b, 900, 3600, f"malha fechada em {PO} °C")
    for tag, aw in (("ensaio_aw1", 1), ("ensaio_aw0", 0)):
        if tag in feitos:
            info(f"{tag} já feito, pulando")
            continue
        b.enviar(f"a {aw}")
        p = b.arquivo(tag)
        b.ensaio_ok, b.ise = False, None
        info(f"{tag}: onda quadrada {PO}/{PO + ENSAIO_AMPL_C} °C, perturbação constante e triangular "
             f"({ENSAIO_FIM_S / 60:.0f} min, anti-windup {'ligado' if aw else 'desligado'})")
        b.enviar(f"e {PO} {PO + ENSAIO_AMPL_C} {ENSAIO_MEIO_S}")
        esperar_ensaio(b, ENSAIO_FIM_S + 120, tag)
        b.arquivo(None)
        feitos[tag] = str(p.relative_to(AQUI)).replace("\\", "/")
        ise_fw[tag] = b.ise
        salvar_estado(st, AQUI / ("estado_simulado.json" if st.get("simulado") else "estado.json"))
        b.enviar("a 1")
        esperar_estavel(b, 600, 3600, f"volta a {PO} °C")
    b.enviar("x")


# ============================================================================
# Relatório
# ============================================================================
def ler_csv(rel):
    rows = []
    with open(AQUI / rel, encoding="utf-8") as f:
        for r in csv.DictReader(f):
            try:
                rows.append(dict(t=float(r["t_s"]), modo=r["modo"], u=float(r["u"]), w=float(r["w"]),
                                 tp=float(r["tp"]), ta=float(r["ta"]), ref=float(r["ref"]), aw=int(r["aw"])))
            except (ValueError, KeyError):
                pass
    return rows


def analisar_degrau(c, umax):
    """Primeiro degrau de referência: Mp, tp (até o próximo degrau ou 900 s)."""
    i = next((j for j in range(1, len(c)) if abs(c[j]["ref"] - c[j - 1]["ref"]) > 0.5), None)
    if i is None:
        return None
    r0, r1, t0 = c[i - 1]["ref"], c[i]["ref"], c[i]["t"]
    jan = []
    for r in c[i:]:
        if r["ref"] != r1 or r["t"] > t0 + 900:
            break
        jan.append(r)
    s = 1 if r1 > r0 else -1
    pk = max(jan, key=lambda r: s * r["tp"])
    return dict(r0=r0, r1=r1, Mp=(pk["tp"] - r1) / (r1 - r0) * 100, tp=pk["t"] - t0)


def analisar_ensaio(rows, umax):
    c = [r for r in rows if r["modo"] == "CTRL" and not math.isnan(r["ref"])]
    if len(c) < 10:
        return None
    t0 = c[0]["t"]
    rel = lambda a, b_: [r for r in c if a <= r["t"] - t0 < b_]
    dt = statistics.median(c[j]["t"] - c[j - 1]["t"] for j in range(1, len(c)))
    res = analisar_degrau(c, umax) or {}
    jise = rel(ISE_T0_S, ISE_T1_S)
    res["ise"] = sum((r["ref"] - r["tp"]) ** 2 for r in jise) * dt if c[-1]["t"] - t0 >= ISE_T1_S else float("nan")
    jc = rel(ENSAIO_T_CONST1_S, ENSAIO_T_CONST1_S + 200)          # ref constante no nível baixo
    if jc:
        res["queda"] = max(r["ref"] - r["tp"] for r in jc)
        res["erro_const"] = statistics.fmean(r["ref"] - r["tp"] for r in jc[-10:])
    jt = rel(2100, 2400)                                             # triangular, ref alta, sem transitório
    if jt:
        e = [r["ref"] - r["tp"] for r in jt]
        res["amp_tri"] = (max(e) - min(e)) / 2
    res["t_sat"] = sum(1 for r in c if r["u"] >= umax - 1) * dt
    return res


def grafico(rows, titulo, png, umax):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t0 = rows[0]["t"]
    t = [(r["t"] - t0) for r in rows]
    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(10, 7))
    ax[0].plot(t, [r["tp"] for r in rows], "r-", label="Tp")
    ax[0].plot(t, [r["ref"] for r in rows], "k--", label="ref")
    ax[0].set_ylabel("°C"); ax[0].legend(); ax[0].grid(alpha=.3); ax[0].set_title(titulo)
    ax[1].step(t, [r["u"] for r in rows], "g-", where="post", label="u")
    ax[1].axhline(umax, color="r", ls="--", lw=1, label="saturação")
    ax[1].set_ylabel("contagens"); ax[1].legend(); ax[1].grid(alpha=.3)
    ax[2].plot(t, [r["w"] for r in rows], "c-", label="cooler")
    ax[2].set_ylabel("contagens"); ax[2].set_xlabel("t [s]"); ax[2].legend(); ax[2].grid(alpha=.3)
    fig.tight_layout(); fig.savefig(png, dpi=110); plt.close(fig)


def etapa_relatorio(b, st):
    g = grupo_spec(st["grupo"])
    p = st["projeto"]
    L = [f"# Relatório automático — Prj1CDig, grupo {st['grupo']}", "",
         f"Gerado em {time.strftime('%d/%m/%Y %H:%M')} por proj1cdig_auto.py", "",
         "## Especificação", "",
         f"P.O. {g['PO']} °C · Mp {g['Mp']} % · Tp {g['Tp']} s · ω0 {g['w0']} rad/s · Sat = U_MAX = {st['umax']}", "",
         "## Bancada", "",
         f"- Calibração: Tp = {st['cal'][0]:.5f}·T {st['cal'][1]:+.3f} · Ta = {st['cal'][2]:.5f}·T {st['cal'][3]:+.3f}",
         "- Ruído σ(Tp): " + (f"{st['sigma'][0]:.4f} °C sem cooler, {st['sigma'][1]:.4f} °C com cooler"
                              if "sigma" in st else "não medido"),
         f"- Zona morta do cooler: {st['dz']} → perturbação {st['woff']} ± {st['wamp']}",
         f"- u0 = {st['u0']} (Tp = {st['T0']:.2f} °C, Ta = {st['Ta0']:.2f} °C) · ganho estático "
         f"{st['ganho']:.6f} °C/cont · du = {st['du']}", "",
         "## Modelo identificado (P2DZ)", "",
         "| Kp [°C/cont] | Tz [s] | Tp1 [s] | Tp2 [s] | Td [s] | fit [%] |", "|---|---|---|---|---|---|",
         f"| {p['Kp']:.6f} | {p['Tz']:.1f} | {p['Tp1']:.1f} | {p['Tp2']:.1f} | {p['Td']:.2f} | {p['fit']:.1f} |", "",
         "## Controlador", "",
         f"- Polo desejado: ζ = {p['zeta']:.4f}, ωn = {p['wn']:.4f} rad/s",
         f"- PI contínuo (ponto de partida LGR): KP = {p['KP_cont']:.3f}, KI = {p['KI_cont']:.4f}",
         f"- PI discreto: C(z) = {p['K']:.3f}·(z − {p['z0']:.5f})/(z − 1), Ts = {p['Ts']:.2f} s",
         f"- Simulação linear: Mp = {p['Mp_sim']:.1f} %, tp = {p['tp_sim']:.1f} s",
         f"- Simulação degrau +5 °C com saturação: pico {p['pico_aw1']:.2f} °C com AW, {p['pico_aw0']:.2f} °C sem AW", "",
         "## Ensaios", ""]
    ens = st.get("ensaios", {})
    res = {}
    for tag, rel_ in ens.items():
        rows = ler_csv(rel_)
        if not rows:
            continue
        grafico(rows, tag, DADOS / f"relatorio_{tag}.png", st["umax"])
        res[tag] = analisar_ensaio(rows, st["umax"])
    f = lambda v, fmt: (fmt.format(v) if v is not None and not (isinstance(v, float) and math.isnan(v)) else "—")
    L += [f"Ensaio do passo 8 (Figura 5 do roteiro): referência em onda quadrada {g['PO']}/{g['PO'] + ENSAIO_AMPL_C} °C "
          f"(meio período {ENSAIO_MEIO_S} s), cooler constante a partir de {ENSAIO_T_CONST1_S} s, triangular de "
          f"{ENSAIO_T_TRI_S} a {ENSAIO_T_CONST2_S} s, constante até {ENSAIO_FIM_S} s.", "",
          "| | com anti-windup | sem anti-windup | especificação / roteiro |", "|---|---|---|---|"]
    a1, a0 = res.get("ensaio_aw1") or {}, res.get("ensaio_aw0") or {}
    fw = st.get("ise_firmware", {})
    linhas = [("Mp no 1º degrau", "Mp", "{:.1f} %", f"{g['Mp']} %"),
              ("tp no 1º degrau", "tp", "{:.1f} s", f"{g['Tp']} s"),
              (f"ISE {ISE_T0_S}–{ISE_T1_S} s", "ise", "{:.0f}", "típico 2000–4000 (Simulink)"),
              ("queda com perturbação constante", "queda", "{:.2f} °C", "—"),
              ("erro final com perturbação constante", "erro_const", "{:+.3f} °C", "→ 0 (canal I)"),
              ("erro com perturbação triangular", "amp_tri", "± {:.2f} °C", "atenuada, não rejeitada"),
              ("tempo com aquecedor saturado", "t_sat", "{:.0f} s", "—")]
    for nome, chave, fmt, esp in linhas:
        L.append(f"| {nome} | {f(a1.get(chave), fmt)} | {f(a0.get(chave), fmt)} | {esp} |")
    L.append(f"| ISE calculado pelo firmware | {f(fw.get('ensaio_aw1'), '{:.0f}')} | "
             f"{f(fw.get('ensaio_aw0'), '{:.0f}')} | — |")
    L += [""] + [f"![{t}](relatorio_{t}.png)" for t in ens] + ["",
          "## Arquivos", ""] + [f"- {t}: `{rel}`" for t, rel in ens.items()] + \
         [f"- PRBS: `{st['arq_prbs']}`", "- Projeto: `dados/projeto.json`", "",
          "Para o vídeo e para o passo 7 do roteiro, rode também o `proj1cdig_m.m` da pasta proj1cdig "
          "(sisotool e Simulink) com estes mesmos CSVs."]
    rel = DADOS / "relatorio_auto.md"
    rel.write_text("\n".join(L), encoding="utf-8")
    atualizar_config_h(st)
    info(f"relatório: {rel}")
    print("\n  Linhas para o proj1cdig_config.h (já aplicadas na cópia desta pasta):\n")
    for n, v in valores_config(st).items():
        print(f"    #define {n:<18} {v}")


def valores_config(st):
    g = grupo_spec(st["grupo"])
    v = {"U_MAX": str(st["umax"]),
         "CAL_TP_GAIN": f"{st['cal'][0]:.5f}f", "CAL_TP_OFFSET": f"{st['cal'][1]:.4f}f",
         "CAL_TA_GAIN": f"{st['cal'][2]:.5f}f", "CAL_TA_OFFSET": f"{st['cal'][3]:.4f}f",
         "GROUP_PO_C": f"{g['PO']:.1f}f", "GROUP_W0_RAD_S": f"{g['w0']:.3f}f"}
    if "woff" in st:
        v.update(COOLER_OFFSET=str(st["woff"]), COOLER_AMP=str(st["wamp"]))
    if "u0" in st:
        v.update(PRBS_DEFAULT_U0=str(st["u0"]), PRBS_DEFAULT_DU=str(st.get("du", 40)))
    if "projeto" in st:
        p = st["projeto"]
        v.update(CTRL_TS_MS=str(round(p["Ts"] * 1000)), PI_K=f"{p['K']:.4f}f", PI_Z0=f"{p['z0']:.5f}f")
    return v


def atualizar_config_h(st):
    s = CONFIG_H.read_text(encoding="utf-8")
    for nome, val in valores_config(st).items():
        s = re.sub(rf"(#define\s+{nome}\s+)(\S+)", lambda m: m.group(1) + val, s, count=1)
    CONFIG_H.write_text(s, encoding="utf-8")


FUNCOES = dict(calibracao=etapa_calibracao, zona_morta=etapa_zona_morta, sensores=etapa_sensores,
               u0=etapa_u0, du=etapa_du, prbs_teste=etapa_prbs_teste, prbs=etapa_prbs,
               projeto=etapa_projeto, ensaios=etapa_ensaios, relatorio=etapa_relatorio)


# ============================================================================
# Execução
# ============================================================================
def executar(b, st, est_path, resultado):
    try:
        aplicar_parametros(b, st)
        for etapa in ETAPAS:
            if etapa in st["feitas"]:
                continue
            STATUS["etapa"], STATUS["detalhe"] = etapa, ""
            print("\n" + "=" * 70 + f"\n  ETAPA: {etapa}\n" + "=" * 70)
            FUNCOES[etapa](b, st)
            st["feitas"].append(etapa)
            salvar_estado(st, est_path)
            info(f"etapa {etapa} concluída")
        STATUS["etapa"] = "FIM"
        info("Projeto 1 concluído. Veja dados/relatorio_auto.md")
        resultado["ok"] = True
    except Abortar as e:
        info(f"PARADO: {e}")
        b.enviar_seguro("x")
        resultado["erro"] = str(e)
    except Exception:
        b.enviar_seguro("x")
        raise
    finally:
        b.arquivo(None)
        salvar_estado(st, est_path)


def plot_loop(b, worker):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    fig, (a1, a2) = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    l_tp, = a1.plot([], [], "r-", label="Tp")
    l_ref, = a1.plot([], [], "k--", label="ref")
    l_ta, = a1.plot([], [], "b-", alpha=.5, label="Ta")
    a1.set_ylabel("°C"); a1.legend(loc="upper left"); a1.grid(alpha=.3)
    l_u, = a2.plot([], [], "g-", label="u")
    l_w, = a2.plot([], [], "c-", alpha=.7, label="cooler")
    a2.set_ylabel("contagens"); a2.set_xlabel("t [s]"); a2.legend(loc="upper left"); a2.grid(alpha=.3)

    def upd(_):
        if not worker.is_alive():
            plt.close(fig)
            return
        d = list(b.plot_buf)
        if len(d) < 2:
            return
        t = [x["t"] for x in d]
        l_tp.set_data(t, [x["tp"] for x in d]); l_ref.set_data(t, [x["ref"] for x in d])
        l_ta.set_data(t, [x["ta"] for x in d]); l_u.set_data(t, [x["u"] for x in d])
        l_w.set_data(t, [x["w"] for x in d])
        for ax in (a1, a2):
            ax.relim(); ax.autoscale_view()
        a1.set_title(f"etapa: {STATUS['etapa']}   {STATUS['detalhe']}   Tp={d[-1]['tp']:.2f} °C")

    _ani = FuncAnimation(fig, upd, interval=1000, cache_frame_data=False)
    plt.tight_layout()
    plt.show()


def main():
    ap = argparse.ArgumentParser(description="Projeto 1 de Controle Digital automatizado")
    ap.add_argument("--simular", action="store_true", help="bancada virtual, sem hardware")
    ap.add_argument("--de", choices=ETAPAS, help="refaz a partir desta etapa")
    ap.add_argument("--novo", action="store_true", help="descarta o progresso")
    ap.add_argument("--reconfigurar", action="store_true", help="refaz as perguntas iniciais")
    ap.add_argument("--sem-plot", action="store_true", help="sem gráfico ao vivo")
    a = ap.parse_args()

    DADOS.mkdir(exist_ok=True)
    est_path = AQUI / ("estado_simulado.json" if a.simular else "estado.json")
    if a.novo and est_path.exists():
        est_path.rename(est_path.with_name(f"{est_path.stem}_{time.strftime('%Y%m%d_%H%M%S')}.json"))
    st = carregar_estado(est_path)
    if a.simular:
        st["simulado"] = True
    if not st.get("configurado") or a.reconfigurar:
        configurar(st, a)
        salvar_estado(st, est_path)
    if a.de:
        i = ETAPAS.index(a.de)
        st["feitas"] = [e for e in st["feitas"] if ETAPAS.index(e) < i]
        if i <= ETAPAS.index("ensaios"):
            st.pop("ensaios", None)
    if st["feitas"]:
        info(f"etapas já feitas: {', '.join(st['feitas'])}")
    gravar_firmware(st, a)
    salvar_estado(st, est_path)

    b = Bancada(st.get("porta"), DADOS / f"auto_{time.strftime('%Y-%m-%d')}.log", simular=a.simular)
    resultado = {}
    worker = threading.Thread(target=executar, args=(b, st, est_path, resultado), daemon=True)
    worker.start()
    try:
        if a.sem_plot:
            while worker.is_alive():
                worker.join(0.5)
        else:
            plot_loop(b, worker)
            while worker.is_alive():
                worker.join(0.5)
    except KeyboardInterrupt:
        info("Ctrl+C: desligando aquecedor e cooler...")
        b.enviar_seguro("x")
        b.parar.set()
        worker.join(10)
    finally:
        b.enviar_seguro("x")
        b.fechar()
    sys.exit(0 if resultado.get("ok") else 1)


if __name__ == "__main__":
    main()
