"""
simulador.py — bancada térmica virtual que fala o mesmo protocolo do firmware AUTO.

Usado por `python proj1cdig_auto.py --simular` para ensaiar o fluxo inteiro sem hardware
(a coleta de 9 h roda em segundos). A planta é o modelo P2DZ de referência do roteiro,
com ruído, deriva de ambiente, zona morta do cooler e um erro de ganho/offset no LM35
para a calibração ter o que corrigir.

Não é usado na bancada real.
"""
import math
import random
import threading
import time
from collections import deque

FULL = 1023


class SerialSimulada:
    """Imita serial.Serial: readline() devolve linhas do 'firmware', write() recebe comandos."""

    def __init__(self, velocidade=600.0, semente=1):
        random.seed(semente)
        self.vel = velocidade
        self.saida = deque()
        self.lock = threading.Lock()
        # --- planta (modelo de referência do roteiro) ---
        # ganho esperado para 3 × 150 Ω em 12 V (o grupo de 1/2026 chegou a ~52 °C com 14 %),
        # por contagem do PWM de 16 bits: 0,19 °C por contagem de 10 bits ÷ 64
        self.Kp, self.Tp1, self.Tp2, self.Tz, self.Td = 0.19 / 64, 353.0, 48.9, 249.0, 4.66
        self.dz = 320                      # zona morta real do cooler (escala 0..1023)
        self.kc = 2.56                     # efeito do cooler em "contagens equivalentes" do aquecedor
        self.x1 = self.x2 = 0.0            # estados modais (°C acima do ambiente)
        self.t = 0.0                       # tempo da bancada [s]
        self.atraso = deque([0.0] * int(self.Td / 0.1), maxlen=int(self.Td / 0.1))
        # --- firmware ---
        self.umax_hard = 16000
        self.umax, self.ts_ms = 16000, 2330
        self.cal = [1.0, 0.0, 1.0, 0.0]
        self.woff, self.wamp, self.w0 = 460, 256, 0.065
        self.mode, self.k, self.u = "IDLE", 0, 0
        self.u_open, self.prbs = 0, [8320, 1680, 60, 2000]
        self.lfsr, self.prbs_u = 0x1A5, 0
        self.ref, self.aw, self.K, self.z0, self.I = 50.0, True, 2140.0, 0.9447, 0.0
        self.ensaio, self.e_cfg, self.t_ens0, self.fase, self.ise, self.ise_ok = False, (50.0, 60.0, 400), 0.0, -1, 0.0, False
        self.wmode, self.wc, self.wt0 = "off", 0, 0.0
        self.prox = 0.0                    # instante da próxima amostra
        self._msg("#proj1cdig iniciado (AUTO) [SIMULADOR]")
        self._msg("#ADC ok: Tp no GPIO35, Ta no GPIO34")

    # ---------------- interface tipo pyserial ----------------
    dtr = rts = False

    def open(self):
        pass

    def close(self):
        pass

    def write(self, data):
        for linha in data.decode().splitlines():
            if linha.strip():
                with self.lock:
                    self._comando(linha.strip())
        return len(data)

    def readline(self):
        with self.lock:
            if self.saida:
                return (self.saida.popleft() + "\n").encode()
            modo = self.mode
            periodo = self.ts_ms / 1000 if modo == "CTRL" else 1.0
            if modo != "IDLE":
                self._avancar_ate(self.prox)
                self._amostra()             # a linha D vai para a fila; sai na próxima chamada
                self.prox += periodo
        if modo == "IDLE":
            time.sleep(0.05)
            with self.lock:
                self._avancar_ate(self.t + 0.05 * self.vel)
                self.prox = self.t
            return b""
        time.sleep(periodo / self.vel)
        return b""

    # ---------------- planta ----------------
    def _ta_real(self):
        return 25.0 + 0.4 * math.sin(2 * math.pi * self.t / 20000)

    def _w(self):
        th = self.w0 * (self.t - self.wt0)
        if self.wmode == "off":
            return 0
        if self.wmode == "c":
            return self.wc
        if self.wmode == "s":
            return int(round(self.woff + self.wamp * math.sin(th)))
        return int(round(self.woff + self.wamp * (2 / math.pi) * math.asin(math.sin(th))))

    def _avancar_ate(self, t_alvo):
        dt = 0.1
        A = (self.Tz - self.Tp1) / (self.Tp2 - self.Tp1)
        B = 1 - A
        while self.t < t_alvo:
            w = self._w()
            # o cooler aumenta a perda de calor: efeito proporcional a (T - Ta), nulo no ambiente
            ueff = self.u - self.kc * max(0, w - self.dz) * max(0.0, self.x1 + self.x2) / 25.0
            self.atraso.append(ueff)
            ud = self.atraso[0]
            self.x1 += dt * (-self.x1 + self.Kp * A * ud) / self.Tp1
            self.x2 += dt * (-self.x2 + self.Kp * B * ud) / self.Tp2
            self.t += dt

    def _tp_real(self):
        return self._ta_real() + self.x1 + self.x2

    def _ler(self, real):
        bruto = (real - 0.8) / 1.02 + random.gauss(0, 0.03)   # LM35 + ADC interno: erro de ganho, offset e ruído
        return bruto

    # ---------------- firmware ----------------
    def _msg(self, s):
        self.saida.append(s)

    def _estado(self):
        self._msg(f"#ESTADO modo={self.mode} ref={self.ref:.2f} K={self.K:.4f} z0={self.z0:.5f} aw={int(self.aw)} "
                  f"prbs=[{' '.join(map(str, self.prbs))}] wmode={self.wmode} umax={self.umax} ts={self.ts_ms} "
                  f"cal=[{' '.join(f'{c:.5f}' for c in self.cal)}] wcfg=[{self.woff} {self.wamp}] w0={self.w0:.4f} ens={int(self.ensaio)}")

    def _lfsr(self):
        b = ((self.lfsr >> 0) ^ (self.lfsr >> 4)) & 1
        self.lfsr = (self.lfsr >> 1) | (b << 8)
        return self.lfsr & 1

    def _amostra(self):
        tp = self.cal[0] * self._ler(self._tp_real()) + self.cal[1]
        ta = self.cal[2] * self._ler(self._ta_real()) + self.cal[3]
        if self.mode in ("IDLE", "SENS"):
            self.u = 0
        elif self.mode == "OPEN":
            self.u = self.u_open
        elif self.mode == "PRBS":
            u0, du, bit, aq = self.prbs
            if self.k < aq:
                self.prbs_u = u0
            elif (self.k - aq) % bit == 0:
                self.prbs_u = u0 + du if self._lfsr() else u0 - du
            if self.k >= aq + 511 * bit:
                self.prbs_u = u0
                if self.k == aq + 511 * bit:
                    self._msg(f"#PRBS concluido em k={self.k}")
            self.u = self.prbs_u
        elif self.mode == "CTRL":
            te = -1.0
            if self.ensaio:                                   # ensaio do roteiro (comando e)
                te = self.t - self.t_ens0
                lo, hi, meio = self.e_cfg
                if te >= 3600:
                    self.ensaio, self.ref, self.wmode = False, lo, "off"
                    self._msg(f"#ENSAIO concluido: ISE(2100-3000 s) = {self.ise:.1f}")
                else:
                    self.ref = lo if int(te // meio) % 2 == 0 else hi
                    f = 0 if te < 1000 else 1 if te < 1800 else 2 if te < 2600 else 3
                    if f != self.fase:
                        self.wt0 = self.t
                        self.wmode = ["off", "c", "t", "c"][f]
                        self.wc = self.woff + self.wamp
                        self._msg(f"#ENSAIO t={te:.0f} s: fase {f}")
                        self.fase = f
            e = self.ref - tp
            if self.ensaio and 2100 <= te < 3000:
                self.ise += e * e * self.ts_ms / 1000
            if self.ensaio and te >= 3000 and not self.ise_ok:
                self.ise_ok = True
                self._msg(f"#ISE(2100-3000 s) = {self.ise:.1f}")
            u = self.K * e + self.I
            hi, lo = u > self.umax, u < 0
            if not (self.aw and ((hi and e > 0) or (lo and e < 0))):
                self.I += self.K * (1 - self.z0) * e
            self.u = int(round(min(max(u, 0), self.umax)))
        self.u = min(max(self.u, 0), self.umax)
        if tp > 95 and self.u > 0:
            self.u, self.mode = 0, "IDLE"
            self._msg("#TRIP")
        ref = f"{self.ref:.2f}" if self.mode == "CTRL" else "nan"
        self._msg(f"D,{self.t:.3f},{self.k},{self.mode},{self.u},{self._w()},{tp:.3f},{ta:.3f},{ref},{int(self.aw)}")
        self.k += 1

    def _reinicia(self, modo):
        self.mode, self.k, self.prox = modo, 0, self.t
        if modo == "CTRL":
            self.I = float(self.u)

    def _comando(self, l):
        c, a = l[0], l[1:].split()
        if c in "xsopcrw":
            self.ensaio = False
        try:
            if c == "e":
                v = [float(x) for x in a] + [50.0, 60.0, 400.0][len(a):]
                self.e_cfg = (v[0], v[1], int(v[2]))
                self.ref, self.ensaio, self.wmode = v[0], True, "off"
                self.t_ens0, self.fase, self.ise, self.ise_ok = self.t, -1, 0.0, False
                self._msg("#ENSAIO iniciado")
                self._reinicia("CTRL")
            elif c == "x":
                self._reinicia("IDLE"); self.wmode = "off"; self.u = 0
            elif c == "s":
                self._reinicia("SENS")
            elif c == "o":
                self.u_open = min(max(int(a[0]), 0), self.umax); self._reinicia("OPEN")
            elif c == "p":
                v = [int(x) for x in a] + self.prbs[len(a):]
                if v[0] + v[1] > self.umax or v[0] - v[1] < 0:
                    self._msg("#ERRO PRBS fora da faixa"); return
                self.prbs = v[:4]; self.lfsr = 0x1A5; self._reinicia("PRBS")
            elif c == "c":
                if a: self.ref = float(a[0])
                self._reinicia("CTRL")
            elif c == "r":
                self.ref = float(a[0])
            elif c == "a":
                self.aw = int(a[0]) != 0
            elif c == "k":
                self.K, self.z0 = float(a[0]), float(a[1])
            elif c == "w":
                self.wt0 = self.t
                m = l[1]
                if m == "0": self.wmode = "off"
                elif m == "c": self.wmode, self.wc = "c", int(l[2:])
                elif m == "s": self.wmode = "s"
                elif m == "t": self.wmode = "t"
            elif c == "U":
                v = int(a[0])
                if not 1 <= v <= self.umax_hard:
                    self._msg(f"#ERRO U fora de 1..{self.umax_hard}"); return
                self.umax = v
            elif c == "T":
                self.ts_ms = int(a[0])
            elif c == "C":
                self.cal = [float(x) for x in a[:4]]
            elif c == "W":
                self.woff, self.wamp = int(a[0]), int(a[1])
            elif c == "F":
                self.w0 = float(a[0])
            elif c == "h":
                self._msg("# (ajuda do simulador)"); return
            else:
                self._msg(f"#? comando desconhecido: {l}"); return
        except (IndexError, ValueError):
            self._msg(f"#ERRO argumento invalido: {l}"); return
        self._estado()
