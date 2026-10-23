#!/usr/bin/env python3
"""
proj1cdig_logger.py — coleta serial do firmware proj1cdig, grava CSV e plota em tempo real.

Uso:
    python proj1cdig_logger.py COM5 --tag prbs --cmd "p 8320 1680 60 2000"
    python proj1cdig_logger.py COM5 --tag sensores --cmd s
    python proj1cdig_logger.py COM5 --tag ensaio_aw1 --cmd "e"

Enquanto roda, o que você digitar no terminal (e Enter) vai direto para a ESP32:
    e         ensaio do roteiro (Fig. 5)  a 0 / a 1   anti-windup off/on
    r 60      degrau de referência        wc 600      cooler constante
    ws / wt   perturbação senoidal/triang.
    w0        cooler off                  x            para tudo

Dependências: pip install pyserial matplotlib

Notas para a coleta de 8 h:
  * A porta é aberta com DTR/RTS desligados para NÃO resetar a ESP32. Se a conexão cair,
    o script tenta reabrir sozinho, sem reset, e a coleta continua.
  * Desative a suspensão do Windows (Configurações > Energia) e o "USB selective suspend".
  * --noplot economiza CPU se o notebook for fraco; o CSV é gravado do mesmo jeito.

Gráfico: Tp (bruto + média de 30 s; ref só em malha fechada), Ta, u (eixo da esquerda, com a
linha de U_MAX) e w (eixo da direita). Selo verde = ESTÁVEL. Ao fechar (Ctrl+C ou fechando a
janela) o logger salva dados/<mesmo nome do CSV>.png com o arquivo inteiro.
Figuras para o relatório: proj1cdig_plot.py  (python proj1cdig_plot.py -h)
"""
import argparse
import queue
import sys
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np
import serial

FIELDS = ["t_s", "k", "modo", "u", "w", "tp", "ta", "ref", "aw"]
MODOS = {"IDLE", "SENS", "OPEN", "PRBS", "CTRL"}


def open_port(port, baud):
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, baud, 1
    s.dtr = False          # evita o auto-reset da DevKit ao abrir a porta
    s.rts = False
    s.open()
    # O CH340 costuma mandar um byte de lixo para a ESP ao abrir a porta; ele grudaria no
    # primeiro comando ("#? comando desconhecido"). Um Enter sozinho encerra essa linha.
    time.sleep(0.1)
    s.write(b"\n")
    time.sleep(0.2)
    return s


class Link:
    """Serial com reconexão automática (sem reset)."""

    def __init__(self, port, baud):
        self.port, self.baud = port, baud
        self.ser = None
        self.lock = threading.Lock()
        self.connect()

    def connect(self):
        while True:
            try:
                self.ser = open_port(self.port, self.baud)
                print(f"[logger] conectado em {self.port}")
                return
            except serial.SerialException as e:
                print(f"[logger] aguardando {self.port}: {e}")
                time.sleep(2)

    def readline(self):
        try:
            return self.ser.readline()
        except (serial.SerialException, OSError) as e:
            print(f"[logger] conexão caiu ({e}); reconectando...")
            try:
                self.ser.close()
            except Exception:
                pass
            self.connect()
            return b""

    def send(self, text):
        with self.lock:
            try:
                self.ser.write((text.strip() + "\n").encode())
            except (serial.SerialException, OSError) as e:
                print(f"[logger] falha ao enviar '{text}': {e}")


def reader(link, q, csv_f, log_f, stop):
    n_dados = 0
    while not stop.is_set():
        raw = link.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").strip()
        if line.startswith("D,"):
            # Só vai para o CSV a linha que passa inteira na conversão: um byte corrompido
            # numa linha de 9 h deixaria a coluna como texto no readtable do MATLAB.
            parts = line[2:].split(",")
            try:
                if len(parts) != len(FIELDS) or parts[2] not in MODOS:
                    raise ValueError
                rec = dict(zip(FIELDS, parts))
                d = {
                    "t": float(rec["t_s"]), "modo": rec["modo"],
                    "u": float(rec["u"]), "w": float(rec["w"]),
                    "tp": float(rec["tp"]), "ta": float(rec["ta"]),
                    "ref": float(rec["ref"]),
                }
                int(rec["k"]); int(rec["aw"])
            except ValueError:
                log_f.write(f"{time.strftime('%H:%M:%S')} [descartada] {line}\n")
                log_f.flush()
                continue
            csv_f.write(",".join(parts) + f",{time.time():.3f}\n")
            csv_f.flush()
            n_dados += 1
            q.put(d)
        elif line:
            print(line)
            log_f.write(f"{time.strftime('%H:%M:%S')} {line}\n")
            log_f.flush()
            if line.startswith("#proj1cdig iniciado") and n_dados > 0:
                aviso = ("[logger] *** A ESP32 REINICIOU depois de {} amostras: voltou para IDLE e a coleta "
                         "parou. Confira alimentação/USB e reenvie o comando. ***".format(n_dados))
                print(aviso)
                log_f.write(f"{time.strftime('%H:%M:%S')} {aviso}\n")
                log_f.flush()


def stdin_forward(link, stop):
    for line in sys.stdin:
        if stop.is_set():
            break
        if line.strip():
            link.send(line)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("port")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--tag", default="coleta", help="vai no nome do arquivo")
    ap.add_argument("--cmd", action="append", default=[], help="comando(s) enviados ao abrir")
    ap.add_argument("--dir", default="dados")
    ap.add_argument("--noplot", action="store_true")
    ap.add_argument("--window", type=float, default=0, help="janela do gráfico [s]; 0 = tudo")
    ap.add_argument("--umax", type=int, default=21000, help="linha do limite do aquecedor")
    a = ap.parse_args()

    Path(a.dir).mkdir(exist_ok=True)
    stamp = time.strftime("%Y-%m-%d_%H-%M-%S")
    csv_path = Path(a.dir) / f"proj1cdig_{a.tag}_{stamp}.csv"
    log_path = csv_path.with_suffix(".log")
    csv_f = open(csv_path, "w", encoding="utf-8")
    csv_f.write(",".join(FIELDS) + ",pc_time\n")
    log_f = open(log_path, "w", encoding="utf-8")
    print(f"[logger] gravando {csv_path}")

    link = Link(a.port, a.baud)
    time.sleep(0.5)
    for c in a.cmd:
        link.send(c)

    q, stop = queue.Queue(), threading.Event()
    threading.Thread(target=reader, args=(link, q, csv_f, log_f, stop), daemon=True).start()
    threading.Thread(target=stdin_forward, args=(link, stop), daemon=True).start()

    try:
        if a.noplot:
            n = 0
            while True:
                d = q.get()
                n += 1
                if n % 10 == 0:
                    print(f"t={d['t']:.0f}s {d['modo']} u={d['u']:.0f} w={d['w']:.0f} "
                          f"Tp={d['tp']:.3f} Ta={d['ta']:.3f}")
        else:
            plot_loop(q, a.window, a.umax, csv_path.name)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        csv_f.close()
        log_f.close()
        print(f"[logger] salvo em {csv_path}")
        try:                                   # figura do arquivo inteiro, para o relatório/avaliação
            from proj1cdig_plot import salvar
            salvar(csv_path, umax=a.umax)
        except Exception as e:
            print(f"[logger] não consegui salvar o PNG: {e}")


def plot_loop(q, window, umax, titulo):
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation
    from proj1cdig_plot import Painel

    N = int(window) if window and window > 0 else None      # None = guarda tudo
    buf = {k: deque(maxlen=N) for k in ("t", "tp", "ta", "ref", "u", "w", "modo")}

    fig = plt.figure(figsize=(12, 7.5))
    try:
        fig.canvas.manager.set_window_title(f"proj1cdig — {titulo}")
    except Exception:
        pass
    painel = Painel(fig, "tela", umax)

    def upd(_):
        got = False
        while not q.empty():
            d = q.get_nowait()
            for k in buf:
                buf[k].append(d[k])
            got = True
        if not got or len(buf["t"]) < 2:
            return
        painel.atualizar({k: (list(v) if k == "modo" else np.fromiter(v, float, len(v)))
                          for k, v in buf.items()})

    _ani = FuncAnimation(fig, upd, interval=1000, cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    main()
