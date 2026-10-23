#!/usr/bin/env python3
"""
proj1cdig_plot.py — figuras dos CSV do proj1cdig (mesmo visual do gráfico do logger).

Uso:
    python proj1cdig_plot.py dados/proj1cdig_u0_....csv                 # PNG no visual da tela
    python proj1cdig_plot.py dados/proj1cdig_prbs_....csv --relatorio --modo PRBS
    python proj1cdig_plot.py dados/proj1cdig_ensaio_aw1_....csv --relatorio --ise --formato pdf
    python proj1cdig_plot.py arquivo.csv --relatorio --duas-colunas --de 600 --ate 3600 --ta

Opções:
    --relatorio      figura para o relatório (largura de uma coluna do modelo IFAC, fonte 8 pt)
    --duas-colunas   com --relatorio: largura da página inteira
    --formato        png (padrão) ou pdf (vetorial, melhor para o LaTeX)
    --modo X         usa só as linhas do modo X (IDLE, SENS, OPEN, PRBS, CTRL)
    --de / --ate     recorte em segundos, contados do início do arquivo (ou do modo escolhido)
    --ise            sombreia 2100–3000 s e calcula o ISE nessa janela (ensaio do passo 8)
    --ta             com --relatorio: inclui o painel de Ta
    --umax           linha do limite do aquecedor (padrão 16000)
    --saida          nome do arquivo de saída

O logger também usa este módulo: o desenho ao vivo e o PNG salvo no Ctrl+C saem daqui.
"""
import argparse
import csv
import math
import statistics
from pathlib import Path

import numpy as np

U_MAX_PADRAO = 21000
W_FULL = 1023

COR = {
    "tp": "#c62828", "tp_media": "#7f0000", "ta": "#1565c0", "ref": "#212121",
    "u": "#2e7d32", "w": "#0097a7", "umax": "#c62828", "ise": "#fff3cd",
    "estavel": "#2e7d32", "variando": "#ef6c00", "aguardando": "#757575",
}
MEDIA_S = 30          # janela da média móvel de Tp [s]
MAX_PONTOS = 5000     # pontos desenhados por linha (o resto é dizimado só no desenho)


# ---------------------------------------------------------------- utilidades
def media_movel(t, y, janela_s=MEDIA_S):
    """Média móvel causal (últimos janela_s segundos), ignorando NaN."""
    y = np.asarray(y, float)
    if len(y) < 2:
        return y.copy()
    dt = float(np.median(np.diff(t))) or 1.0
    n = max(1, int(round(janela_s / dt)))
    ok = ~np.isnan(y)
    cs = np.concatenate([[0.0], np.cumsum(np.where(ok, y, 0.0))])
    cn = np.concatenate([[0], np.cumsum(ok)])
    i = np.arange(1, len(y) + 1)
    j = np.maximum(0, i - n)
    cnt = cn[i] - cn[j]
    with np.errstate(invalid="ignore", divide="ignore"):
        return np.where(cnt > 0, (cs[i] - cs[j]) / cnt, np.nan)


def estabilidade(t, tp, janela=300, media=30, limite=0.1):
    """Média dos últimos 30 s contra a média de 5 min antes. Devolve (estado, delta)."""
    if len(t) < media or t[-1] - t[0] < janela + media:
        return "aguardando", float("nan")
    agora = float(np.nanmean(tp[-media:]))
    i = int(np.searchsorted(t, t[-1] - janela, side="right")) - 1
    antes = float(np.nanmean(tp[max(0, i - media + 1):i + 1]))
    d = agora - antes
    return ("estavel" if abs(d) < limite else "variando"), d


def escala_tempo(t_rel):
    """Minutos; horas quando o trecho passa de 2 h."""
    if len(t_rel) and t_rel[-1] > 7200:
        return t_rel / 3600.0, "t [h]"
    return t_rel / 60.0, "t [min]"


def dizimar(*arrs):
    n = len(arrs[0])
    if n <= MAX_PONTOS:
        return arrs
    p = math.ceil(n / MAX_PONTOS)
    return tuple(a[::p] for a in arrs)


def ler_csv(caminho, modo=None):
    cols = {k: [] for k in ("t", "k", "modo", "u", "w", "tp", "ta", "ref", "aw")}
    with open(caminho, encoding="utf-8") as f:
        for r in csv.DictReader(f):
            try:
                if modo and r["modo"] != modo:
                    continue
                cols["t"].append(float(r["t_s"])); cols["k"].append(int(r["k"]))
                cols["modo"].append(r["modo"]);    cols["u"].append(float(r["u"]))
                cols["w"].append(float(r["w"]));   cols["tp"].append(float(r["tp"]))
                cols["ta"].append(float(r["ta"])); cols["ref"].append(float(r["ref"]))
                cols["aw"].append(int(r["aw"]))
            except (KeyError, ValueError, TypeError):
                continue
    d = {k: (np.array(v) if k != "modo" else v) for k, v in cols.items()}
    return d


# ---------------------------------------------------------------- painel
class Painel:
    """Monta e atualiza a figura. Serve para o gráfico ao vivo e para os PNG/PDF."""

    def __init__(self, fig, estilo="tela", umax=U_MAX_PADRAO, com_ta=True, ise=False):
        self.fig, self.estilo, self.umax, self.ise = fig, estilo, umax, ise
        tela = estilo == "tela"
        fs = 10 if tela else 8
        lw = 1.0 if tela else 0.8
        self.fs = fs

        alturas = [3, 1.4, 1.6] if com_ta else [3, 1.6]
        larg = fig.get_figwidth()                      # margens fixas em polegadas no relatório
        gs = fig.add_gridspec(len(alturas), 1, height_ratios=alturas, hspace=0.12,
                              left=0.08 if tela else 0.66 / larg, right=0.92 if tela else 1 - 0.62 / larg,
                              top=0.90 if tela else 1 - 0.08 / fig.get_figheight(),
                              bottom=0.08 if tela else 0.45 / fig.get_figheight())
        self.a_tp = fig.add_subplot(gs[0])
        self.a_ta = fig.add_subplot(gs[1], sharex=self.a_tp) if com_ta else None
        self.a_u = fig.add_subplot(gs[-1], sharex=self.a_tp)
        self.a_w = self.a_u.twinx()

        a = self.a_tp
        self.l_tp, = a.plot([], [], color=COR["tp"], lw=lw * (0.7 if tela else 1), alpha=0.45 if tela else 1,
                            label="Tp")
        self.l_tpm, = a.plot([], [], color=COR["tp_media"], lw=lw * 1.8, label=f"Tp (média {MEDIA_S} s)")
        self.l_ref, = a.plot([], [], "--", color=COR["ref"], lw=lw * 1.2, label="ref")
        self.l_tpm.set_visible(tela)
        a.set_ylabel("Tp [°C]", fontsize=fs)
        if self.a_ta is not None:
            self.l_ta, = self.a_ta.plot([], [], color=COR["ta"], lw=lw, label="Ta")
            self.a_ta.set_ylabel("Ta [°C]", fontsize=fs)
        self.l_u, = self.a_u.plot([], [], color=COR["u"], lw=lw, drawstyle="steps-post",
                                  label="u aquecedor" if tela else "u")
        self.l_umax = self.a_u.axhline(umax, ls=":", color=COR["umax"], lw=lw,
                                       label=f"U_MAX = {umax}" if tela else "U_MAX")
        self.l_w, = self.a_w.plot([], [], color=COR["w"], lw=lw, alpha=0.85, drawstyle="steps-post",
                                  label="w cooler" if tela else "w")
        self.a_u.set_ylabel("u [0–65535]" if tela else "u [cont.]", fontsize=fs, color=COR["u"])
        self.a_w.set_ylabel("w [0–1023]" if tela else "w [cont.]", fontsize=fs, color=COR["w"])
        self.a_w.set_ylim(0, W_FULL * 1.45)   # folga no topo para a legenda
        self.a_u.tick_params(axis="y", colors=COR["u"], labelsize=fs)
        self.a_w.tick_params(axis="y", colors=COR["w"], labelsize=fs)

        for ax in (self.a_tp, self.a_ta, self.a_u):
            if ax is None:
                continue
            ax.grid(True, alpha=0.3, lw=0.6)
            ax.tick_params(labelsize=fs)
            ax.margins(x=0.01)
            if ax is not self.a_u:
                plt_setp(ax.get_xticklabels(), visible=False)
        self.a_u.legend(handles=[self.l_u, self.l_umax, self.l_w], loc="upper left",
                        fontsize=fs - 1 if tela else fs - 2, ncol=3, framealpha=0.85,
                        borderpad=0.3, handlelength=1.5, columnspacing=1.0)
        self._ref_visivel = None
        self._ise_patch = None

        if tela:
            self.txt = fig.text(0.08, 0.955, "", fontsize=fs - 0.5, family="monospace", va="center",
                                bbox=dict(boxstyle="round,pad=0.4", fc="#f5f5f5", ec="#bdbdbd"))
            self.selo = fig.text(0.92, 0.955, "", fontsize=fs + 1, weight="bold", color="white",
                                 ha="right", va="center",
                                 bbox=dict(boxstyle="round,pad=0.4", fc=COR["aguardando"], ec="none"))

    def atualizar(self, d):
        t = np.asarray(d["t"], float)
        if len(t) < 2:
            return
        t_rel = t - t[0]
        x, xlab = escala_tempo(t_rel)
        tp, ta = np.asarray(d["tp"], float), np.asarray(d["ta"], float)
        u, w, ref = np.asarray(d["u"], float), np.asarray(d["w"], float), np.asarray(d["ref"], float)
        modos = d["modo"]
        tpm = media_movel(t, tp)

        xs, tps, tpms, tas, us, ws, refs = dizimar(x, tp, tpm, ta, u, w, ref)
        self.l_tp.set_data(xs, tps)
        self.l_tpm.set_data(xs, tpms)
        ctrl = np.array([m == "CTRL" for m in modos])
        ref_plot = np.where(ctrl, ref, np.nan)
        self.l_ref.set_data(*dizimar(x, ref_plot))
        tem_ref = bool(ctrl.any())
        if tem_ref != self._ref_visivel:
            self._ref_visivel = tem_ref
            self.l_ref.set_visible(tem_ref)
            hs = [self.l_tp] + ([self.l_tpm] if self.estilo == "tela" else []) + ([self.l_ref] if tem_ref else [])
            self.a_tp.legend(handles=hs, loc="best", fontsize=self.fs - 1 if self.estilo == "tela" else self.fs - 2,
                             framealpha=0.85, ncol=len(hs), borderpad=0.3, handlelength=1.5)
        if self.a_ta is not None:
            self.l_ta.set_data(xs, tas)
        self.l_u.set_data(xs, us)
        self.l_w.set_data(xs, ws)

        for ax in (self.a_tp, self.a_ta):
            if ax is not None:
                ax.relim(); ax.autoscale_view()
        self.a_u.set_xlim(x[0], x[-1] if x[-1] > x[0] else x[0] + 1)
        self.a_u.set_ylim(0, max(self.umax, np.nanmax(u) if len(u) else 0) * 1.4)   # folga para a legenda
        self.a_u.set_xlabel(xlab, fontsize=self.fs)

        if self.ise:
            fator = 3600.0 if xlab == "t [h]" else 60.0
            if self._ise_patch is None:
                self._ise_patch = self.a_tp.axvspan(2100 / fator, 3000 / fator, color=COR["ise"], zorder=0,
                                                    label="janela do ISE")

        if self.estilo == "tela":
            sd = statistics.pstdev([v for v in tp[-100:] if not math.isnan(v)]) if len(tp) > 2 else float("nan")
            estado, delta = estabilidade(t, tp)
            self.txt.set_text(
                f"{modos[-1]} u={u[-1]:.0f} w={w[-1]:.0f} │ "
                f"Tp={tp[-1]:.3f} Ta={ta[-1]:.3f} °C │ "
                f"σ(Tp,{min(100, len(tp))})={sd:.4f} │ Δ5min={delta:+.3f} °C")
            rot = {"estavel": "ESTÁVEL", "variando": "ainda variando",
                   "aguardando": "aguardando 5,5 min"}[estado]
            self.selo.set_text(rot)
            self.selo.get_bbox_patch().set_facecolor(COR[estado])


def plt_setp(objs, **kw):
    for o in objs:
        o.set(**kw)


# ---------------------------------------------------------------- PNG / PDF a partir do CSV
def salvar(csv_path, saida=None, estilo="tela", umax=U_MAX_PADRAO, modo=None, de=None, ate=None,
           ise=False, com_ta=None, duas_colunas=False, formato="png"):
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg

    d = ler_csv(csv_path, modo)
    if len(d["t"]) < 2:
        print(f"[plot] {csv_path}: poucas amostras, nada a desenhar")
        return None
    t_rel = d["t"] - d["t"][0]
    sel = np.ones(len(t_rel), bool)
    if de is not None:
        sel &= t_rel >= de
    if ate is not None:
        sel &= t_rel <= ate
    d = {k: (v[sel] if k != "modo" else [m for m, s in zip(v, sel) if s]) for k, v in d.items()}

    if com_ta is None:
        com_ta = estilo == "tela"
    if estilo == "tela":
        tam = (12, 7.5)
    else:
        tam = (7.0, 3.6 if not com_ta else 4.4) if duas_colunas else (3.4, 3.0 if not com_ta else 3.8)
    fig = Figure(figsize=tam, dpi=150 if estilo == "tela" else 300)
    FigureCanvasAgg(fig)
    p = Painel(fig, estilo, umax, com_ta=com_ta, ise=ise)
    p.atualizar(d)

    if ise:
        t0 = d["t"][0]
        j = (d["t"] - t0 >= 2100) & (d["t"] - t0 < 3000)
        if j.any():
            dt = np.diff(d["t"], append=d["t"][-1] + np.median(np.diff(d["t"])))
            ise_val = float(np.nansum((d["ref"][j] - d["tp"][j]) ** 2 * dt[j]))
            print(f"[plot] ISE (2100–3000 s) = {ise_val:.1f} °C²·s")

    if saida is None:
        suf = "" if estilo == "tela" else "_relatorio"
        saida = Path(csv_path).with_name(Path(csv_path).stem + suf + "." + formato)
    fig.savefig(saida)
    print(f"[plot] figura salva em {saida}")
    return saida


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv")
    ap.add_argument("--relatorio", action="store_true")
    ap.add_argument("--duas-colunas", action="store_true")
    ap.add_argument("--formato", choices=["png", "pdf"], default="png")
    ap.add_argument("--modo")
    ap.add_argument("--de", type=float)
    ap.add_argument("--ate", type=float)
    ap.add_argument("--ise", action="store_true")
    ap.add_argument("--ta", action="store_true")
    ap.add_argument("--umax", type=int, default=U_MAX_PADRAO)
    ap.add_argument("--saida")
    a = ap.parse_args()
    estilo = "relatorio" if a.relatorio else "tela"
    salvar(a.csv, a.saida, estilo, a.umax, a.modo, a.de, a.ate, a.ise,
           com_ta=(a.ta if a.relatorio else True), duas_colunas=a.duas_colunas, formato=a.formato)


if __name__ == "__main__":
    main()
