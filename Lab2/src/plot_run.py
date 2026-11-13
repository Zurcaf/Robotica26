"""
Tarefa 3 — Referência vs pose medida nos ensaios em servo (dados do laboratório).

Uso (a partir de Lab2/):
    python src/plot_run.py            # estatísticas + report/run_servo.png
    python src/plot_run.py --golfista-perto   # ensaios do relatório -> report/run_servo_golfista_perto.png

Lê data/run_servo_lento<N>_phi50.csv (colunas: t, pose pedida ao controlador,
pose medida com get_position(); com tcp_offset = 0 ambas são do FLANGE) e, para
cada ensaio, calcula:
  - erro |p_medido - p_pedido| (médio, máximo e por fase);
  - atraso: o tau que minimiza o erro médio entre p_medido(t) e p_pedido(t - tau);
  - erro que sobra depois de descontar esse atraso (erro de CAMINHO, não de tempo).
"""
import argparse
import os

import numpy as np

import config as C
from kinematics import fk, tool_transform
from trajectory import DEFAULTS, GOLFISTA_PERTO

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "data")
REPORT = os.path.join(HERE, "..", "report")
# cores: 3 primeiras do conjunto categórico (distinguíveis com daltonismo)
COR = ["#2a78d6", "#eb6834", "#1baf7a"]
CINZA = "#52514e"


def atraso(t, ref, med, tau_max=0.2, passo=0.001):
    """tau [s] que minimiza o erro médio entre med(t) e ref(t - tau), e esse erro em cada instante."""
    melhor = (np.inf, 0.0, None)
    for tau in np.arange(0.0, tau_max + passo / 2, passo):
        r = np.column_stack([np.interp(t - tau, t, ref[:, i]) for i in range(ref.shape[1])])
        e = np.linalg.norm(med - r, axis=1)
        if e.mean() < melhor[0]:
            melhor = (e.mean(), tau, e)
    return melhor[1], melhor[2]


def fases(t, lento, t_back=DEFAULTS["t_back"], t_down=DEFAULTS["t_down"]):
    tb, td = t_back * lento, t_down * lento
    return {"backswing": t <= tb, "downswing": (t > tb) & (t <= tb + td), "follow-through": t > tb + td}, (tb, tb + td)


def ensaio(nome, lento, preset=DEFAULTS):
    d = np.loadtxt(os.path.join(DATA, nome), delimiter=",")
    t, ref, med = d[:, 0], d[:, 1:4], d[:, 7:10]
    e = np.linalg.norm(med - ref, axis=1)
    tau, e_tau = atraso(t, ref, med)
    f, cortes = fases(t, lento, preset["t_back"], preset["t_down"])
    w = 5                                                    # janela de 0.1 s: os instantes gravados têm jitter
    vel = lambda p: np.linalg.norm(p[2 * w:] - p[:-2 * w], axis=1) / (t[2 * w:] - t[:-2 * w])
    v_ref, v_med = vel(ref), vel(med)
    print(f"  {nome}: {len(t)} pontos, {t[-1]:.1f} s")
    print(f"    erro |p_med - p_ref|   : médio {e.mean():.2f}  máx {e.max():.2f} mm (em t = {t[e.argmax()]:.2f} s)")
    print("    por fase (médio)       : " + ", ".join(f"{k} {e[m].mean():.2f}" for k, m in f.items()) + " mm")
    print(f"    atraso                 : {tau * 1000:.0f} ms -> erro descontado médio {e_tau.mean():.2f}  máx {e_tau.max():.2f} mm")
    print(f"    velocidade do flange   : pedida máx {v_ref.max():.0f}, medida máx {v_med.max():.0f} mm/s")
    return dict(t=t, ref=ref, med=med, e=e, e_tau=e_tau, tau=tau, cortes=cortes, lento=lento)


def figura(ens, out):
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(2, len(ens), figsize=(5.2 * len(ens), 6.2), sharex="col", sharey="row", squeeze=False)
    for j, r in enumerate(ens):
        a, b = ax[0, j], ax[1, j]
        for i, n in enumerate("xyz"):
            a.plot(r["t"], r["med"][:, i], color=COR[i], lw=2, label=f"{n} medido")
            a.plot(r["t"], r["ref"][:, i], color="k", lw=0.9, ls=(0, (4, 3)), label="pedido" if i == 0 else None)
        b.plot(r["t"], r["e"], color=COR[0], lw=2, label="erro no mesmo instante")
        b.plot(r["t"], r["e_tau"], color=COR[1], lw=2, label=f"descontando o atraso ({r['tau'] * 1000:.0f} ms)")
        for eixo in (a, b):
            for c in r["cortes"]:
                eixo.axvline(c, color=CINZA, lw=0.8, ls=":")
            eixo.grid(alpha=0.25, lw=0.5)
            eixo.spines[["top", "right"]].set_visible(False)
        topo = a.get_ylim()[1]
        for x, s in zip([0.5 * r["cortes"][0], np.mean(r["cortes"]), 0.5 * (r["cortes"][1] + r["t"][-1])],
                        ["backswing", "downswing", "follow-\nthrough"]):
            a.text(x, topo, s, ha="center", va="bottom", fontsize=8, color=CINZA)
        a.set_title(f"--lento {r['lento']:g}  ({r['t'][-1]:.1f} s)", fontsize=10, pad=24)
        b.set_xlabel("t [s]")
        b.legend(fontsize=8, frameon=False, loc="upper center")
    ax[0, 0].set_ylabel("posição do flange [mm]")
    ax[1, 0].set_ylabel("erro de posição [mm]")
    ax[0, 0].legend(fontsize=8, frameon=False, ncol=2, loc="center left")
    fig.tight_layout()
    os.makedirs(REPORT, exist_ok=True)
    fig.savefig(out, dpi=170)
    print(f"  gráfico em {os.path.normpath(out)}")


def ponta(q):
    return (fk(q) @ tool_transform(C.GRIPPER_TCP_Z))[:3, 3]


def v3(nome="run_v3_lento2.csv"):
    """Swing V3 (servo nas juntas): erro das juntas, erro da ponta, e a nossa FK vs a pose do controlador."""
    import matplotlib.pyplot as plt

    d = np.loadtxt(os.path.join(DATA, nome), delimiter=",")
    t, qr, qm, pose = d[:, 0], d[:, 1:7], d[:, 7:13], d[:, 13:19]
    eq = np.abs(qm - qr)
    tau, _ = atraso(t, qr, qm)
    pr, pm = np.array([ponta(q) for q in qr]), np.array([ponta(q) for q in qm])
    ep = np.linalg.norm(pm - pr, axis=1)
    fl = np.array([fk(q)[:3, 3] for q in qm])                 # nossa FK das juntas MEDIDAS
    efk = fl - pose[:, :3]                                    # vs flange lido do controlador
    print(f"  {nome}: {len(t)} amostras, {t[-1]:.1f} s")
    print(f"    erro das juntas        : médio {eq.mean():.3f}  máx {eq.max():.2f} deg (atraso {tau * 1000:.0f} ms)")
    print(f"    ponta (FK de q medido) : médio {ep.mean():.2f}  máx {ep.max():.2f} mm vs planeada; z mín {pm[:, 2].min():.1f} mm")
    n = np.linalg.norm(efk, axis=1)
    print(f"    nossa FK vs controlador: médio {n.mean():.2f}  máx {n.max():.2f} mm; viés (x,y,z) {np.round(efk.mean(0), 2)}")
    print(f"    amplitude das juntas   : {np.round(qm.max(0) - qm.min(0), 0)} deg")

    fig, ax = plt.subplots(2, 1, figsize=(7, 5.2), sharex=True)
    ax[0].plot(t, ep, color=COR[0], lw=2)
    ax[0].set_ylabel("erro da ponta [mm]")
    ax[1].plot(t, n, color=COR[1], lw=2)
    ax[1].set_ylabel("nossa FK vs controlador [mm]")
    ax[1].set_xlabel("t [s]")
    for a in ax:
        a.grid(alpha=0.25, lw=0.5)
        a.spines[["top", "right"]].set_visible(False)
        a.set_ylim(bottom=0)
    fig.tight_layout()
    out = os.path.join(REPORT, "run_v3.png")
    fig.savefig(out, dpi=170)
    print(f"  gráfico em {os.path.normpath(out)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--v3", action="store_true")
    ap.add_argument("--golfista-perto", action="store_true", help="ensaios do GOLFISTA_PERTO -> report/run_servo_golfista_perto.png")
    args = ap.parse_args()
    if args.v3:
        v3()
        v3("run_v3_lento2_rep2.csv")
        return
    if args.golfista_perto:
        G = GOLFISTA_PERTO
        ens = [ensaio(f"run_servo_lento{l:g}_golfista_perto.csv", l, G) for l in (2, 1, 0.7)]
        for nome, l in (("run_servo_lento2_golfista_perto_rep2.csv", 2), ("run_servo_lento1_golfista_perto_rep2.csv", 1),
                        ("run_servo_lento2_golfista_perto_sem_tampa.csv", 2), ("run_servo_lento2_golfista_perto_tampa.csv", 2)):
            ensaio(nome, l, G)
        figura(ens, os.path.join(REPORT, "run_servo_golfista_perto.png"))
        return
    ens = [ensaio("run_servo_lento2_phi50.csv", 2), ensaio("run_servo_lento1_phi50.csv", 1)]
    for nome, lento in (("run_servo_lento2_phi50_rep2.csv", 2), ("run_servo_lento1_phi50_rep2.csv", 1)):
        ensaio(nome, lento)                                    # repetições: só estatísticas
    figura(ens, os.path.join(REPORT, "run_servo.png"))


if __name__ == "__main__":
    main()
