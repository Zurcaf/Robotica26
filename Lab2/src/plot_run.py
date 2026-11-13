"""
Tarefa 3 — Pose pedida vs pose medida nos ensaios em modo servo.

Uso (a partir da pasta acima de src/):
    python src/plot_run.py --golfista-perto   # ensaios do relatório -> report/run_servo_golfista_perto.png
    python src/plot_run.py                    # ensaios da primeira versão -> report/run_servo.png

Cada ficheiro data/run_servo_*.csv tem o tempo, a pose pedida ao controlador e
a pose medida com get_position() (ambas do flange, porque o controlador não tem
desvio de ferramenta configurado). Para cada ensaio calcula-se:
  - o erro |p_medido - p_pedido| no mesmo instante (médio, máximo e por fase);
  - o atraso: o tau que minimiza o erro médio entre p_medido(t) e p_pedido(t - tau);
  - o erro que sobra depois de descontar esse atraso, que é o erro de caminho.
"""
import argparse
import os

import numpy as np

from trajectory import DEFAULTS, GOLFISTA_PERTO

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "data")
REPORT = os.path.join(HERE, "..", "report")
# três cores fáceis de distinguir, também por quem tem daltonismo
COR = ["#2a78d6", "#eb6834", "#1baf7a"]
CINZA = "#52514e"
TITULO = {2: "metade da velocidade", 1: "velocidade planeada", 0.7: "43 % acima da planeada"}


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
    """Máscaras de cada fase do swing e os instantes que as separam."""
    tb, td = t_back * lento, t_down * lento
    return {"backswing": t <= tb, "downswing": (t > tb) & (t <= tb + td), "follow-through": t > tb + td}, (tb, tb + td)


def ensaio(nome, lento, preset=DEFAULTS):
    """Lê um ensaio, imprime as estatísticas e devolve o que a figura precisa."""
    d = np.loadtxt(os.path.join(DATA, nome), delimiter=",")
    t, ref, med = d[:, 0], d[:, 1:4], d[:, 7:10]
    e = np.linalg.norm(med - ref, axis=1)
    tau, e_tau = atraso(t, ref, med)
    f, cortes = fases(t, lento, preset["t_back"], preset["t_down"])
    w = 5                                                    # velocidade numa janela de 0.1 s (os instantes têm jitter)
    vel = lambda p: np.linalg.norm(p[2 * w:] - p[:-2 * w], axis=1) / (t[2 * w:] - t[:-2 * w])
    v_ref, v_med = vel(ref), vel(med)
    print(f"  {nome}: {len(t)} pontos, {t[-1]:.1f} s")
    print(f"    erro |p_med - p_ref|   : médio {e.mean():.2f}  máx {e.max():.2f} mm (em t = {t[e.argmax()]:.2f} s)")
    print("    por fase (médio)       : " + ", ".join(f"{k} {e[m].mean():.2f}" for k, m in f.items()) + " mm")
    print(f"    atraso                 : {tau * 1000:.0f} ms -> erro descontado médio {e_tau.mean():.2f}  máx {e_tau.max():.2f} mm")
    print(f"    velocidade do flange   : pedida máx {v_ref.max():.0f}, medida máx {v_med.max():.0f} mm/s")
    return dict(t=t, ref=ref, med=med, e=e, e_tau=e_tau, tau=tau, cortes=cortes, lento=lento)


def figura(ens, out):
    """Pose pedida e medida (em cima) e erro (em baixo), uma coluna por ensaio."""
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
        titulo = TITULO.get(r["lento"], f"--lento {r['lento']:g}")
        a.set_title(f"{titulo}  ({r['t'][-1]:.1f} s)", fontsize=10, pad=24)
        b.set_xlabel("t [s]")
        b.legend(fontsize=8, frameon=False, loc="upper center")
    ax[0, 0].set_ylabel("posição do flange [mm]")
    ax[1, 0].set_ylabel("erro de posição [mm]")
    ax[0, 0].legend(fontsize=8, frameon=False, ncol=2, loc="center left")
    fig.tight_layout()
    os.makedirs(REPORT, exist_ok=True)
    fig.savefig(out, dpi=170)
    print(f"  gráfico em {os.path.normpath(out)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--golfista-perto", action="store_true", help="ensaios do relatório (preset GOLFISTA_PERTO)")
    args = ap.parse_args()
    if args.golfista_perto:
        G = GOLFISTA_PERTO
        ens = [ensaio(f"run_servo_lento{l:g}_golfista_perto.csv", l, G) for l in (2, 1, 0.7)]
        for nome, l in (("run_servo_lento2_golfista_perto_rep2.csv", 2), ("run_servo_lento1_golfista_perto_rep2.csv", 1),
                        ("run_servo_lento2_golfista_perto_sem_tampa.csv", 2), ("run_servo_lento2_golfista_perto_tampa.csv", 2)):
            ensaio(nome, l, G)                                 # repetições e ensaio com a tampa: só estatísticas
        figura(ens, os.path.join(REPORT, "run_servo_golfista_perto.png"))
        return
    ens = [ensaio("run_servo_lento2_phi50.csv", 2), ensaio("run_servo_lento1_phi50.csv", 1)]
    for nome, lento in (("run_servo_lento2_phi50_rep2.csv", 2), ("run_servo_lento1_phi50_rep2.csv", 1)):
        ensaio(nome, lento)                                    # repetições: só estatísticas
    figura(ens, os.path.join(REPORT, "run_servo.png"))


if __name__ == "__main__":
    main()
