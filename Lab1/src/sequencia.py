"""
Sequência cinemática do swing: ângulos, velocidades e binários das juntas
atuadas ao longo do tempo, com a referência a pontilhado e o impacto marcado.

Uso (a partir de Lab1/):
    python src/sequencia.py                          # V17 -> Photos/v17_sequencia.png
    python src/sequencia.py --modelo models/golfer_v13.xml
    python src/sequencia.py --penetracao             # + penetração braço/taco no corpo

Imprime também os números que o relatório usa: pico de velocidade e de
binário por junta (e quando), saturação no downswing, estado no impacto.
"""

import argparse
import os
import numpy as np
import mujoco

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from swing import load, run_swing, _prep, controlo, t_impacto, config, MODEL

PHOTOS = os.path.join("report", "ROB_LAB1_Report", "Photos")
CORES = ["#2a78d6", "#eb6834", "#1baf7a"]


def penetracao(m, d, p, t_fim):
    """
    Distância mínima (negativa = penetração) entre as partes móveis do lado
    esquerdo (braço, mão, punho do taco) e o corpo (tronco, pescoço, cabeça),
    ao longo do swing. Mede o artefacto "braço a atravessar o peito".
    """
    _, cfg, qadr, vadr, uadr = _prep(m, p)
    M_full = np.zeros((m.nv, m.nv))
    moveis = [m.geom(n).id for n in ("upperarm_l", "forearm_l", "hand_l", "grip")]
    corpo = [m.geom(n).id for n in ("waist", "chest", "neck", "head")]
    pior = (np.inf, None, None, 0.0)
    mujoco.mj_resetData(m, d)
    mujoco.mj_forward(m, d)        # sem isto as geoms ainda estão todas na origem
    proximo = 0.0
    while d.time < t_fim:
        if d.time >= proximo:
            proximo += 2e-3
            for g1 in moveis:
                for g2 in corpo:
                    dist = mujoco.mj_geomDistance(m, d, g1, g2, 0.05, None)
                    if dist < pior[0]:
                        pior = (dist, m.geom(g1).name, m.geom(g2).name, d.time)
        d.ctrl[uadr] = controlo(m, d, p, qadr, vadr, uadr, M_full)
        mujoco.mj_step(m, d)
    return pior


def main():
    ap = argparse.ArgumentParser(description="Sequência cinemática do swing")
    ap.add_argument("--modelo", default=MODEL)
    ap.add_argument("--penetracao", action="store_true")
    ap.add_argument("--t-fim", type=float, default=None, help="fim do gráfico [s]")
    args = ap.parse_args()

    m, d = load(args.modelo)
    p, cfg, _, _, _ = _prep(m)
    versao = "v17" if "torso" in cfg["joints"] else "v13"
    nj = len(cfg["joints"])
    t_imp_ref = t_impacto(p)
    t_fim = args.t_fim or (t_imp_ref + 0.6)

    r = run_swing(m, d, {"t_sim": t_fim}, guardar_juntas=True)
    J = r["juntas"]
    t = J[:, 0]
    q, q_ref = J[:, 1:1 + nj], J[:, 1 + nj:1 + 2 * nj]
    w, tau = J[:, 1 + 2 * nj:1 + 3 * nj], J[:, 1 + 3 * nj:1 + 4 * nj]
    t_imp = r["t_impact"] if r["hit"] else t_imp_ref
    lim = [m.actuator_ctrlrange[m.actuator(a).id, 1] for a in cfg["motors"]]

    print(f"modelo {versao}: impacto em t = {t_imp:.3f} s "
          f"(referência {t_imp_ref:.2f} s), v_taco {r['v_head']:.1f} m/s")
    down = (t >= p["t_back"]) & (t <= t_imp)
    for i, (j, lab) in enumerate(zip(cfg["joints"], cfg["labels"])):
        k = np.argmax(np.abs(w[:, i]))
        kt = np.argmax(np.abs(tau[down, i]))
        print(f"  {lab:<7} ({j:<8}) pico vel. {w[k, i]:+7.0f} deg/s em t = {t[k]:.3f} s "
              f"({t[k]-t_imp:+.3f} s do impacto); pico binário no downswing "
              f"{np.abs(tau[down, i]).max():5.0f} N.m (limite {lim[i]:.0f}); "
              f"saturado {100*r['sat'][i]:.0f} % do downswing; "
              f"no impacto q = {r['q_impact'][i]:+.1f} deg, w = {r['w_impact'][i]:+.0f} deg/s")

    # ---- gráfico
    fig, (a1, a2, a3) = plt.subplots(3, 1, figsize=(8.5, 9.6), sharex=True)
    for i, j in enumerate(cfg["joints"]):
        a1.plot(t, q[:, i], color=CORES[i], lw=1.8, label=j)
        a1.plot(t, q_ref[:, i], color=CORES[i], lw=1.0, ls=":", alpha=0.8)
        a2.plot(t, w[:, i], color=CORES[i], lw=1.8, label=j)
        a3.plot(t, tau[:, i], color=CORES[i], lw=1.8, label=f"{j} (±{lim[i]:.0f})")
    a1.set_ylabel("ângulo [°]  (pontilhado = referência)")
    a2.set_ylabel("velocidade [°/s]")
    a3.set_ylabel("binário [N·m]")
    a3.set_xlabel("tempo [s]")
    for ax in (a1, a2, a3):
        ax.axvline(t_imp, color="k", lw=0.9, ls="--")
        ax.grid(alpha=0.3)
        ax.legend(frameon=True, fontsize=9, loc="upper left")
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
    a1.set_xlim(0, t_fim)
    a1.set_title(f"Sequência cinemática do swing {versao.upper()} (tracejado = impacto)")
    fig.tight_layout()
    os.makedirs(PHOTOS, exist_ok=True)
    out = os.path.join(PHOTOS, f"{versao}_sequencia.png")
    fig.savefig(out, dpi=130)
    print(f"  gráfico guardado em {out}")

    if args.penetracao:
        dist, g1, g2, tp = penetracao(m, d, p, t_fim)
        if dist < 0:
            print(f"  penetração máxima: {-dist*1000:.1f} mm ({g1} em {g2}, t = {tp:.2f} s)")
        else:
            print(f"  sem penetração; distância mínima {dist*1000:.1f} mm ({g1}-{g2}, t = {tp:.2f} s)")


if __name__ == "__main__":
    main()
