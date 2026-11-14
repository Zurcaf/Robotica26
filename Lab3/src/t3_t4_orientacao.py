"""Tarefas 3 e 4: inclinómetro (acelerómetro) e integração do giroscópio.

    .venv/bin/python src/t3_t4_orientacao.py --sim inclinacoes
    .venv/bin/python src/t3_t4_orientacao.py --sim inclinar_abanar
    .venv/bin/python src/t3_t4_orientacao.py data/..._D_inclinacoes.csv \\
        --cal resultados/real/calibracao.json
    .venv/bin/python src/t3_t4_orientacao.py data/..._inclinacoes_imu.csv \\
        --robo data/..._inclinacoes_robo.csv          # verdade do robô

Calcula, sobre os mesmos dados:
  - inclinómetro: rolamento e arfagem pelo acelerómetro (Tarefa 3);
  - giroscópio integrado com quaterniões, a partir da inclinação inicial
    dada pelo acelerómetro, sem e com calibração (Tarefa 4).
Se houver verdade (simulador ou robô), mostra o erro de cada um. Sem
verdade, imprime o ângulo médio em cada paragem para comparar com o
transferidor. Figura em resultados/<sim|real>/t3_t4_<ensaio>.png.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import calibracao as cal
import imu_io
import orientacao as ori
from imu_io import DADOS, LAB3


def estimar(reg, C: cal.Calibracao | None, guinada0=0.0):
    """Devolve dict nome -> quaterniões (N,4). A guinada inicial não se mede
    (o acelerómetro não a vê): vem da verdade, se houver, ou é 0."""
    r = C.aplicar(reg) if C else reg
    tr = cal.troços_parados(r)
    a0 = r.acc[tr[0][0]:tr[0][1]].mean(0) if tr else r.acc[:int(r.fs)].mean(0)
    rol0, arf0 = ori.inclinometro(a0)
    q0 = ori.de_euler(rol0, arf0, guinada0)
    out = {"acelerómetro": ori.q_inclinometro(r.acc),
           "giroscópio": ori.integrar_giro(r.gyro, r.t, q0)}
    if C:
        out["giroscópio sem calibrar"] = ori.integrar_giro(reg.gyro, reg.t, q0)
    return out, tr


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ficheiro", nargs="?")
    ap.add_argument("--sim", metavar="CENARIO", help="cenário em data/sim/")
    ap.add_argument("--cal", help="calibracao.json (por omissão: resultados/<sim|real>/)")
    ap.add_argument("--robo", help="registo do robô (data/..._robo.csv) do mesmo ensaio: dá a verdade")
    a = ap.parse_args()
    f = DADOS / "sim" / f"{a.sim}.csv" if a.sim else Path(a.ficheiro)
    pasta = LAB3 / "resultados" / ("sim" if a.sim else "real")
    pcal = Path(a.cal) if a.cal else pasta / "calibracao.json"
    C = cal.Calibracao.carregar(pcal) if pcal.exists() else None
    if C is None:
        print(f"(sem calibração: {pcal} não existe; corra t2_calibracao.py antes)")

    reg = imu_io.ler(f)
    ver = None
    if a.sim:
        import simular
        ver = simular.ler_verdade(f)["R"].as_quat(scalar_first=True)
    elif a.robo:
        sys.path.insert(0, str(Path(__file__).resolve().parent / "robo"))
        from verdade import verdade_para_imu
        ver, _, info = verdade_para_imu(reg, a.robo)
        print(f"Verdade do robô: desfasamento {1e3 * info['desfasamento_s']:.0f} ms, "
              f"{info['fora']} amostras fora do registo do robô")
    est, tr = estimar(reg, C, ori.para_euler(ver[0])[2] if ver is not None else 0.0)
    eu = {k: ori.para_euler(q) for k, q in est.items()}
    # perto de ±90° de arfagem o rolamento não está definido (bloqueio dos
    # ângulos de Euler): não se desenha, para não parecer um erro
    for e in eu.values():
        e[np.abs(e[:, 1]) > 80, 0] = np.nan

    fig, axs = plt.subplots(3 if ver is not None else 2, 1, figsize=(8, 7 if ver is not None else 5), sharex=True)
    cores = {"acelerómetro": "C1", "giroscópio": "C0", "giroscópio sem calibrar": "C7"}
    for j, nome in enumerate(("rolamento", "arfagem")):
        if ver is not None:
            ev = ori.para_euler(ver)
            ev[np.abs(ev[:, 1]) > 80, 0] = np.nan
            axs[j].plot(reg.t, ev[:, j], "k", lw=2.5, alpha=0.35, label="verdade")
        for k, e in eu.items():
            axs[j].plot(reg.t, e[:, j], color=cores[k], lw=0.9, label=k,
                        ls="--" if "sem" in k else "-")
        axs[j].set_ylabel(f"{nome} [°]"), axs[j].grid(alpha=0.3)
    axs[0].legend(fontsize=8, ncol=4)

    print(f"\n{f.name}: {reg.t[-1]:.0f} s, {len(tr)} paragens")
    if ver is not None:
        print("Erro de inclinação (ângulo entre a vertical estimada e a verdadeira):")
        print(f"  {'método':26s} {'RMS [°]':>8s} {'máx [°]':>8s} {'final [°]':>9s}")
        for k, q in est.items():
            e = ori.erro_inclinacao(q, ver)
            axs[2].plot(reg.t, e, color=cores[k], lw=0.9, ls="--" if "sem" in k else "-", label=k)
            print(f"  {k:26s} {np.sqrt(np.mean(e**2)):8.2f} {e.max():8.2f} {e[-1]:9.2f}")
        g = ori.angulo_entre(est["giroscópio"], ver)
        print(f"  giroscópio, erro total incluindo guinada: final {g[-1]:.2f}°")
        axs[2].set_ylabel("erro de inclinação [°]"), axs[2].grid(alpha=0.3)
        axs[2].set_yscale("log"), axs[2].set_ylim(1e-3, 90)
    else:
        print("Ângulo médio em cada paragem (comparar com o transferidor):")
        print(f"  {'t [s]':>7s} {'rol acc':>8s} {'arf acc':>8s} {'rol giro':>8s} {'arf giro':>8s}")
        for i0, i1 in tr:
            ea, eg = np.nanmean(eu["acelerómetro"][i0:i1], 0), np.nanmean(eu["giroscópio"][i0:i1], 0)
            print(f"  {reg.t[i0]:7.1f} {ea[0]:8.2f} {ea[1]:8.2f} {eg[0]:8.2f} {eg[1]:8.2f}")
    axs[-1].set_xlabel("t [s]")
    fig.tight_layout()
    out = pasta / f"t3_t4_{f.stem}.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=200)
    print(f"Figura: {out}")


if __name__ == "__main__":
    main()
