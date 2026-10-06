"""
Teste de perturbações — Tarefa 2 do enunciado.

O enunciado pede que se considere a existência de perturbações em cada grau de
liberdade, dando como exemplo "o pulso do jogador a tremer". Soma-se ruído
gaussiano ao binário comandado de UMA junta de cada vez e compara-se com o
swing nominal (sem ruído). O ruído é mantido 1/10 s (banda do tremor
fisiológico reforçado, 8-12 Hz) e só atua depois de o taco sair do endereço
(ver t_noise_on em swing.py): com o taco encostado à bola, 10 N.m de
desequilíbrio no pulso deslocavam-na antes de o swing começar.

Uso (a partir de Lab1/):
    python src/perturbacoes.py                      # todas as juntas, N=20, sigma=10 N.m
    python src/perturbacoes.py --junta wrist --n 50
    python src/perturbacoes.py --std 4              # sigma mais pequeno
    python src/perturbacoes.py --so-grafico         # redesenha a partir dos CSV
    python src/perturbacoes.py --modelo models/golfer_v13.xml

Produz report/perturbacoes_<junta>.csv (um swing por linha, o nominal com
seed = -1), report/dispersao_perturbacao.png e a mesma figura em
report/ROB_LAB1_Report/Photos/<v>_dispersao.png, e imprime a estatística e a
linha LaTeX de cada junta.
"""

import argparse
import csv
import os
import numpy as np

import matplotlib
matplotlib.use("Agg")          # sem janela: guarda direto para ficheiro
import matplotlib.pyplot as plt

from swing import load, run_swing, config, MODEL

REPORT = "report"
PHOTOS = os.path.join("report", "ROB_LAB1_Report", "Photos")
CAMPOS = ["seed", "hit", "v_head", "v_ball", "launch", "side", "carry", "land_y",
          "apogeu", "spin_rpm"]


def csv_path(junta):
    return os.path.join(REPORT, f"perturbacoes_{junta}.csv")


def _linha(seed, r, joints):
    d = dict(seed=seed, hit=int(r["hit"]))
    for k in CAMPOS[2:]:
        d[k] = round(float(r[k]), 3)
    for j, q in zip(joints, r["q_impact"]):
        d[f"q_{j}"] = round(float(q), 3)
    return d


def corre_lote(m, d, junta, n, std, seed0=0):
    """Corre n swings com ruído no binário de `junta` e grava o CSV."""
    joints = config(m)["joints"]
    nominal = run_swing(m, d, {})
    linhas = [_linha(-1, nominal, joints)]
    for i in range(n):
        r = run_swing(m, d, {}, noise={f"{junta}_torque_std": std}, seed=seed0 + i)
        linhas.append(_linha(seed0 + i, r, joints))
        ok = r["hit"] and not np.isnan(r["carry"])
        print(f"  {junta:<9} swing {i+1:3}/{n}  "
              + (f"carry {r['carry']:6.1f} m  lateral {r['land_y']:+6.2f} m  "
                 f"v_taco {r['v_head']:5.1f} m/s  lanç. {r['launch']:5.1f} deg"
                 if ok else "FALHOU (não acertou na bola)"))
    os.makedirs(REPORT, exist_ok=True)
    with open(csv_path(junta), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(linhas[0].keys()))
        w.writeheader()
        w.writerows(linhas)
    return le_lote(junta)


def le_lote(junta):
    """Lê o CSV de uma junta -> (nominal, lista de swings com ruído)."""
    with open(csv_path(junta), newline="") as f:
        rows = [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]
    nominal = next(r for r in rows if r["seed"] == -1)
    return nominal, [r for r in rows if r["seed"] != -1]


def validos(res):
    return [r for r in res if r["hit"] and not np.isnan(r["carry"])]


def estatistica(nome, valores):
    """Imprime média ± desvio padrão de uma lista, ignorando NaN."""
    v = np.array([x for x in valores if not np.isnan(x)])
    if v.size == 0:
        return np.nan, np.nan
    print(f"    {nome:<28} {v.mean():8.2f}  ±{v.std():6.2f}   "
          f"[min {v.min():.2f}, max {v.max():.2f}]")
    return v.mean(), v.std()


def resumo(junta, label, tau_max, std, nominal, res):
    ok = validos(res)
    print(f"\n  === {label} ({junta}): sigma = {std} N.m = {100*std/tau_max:.0f} % de tau_max, "
          f"{len(ok)}/{len(res)} acertaram ===")
    print(f"    nominal: v_taco {nominal['v_head']:.1f}  carry {nominal['carry']:.1f} m  "
          f"lateral {nominal['land_y']:+.2f} m  lançamento {nominal['launch']:.1f} deg")
    e = {}
    for k, nome in [("v_head", "v cabeça do taco [m/s]"), ("v_ball", "v bola [m/s]"),
                    ("launch", "ângulo de lançamento [deg]"), ("carry", "carry [m]"),
                    ("land_y", "desvio lateral [m]"), ("apogeu", "apogeu [m]")]:
        e[k] = estatistica(nome, [r[k] for r in ok])
    topadas = sum(1 for r in ok if r["launch"] < 10.0)
    print(f"    bolas 'topadas' (lançamento < 10 deg): {topadas}")

    def pm(k, nd):
        mu, sd = e[k]
        return f"${mu:.{nd}f} \\pm {sd:.{nd}f}$".replace(".", "{,}")
    print("    LaTeX: \\texttt{" + junta + "} & " + f"{100*std/tau_max:.0f}\\,\\% & "
          f"{len(ok)}/{len(res)} & {pm('v_head', 1)} & {pm('launch', 1)} & "
          f"{pm('carry', 0)} & {pm('land_y', 1)} \\\\")
    return e


def grafico(juntas, labels, taus, std, dados, caminhos):
    """Um painel por junta: pontos de aterragem (lateral vs carry)."""
    n = len(juntas)
    fig, axes = plt.subplots(1, n, figsize=(4.6 * n + 0.6, 4.4), sharey=True)
    axes = np.atleast_1d(axes)
    for ax, junta, label, tau in zip(axes, juntas, labels, taus):
        nominal, res = dados[junta]
        ok = validos(res)
        lat = [r["land_y"] for r in ok]
        car = [r["carry"] for r in ok]
        ax.scatter(lat, car, s=40, alpha=0.75, color="#2a78d6", edgecolor="k",
                   linewidth=0.5, label=f"com ruído ({len(ok)}/{len(res)})", zorder=3)
        ax.scatter([nominal["land_y"]], [nominal["carry"]], s=190, marker="*",
                   color="#e34948", edgecolor="k", linewidth=0.6,
                   label="nominal", zorder=4)
        if len(ok) > 1:
            ax.errorbar(np.mean(lat), np.mean(car), xerr=np.std(lat), yerr=np.std(car),
                        fmt="o", color="#eb6834", capsize=4, markersize=6,
                        label="média ± 1 d.p.", zorder=3)
        ax.axvline(0.0, color="grey", lw=0.8, ls="--", zorder=1)
        ax.set_title(f"{label}  (σ = {std:g} N·m = {100*std/tau:.0f} % de τ$_{{\\max}}$)",
                     fontsize=10)
        ax.set_xlabel("desvio lateral na aterragem [m]  (+ = direita)")
        ax.grid(alpha=0.3, zorder=0)
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
    axes[0].set_ylabel("carry [m]")
    axes[0].legend(frameon=False, fontsize=9)
    fig.tight_layout()
    for c in caminhos:
        os.makedirs(os.path.dirname(c), exist_ok=True)
        fig.savefig(c, dpi=130)
        print(f"  gráfico guardado em {c}")
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description="Perturbações no binário das juntas")
    ap.add_argument("--modelo", default=MODEL)
    ap.add_argument("--junta", default="all", help="nome da junta ou 'all'")
    ap.add_argument("--n", type=int, default=20, help="número de swings por junta")
    ap.add_argument("--std", type=float, default=10.0,
                    help="desvio padrão do ruído no binário [N.m]")
    ap.add_argument("--seed0", type=int, default=0)
    ap.add_argument("--so-grafico", action="store_true",
                    help="não simula: lê os CSV existentes e redesenha")
    args = ap.parse_args()

    m, d = load(args.modelo)
    cfg = config(m)
    versao = "v17" if "torso" in cfg["joints"] else "v13"
    juntas = cfg["joints"] if args.junta == "all" else [args.junta]
    for j in juntas:
        if j not in cfg["joints"]:
            raise SystemExit(f"junta '{j}' não existe neste modelo; opções: {cfg['joints']}")
    labels = [cfg["labels"][cfg["joints"].index(j)] for j in juntas]
    taus = [m.actuator_ctrlrange[m.actuator(cfg["motors"][cfg["joints"].index(j)]).id, 1]
            for j in juntas]

    dados = {}
    for j in juntas:
        if args.so_grafico:
            dados[j] = le_lote(j)
        else:
            print(f"\n{args.n} swings com ruído gaussiano de sigma = {args.std} N.m "
                  f"no binário de '{j}'")
            dados[j] = corre_lote(m, d, j, args.n, args.std, args.seed0)

    for j, lab, tau in zip(juntas, labels, taus):
        resumo(j, lab, tau, args.std, *dados[j])

    grafico(juntas, labels, taus, args.std, dados,
            [os.path.join(REPORT, "dispersao_perturbacao.png"),
             os.path.join(PHOTOS, f"{versao}_dispersao.png")])


if __name__ == "__main__":
    main()
