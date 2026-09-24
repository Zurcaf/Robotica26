"""
Teste de perturbações — Tarefa 2 do enunciado.

O enunciado pede que se considere a existência de perturbações em cada grau de
liberdade, dando como exemplo "o pulso do jogador a tremer". O modelo tem
exatamente esse dof, por isso o exemplo do enunciado traduz-se diretamente:
soma-se um ruído gaussiano ao binário comandado da junta escolhida e compara-se
com o swing nominal (sem ruído).

Uso (a partir de Lab1/):
    python src/perturbacoes.py              # N=20 swings com ruído no PULSO
    python src/perturbacoes.py --n 50       # mais repetições
    python src/perturbacoes.py --junta shoulder   # ruído no braço
    python src/perturbacoes.py --std 8      # desvio padrão do ruído [N·m]

Produz report/dispersao_perturbacao.png e imprime a estatística.
"""

import argparse
import os
import numpy as np

import matplotlib
matplotlib.use("Agg")          # sem janela: guarda direto para ficheiro
import matplotlib.pyplot as plt

from swing import load, run_swing

OUT = os.path.join("report", "dispersao_perturbacao.png")


def corre_lote(n, junta, std, seed0=0):
    """Corre n swings com ruído no binário de `junta` e devolve os resultados."""
    m, d = load()
    chave = f"{junta}_torque_std"
    nominal = run_swing(m, d, {})
    res = []
    for i in range(n):
        r = run_swing(m, d, {}, noise={chave: std}, seed=seed0 + i)
        res.append(r)
        print(f"  swing {i+1:3}/{n}  "
              + (f"carry {r['carry']:6.1f} m  lateral {r['land_y']:+6.2f} m  "
                 f"v_taco {r['v_head']:5.1f} m/s" if r["hit"] and not np.isnan(r["carry"])
                 else "FALHOU (não acertou na bola)"))
    return nominal, res


def estatistica(nome, valores):
    """Imprime média ± desvio padrão de uma lista, ignorando NaN."""
    v = np.array([x for x in valores if not np.isnan(x)])
    if v.size == 0:
        return np.nan, np.nan
    print(f"    {nome:<28} {v.mean():8.2f}  ±{v.std():6.2f}   "
          f"[min {v.min():.2f}, max {v.max():.2f}]")
    return v.mean(), v.std()


def main():
    ap = argparse.ArgumentParser(description="Perturbações no binário das juntas")
    ap.add_argument("--n", type=int, default=20, help="número de swings")
    ap.add_argument("--junta", default="wrist", choices=["wrist", "shoulder"])
    ap.add_argument("--std", type=float, default=5.0,
                    help="desvio padrão do ruído no binário [N.m]")
    args = ap.parse_args()

    print(f"{args.n} swings com ruído gaussiano de sigma = {args.std} N.m "
          f"no binário de '{args.junta}'\n")
    nominal, res = corre_lote(args.n, args.junta, args.std)

    acertou = [r for r in res if r["hit"] and not np.isnan(r["carry"])]
    print(f"\n  acertaram na bola: {len(acertou)}/{args.n}")

    print("\n  NOMINAL (sem ruído):")
    print(f"    v_taco {nominal['v_head']:.1f} m/s   carry {nominal['carry']:.1f} m   "
          f"lateral {nominal['land_y']:+.2f} m   lançamento {nominal['launch']:.1f} deg")

    print("\n  COM RUÍDO:")
    carry = [r["carry"] for r in acertou]
    lat = [r["land_y"] for r in acertou]
    estatistica("carry [m]", carry)
    estatistica("desvio lateral [m]", lat)
    estatistica("v cabeça do taco [m/s]", [r["v_head"] for r in acertou])
    estatistica("v bola [m/s]", [r["v_ball"] for r in acertou])
    estatistica("ângulo de lançamento [deg]", [r["launch"] for r in acertou])

    # ---- gráfico de dispersão dos pontos de aterragem
    os.makedirs("report", exist_ok=True)
    fig, ax = plt.subplots(figsize=(7.5, 5.5))
    ax.scatter(lat, carry, s=45, alpha=0.75, edgecolor="k", linewidth=0.5,
               label=f"com ruído (n={len(acertou)})", zorder=3)
    ax.scatter([nominal["land_y"]], [nominal["carry"]], s=190, marker="*",
               color="crimson", edgecolor="k", linewidth=0.6,
               label="nominal (sem ruído)", zorder=4)
    if len(carry) > 1:
        ax.errorbar(np.mean(lat), np.mean(carry),
                    xerr=np.std(lat), yerr=np.std(carry),
                    fmt="o", color="tab:orange", capsize=4, markersize=7,
                    label="média ± 1 desvio padrão", zorder=3)
    ax.axvline(0.0, color="grey", lw=0.8, ls="--", zorder=1)
    ax.set_xlabel("desvio lateral no ponto de aterragem [m]   "
                  "(+ = para a direita do alvo)")
    ax.set_ylabel("carry [m]")
    ax.set_title(f"Dispersão com ruído no binário de '{args.junta}'\n"
                 f"sigma = {args.std} N.m, {len(acertou)}/{args.n} acertaram na bola")
    ax.grid(alpha=0.3, zorder=0)
    ax.legend()
    fig.tight_layout()
    fig.savefig(OUT, dpi=130)
    print(f"\n  gráfico guardado em {OUT}")


if __name__ == "__main__":
    main()
