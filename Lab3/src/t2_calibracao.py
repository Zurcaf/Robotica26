"""Tarefa 2: calibração do acelerómetro e do giroscópio.

    .venv/bin/python src/t2_calibracao.py --sim          # dados simulados
    .venv/bin/python src/t2_calibracao.py \\
        --seis data/..._B_seis.csv --acaso data/..._B2_acaso.csv \\
        --parado data/..._A_parado.csv --rotacao data/..._C_rotacao.csv

Cada ficheiro é opcional. Escreve em resultados/<sim|real>/:
    calibracao.json      parâmetros (usados por t3_t4_orientacao.py)
    t2_modulo.png        |a| em cada posição, antes e depois (figura 1)
    t2_allan.png         desvio de Allan (figura 4)
    t2_temperatura.png   desvio do giroscópio contra a temperatura (figura 3)
    t2_rotacoes.png      ângulo integrado contra o nominal (ensaio C)
Com --sim compara cada parâmetro com o valor verdadeiro do simulador.
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
from imu_io import DADOS, LAB3

EIXOS = "xyz"


def tabela(titulo, linhas, cab):
    print(f"\n{titulo}")
    print("  " + " | ".join(f"{c:>12s}" for c in cab))
    for l in linhas:
        print("  " + " | ".join(f"{v:>12.5g}" if isinstance(v, (float, np.floating)) else f"{str(v):>12s}"
                                for v in l))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", action="store_true", help="usa data/sim/ (gerar antes com simular.py)")
    ap.add_argument("--seis"), ap.add_argument("--acaso")
    ap.add_argument("--parado"), ap.add_argument("--rotacao")
    ap.add_argument("--saida", default=None)
    a = ap.parse_args()
    if a.sim:
        s = DADOS / "sim"
        a.seis, a.acaso = a.seis or s / "seis_posicoes.csv", a.acaso or s / "posicoes_acaso.csv"
        a.parado, a.rotacao = a.parado or s / "parado.csv", a.rotacao or s / "rotacao_conhecida.csv"
    saida = Path(a.saida) if a.saida else LAB3 / "resultados" / ("sim" if a.sim else "real")
    saida.mkdir(parents=True, exist_ok=True)

    verdade = None
    if a.sim:
        import simular
        verdade = simular.ler_verdade(a.seis).get("par")

    C = cal.Calibracao()
    res = {}

    # ---------------------------------------------------------------- acelerómetro
    fig, ax = plt.subplots(figsize=(7, 3.2))
    for nome, f, cor in (("seis posições", a.seis, "C0"), ("posições ao acaso", a.acaso, "C1")):
        if not f:
            continue
        r = imu_io.ler(f)
        tr = cal.troços_parados(r)
        am, gm, _ = cal.medias(r, tr)
        print(f"{Path(f).name}: {len(tr)} troços parados")
        if nome == "seis posições":
            s6 = cal.seis_posicoes(am)
            res["seis"] = s6
            res["seis_elip"] = cal.elipsoide(am, desalinhamento=False)
        else:
            res["acaso"] = cal.elipsoide(am, desalinhamento=True)
        res.setdefault("medias_acc", []).append((nome, am))
        res.setdefault("medias_gyro_paradas", []).append(gm)

    # escolha: posições ao acaso com desalinhamento, se houver; senão seis posições
    if "acaso" in res:
        C.bias_a, C.M_a = res["acaso"]["bias"].tolist(), res["acaso"]["M"].tolist()
        C.notas["acelerometro"] = "posições ao acaso (b, s, D)"
    elif "seis" in res:
        C.bias_a = res["seis"]["bias"].tolist()
        C.M_a = np.diag(res["seis"]["escala"]).tolist()
        C.notas["acelerometro"] = "seis posições (b, s)"

    if "medias_acc" in res:
        for i, (nome, am) in enumerate(res["medias_acc"]):
            x = np.arange(len(am)) + 0.15 * i
            ax.plot(x, np.linalg.norm(am, axis=1), "o", mfc="none", color=f"C{i}", label=f"{nome}, antes")
            ax.plot(x, np.linalg.norm(C.corrigir_acc(am), axis=1), "o", color=f"C{i}", label=f"{nome}, depois")
        ax.axhline(1, color="k", lw=0.8)
        ax.set_xlabel("posição parada"), ax.set_ylabel("|a| [g]")
        ax.legend(fontsize=8, ncol=2), ax.grid(alpha=0.3)
        fig.tight_layout(), fig.savefig(saida / "t2_modulo.png", dpi=200)
    plt.close(fig)

    linhas = []
    for e in range(3):
        l = [EIXOS[e]]
        if "seis" in res:
            l += [1e3 * res["seis"]["bias"][e], res["seis"]["escala"][e]]
        if "acaso" in res:
            l += [1e3 * res["acaso"]["bias"][e], res["acaso"]["escala"][e]]
        if verdade:
            l += [1e3 * verdade.bias_a[e], verdade.escala_a[e]]
        linhas.append(l)
    cab = ["eixo"] + (["b seis [mg]", "s seis"] if "seis" in res else []) + \
          (["b acaso [mg]", "s acaso"] if "acaso" in res else []) + (["b verd [mg]", "s verd"] if verdade else [])
    tabela("Acelerómetro", linhas, cab)
    if "acaso" in res:
        print(f"  desalinhamento (acaso) [mrad]: {np.round(1e3*res['acaso']['desal'], 2)}"
              + (f"   verdade: {np.round(1e3*np.array(verdade.desal_a), 2)}" if verdade else ""))
        print(f"  resíduo |a|−1 depois do ajuste: {1e3*res['acaso']['residuo_rms']:.2f} mg RMS")

    # ---------------------------------------------------------------- giroscópio parado
    if a.parado:
        r = imu_io.ler(a.parado)
        tr = cal.troços_parados(r, dur_min=10)
        b, dp = cal.bias_giro(r, tr)
        C.bias_g = b.tolist()
        tt = cal.giro_temperatura(r, tr)
        C.temp_g, C.temp_ref = np.asarray(tt["declive"]).tolist(), tt["temp_ref"]
        C.bias_g = (np.asarray(tt.get("ordenada", b))).tolist()
        tau, ad = cal.allan(r.gyro, r.fs)
        tau_a, ad_a = cal.allan(r.acc, r.fs)
        n_w, bi_w = cal.ruido_de_allan(tau, ad)
        n_a, bi_a = cal.ruido_de_allan(tau_a, ad_a)
        print(f"\n{Path(a.parado).name}: {r.t[-1]:.0f} s, {r.fs:.0f} Hz, "
              f"temperatura {r.temp.min():.1f} a {r.temp.max():.1f} °C")
        linhas = [[EIXOS[e], b[e], tt["declive"][e], n_w[e], bi_w[e], 1e3 * n_a[e]]
                  + ([verdade.bias_g[e] + verdade.temp_g[e] * (tt["temp_ref"] - 25), verdade.temp_g[e],
                      verdade.ruido_g] if verdade else []) for e in range(3)]
        tabela("Giroscópio parado", linhas,
               ["eixo", "desvio [dps]", "dps/°C", "ruído dps/√Hz", "instab. dps", "acc mg/√Hz"]
               + (["desvio verd", "dps/°C verd", "ruído verd"] if verdade else []))

        fig, axs = plt.subplots(1, 2, figsize=(8, 3.2))
        for e in range(3):
            axs[0].loglog(tau, ad[:, e], label=f"giro {EIXOS[e]}")
            axs[1].loglog(tau_a, 1e3 * ad_a[:, e], label=f"acc {EIXOS[e]}")
        axs[0].set_ylabel("desvio de Allan [dps]"), axs[1].set_ylabel("desvio de Allan [mg]")
        for x in axs:
            x.set_xlabel("τ [s]"), x.grid(alpha=0.3, which="both"), x.legend(fontsize=8)
        fig.tight_layout(), fig.savefig(saida / "t2_allan.png", dpi=200), plt.close(fig)

        fig, ax = plt.subplots(figsize=(5, 3.2))
        for e in range(3):
            ax.plot(tt["T"], tt["W"][:, e], ".", label=f"{EIXOS[e]}: {tt['declive'][e]*1e3:+.0f} mdps/°C")
        ax.set_xlabel("temperatura do chip [°C]"), ax.set_ylabel("desvio do giroscópio [dps]")
        ax.legend(fontsize=8), ax.grid(alpha=0.3)
        fig.tight_layout(), fig.savefig(saida / "t2_temperatura.png", dpi=200), plt.close(fig)
    elif res.get("medias_gyro_paradas"):
        b = np.vstack(res["medias_gyro_paradas"]).mean(0)
        C.bias_g = b.tolist()
        print(f"\nDesvio do giroscópio (das paragens): {np.round(b, 3)} dps")

    # ---------------------------------------------------------------- rotações conhecidas
    if a.rotacao:
        r = imu_io.ler(a.rotacao)
        tr = cal.troços_parados(r)
        b_local, _ = cal.bias_giro(r, tr)        # desvio do próprio ensaio (mesma temperatura)
        rc = cal.rotacoes_conhecidas(r, tr, b_local)
        C.M_g = np.where(np.isnan(rc["M"]), np.eye(3), rc["M"]).tolist()
        C.notas["giroscopio"] = f"escala de {len(rc['rotacoes'])} rotações conhecidas"
        linhas = [[EIXOS[e], rc["escala"][e]] + ([verdade.escala_g[e]] if verdade else []) for e in range(3)]
        tabela("Escala do giroscópio (rotações conhecidas)", linhas,
               ["eixo", "escala"] + (["verdade"] if verdade else []))
        print("  M_g estimada:\n" + "\n".join("    " + " ".join(f"{v:+.4f}" for v in row) for row in rc["M"]))

        fig, ax = plt.subplots(figsize=(5, 3.2))
        for l in rc["rotacoes"]:
            ax.plot(l["nominal"], l["medido"][l["eixo"]] - l["nominal"], "o", color=f"C{l['eixo']}")
        for e in range(3):
            ax.plot([], [], "o", color=f"C{e}", label=f"em torno de {EIXOS[e]}")
        ax.axhline(0, color="k", lw=0.8)
        ax.set_xlabel("ângulo nominal [°]"), ax.set_ylabel("integrado − nominal [°]")
        ax.legend(fontsize=8), ax.grid(alpha=0.3)
        fig.tight_layout(), fig.savefig(saida / "t2_rotacoes.png", dpi=200), plt.close(fig)

    C.guardar(saida / "calibracao.json")
    print(f"\nCalibração guardada em {saida / 'calibracao.json'}")
    return C, res, verdade


if __name__ == "__main__":
    main()
