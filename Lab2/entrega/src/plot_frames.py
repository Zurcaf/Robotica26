"""
Tarefa 1 — Figura dos referenciais D-H (modificado) do Lite 6, com q = 0.

Uso (a partir de Lab2/):
    python src/plot_frames.py        # -> report/referenciais.png

Vista lateral (plano x0-z0; y0 entra na folha). Em q = 0 todos os eixos x_i e
z_i estão neste plano ou são perpendiculares a ele, por isso a vista 2D mostra
tudo: z_i (eixo da junta i) a azul, x_i a vermelho; um eixo perpendicular à
folha desenha-se como um círculo com ponto (sai) ou com cruz (entra). Os
referenciais {1},{2} e {4},{5} têm a mesma origem (d2 = a1 = 0, d5 = a4 = 0):
{2} e {5} estão desenhados ao lado, ligados à origem por um tracejado.
"""
import os

import matplotlib.pyplot as plt
import numpy as np

import config as C
from kinematics import fk, tool_transform

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "report", "referenciais.png")
CX, CZ, ELO, TXT = "#e34948", "#2a78d6", "#8a8984", "#0b0b0b"
L = 45.0                                   # comprimento das setas [mm]
DESVIO = {2: (-125.0, 0.0), 5: (130.0, 55.0)}   # onde desenhar os referenciais coincidentes


def eixo(ax, o, v, cor, nome):
    """Seta no plano, ou círculo com ponto/cruz se o eixo for perpendicular à folha (v em 3D)."""
    if abs(v[1]) > 0.5:                    # ao longo de y0: +y entra na folha
        ax.add_patch(plt.Circle(o, 9, fc="white", ec=cor, lw=1.6, zorder=6))
        if v[1] > 0:
            d = 6.4
            ax.plot([o[0] - d, o[0] + d], [o[1] - d, o[1] + d], color=cor, lw=1.4, zorder=7)
            ax.plot([o[0] - d, o[0] + d], [o[1] + d, o[1] - d], color=cor, lw=1.4, zorder=7)
        else:
            ax.add_patch(plt.Circle(o, 2.6, fc=cor, ec=cor, zorder=7))
        ax.text(o[0] - 13, o[1] - 20, nome, color=TXT, fontsize=9, ha="right", va="center")
        return
    fim = (o[0] + L * v[0], o[1] + L * v[2])
    ax.annotate("", xy=fim, xytext=o, arrowprops=dict(arrowstyle="-|>", color=cor, lw=1.8, shrinkA=0, shrinkB=0),
                zorder=5)
    if abs(v[0]) > 0.5:                    # seta horizontal: nome por cima da ponta
        ax.text(fim[0], fim[1] + 12, nome, color=TXT, fontsize=9, ha="center", va="center")
    elif v[2] > 0:                         # para cima: nome à direita da ponta
        ax.text(fim[0] + 9, fim[1], nome, color=TXT, fontsize=9, ha="left", va="center")
    else:                                  # para baixo: nome à direita, a meio da seta
        ax.text(o[0] + 9, o[1] - 0.6 * L, nome, color=TXT, fontsize=9, ha="left", va="center")


def main():
    Ts = fk(np.zeros(6), all_frames=True)
    tcp = (Ts[6] @ tool_transform(C.GRIPPER_TCP_Z))[:3, 3]
    O = [T[:3, 3] for T in Ts]
    fig, ax = plt.subplots(figsize=(6.6, 7.0))

    # elos: base, braço, desvio do antebraço, antebraço, pulso, pinça
    pts = np.array([O[0], O[1], O[3], [O[4][0], 0, O[3][2]], O[4], O[6], tcp])
    ax.plot(pts[:-1, 0], pts[:-1, 2], color=ELO, lw=7, solid_capstyle="round", zorder=1)
    ax.plot(pts[-2:, 0], pts[-2:, 2], color=ELO, lw=3.5, solid_capstyle="round", zorder=1)
    ax.plot([-170, 330], [0, 0], color=TXT, lw=1)
    ax.fill_between([-170, 330], -14, 0, color=ELO, alpha=0.25, lw=0)

    # cotas (tabela D-H)
    for (a, b), txt, lado in [((O[0], O[1]), r"$d_1 = 243{,}3$", -1), ((O[1], O[3]), r"$a_2 = 200$", -1),
                              ((O[4], O[6]), r"$d_6 = 61{,}5$", -1), ((O[6], tcp), r"pinça $83{,}6$", -1)]:
        m = 0.5 * (a + b)
        ax.text(m[0] + 14 * lado, m[2], txt, fontsize=9, color=TXT, ha="right", va="center")
    ax.text(O[4][0] + 14, O[3][2] + 4, r"$a_3 = 87$", fontsize=9, color=TXT, ha="left", va="center")
    ax.text(O[4][0] + 14, 0.5 * (O[3][2] + O[4][2]) + 30, r"$d_4 = 227{,}6$", fontsize=9, color=TXT, ha="left", va="center")

    for i, T in enumerate(Ts):
        dx, dz = DESVIO.get(i, (0.0, 0.0))
        o = (T[0, 3] + dx, T[2, 3] + dz)
        if i in DESVIO:
            ax.plot([T[0, 3], o[0]], [T[2, 3], o[1]], color=ELO, lw=0.9, ls=(0, (3, 3)), zorder=2)
        eixo(ax, o, T[:3, 0], CX, f"$x_{i}$")
        eixo(ax, o, T[:3, 2], CZ, f"$z_{i}$")
        ax.add_patch(plt.Circle(o, 4, fc=TXT, ec="white", lw=0.8, zorder=8))
        coincide = {1: r"$\{1\}$", 2: r"$\{2\}$", 4: r"$\{4\}$", 5: r"$\{5\}$"}
        nome = coincide.get(i, rf"$\{{{i}\}}$")
        pos = {0: (-16, 18), 1: (-16, -16), 2: (0, -36), 3: (-16, 20), 4: (-14, 14), 5: (0, -36), 6: (-16, -4)}[i]
        ax.text(o[0] + pos[0], o[1] + pos[1], nome, fontsize=10.5, fontweight="bold", color=TXT, ha="right" if pos[0] else "center",
                va="center")
    ax.add_patch(plt.Circle((tcp[0], tcp[2]), 5, fc="white", ec=TXT, lw=1.4, zorder=8))
    ax.text(tcp[0] + 14, tcp[2], "TCP (ponta da pinça)", fontsize=9, color=TXT, va="center")

    # legenda
    ax.plot([], [], color=CZ, lw=1.8, label=r"$z_i$: eixo da junta $i$")
    ax.plot([], [], color=CX, lw=1.8, label=r"$x_i$: normal comum a $z_i$ e $z_{i+1}$")
    ax.plot([], [], color="none", label=r"$\odot$ sai da folha,  $\otimes$ entra ($+y_0$)")
    ax.legend(loc="upper right", fontsize=8.5, frameon=False, handlelength=1.6)
    ax.set_aspect("equal")
    ax.set_xlim(-175, 335)
    ax.set_ylim(-20, 500)
    ax.set_xlabel(r"$x_0$ [mm]")
    ax.set_ylabel(r"$z_0$ [mm]")
    ax.spines[["top", "right"]].set_visible(False)
    fig.tight_layout()
    fig.savefig(OUT, dpi=170)
    print(f"  figura em {os.path.normpath(OUT)}")


if __name__ == "__main__":
    main()
