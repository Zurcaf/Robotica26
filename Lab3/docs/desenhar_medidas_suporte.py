"""Desenho das medidas a tirar no laboratório para fechar o suporte da IMU.

    .venv/bin/python docs/desenhar_medidas_suporte.py     (a partir de Lab3/)
Gera docs/medidas_suporte.png. Os tamanhos no desenho são ilustrativos.
"""
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch, Rectangle

AZUL = "#1f6fd1"
CINZA = "#d9d8d4"
ESCURO = "#2b2b2b"


def cota(ax, p0, p1, letra, off=(0, 0), lado="centro"):
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle="<|-|>", mutation_scale=12, color=AZUL, lw=1.6))
    mx, my = (p0[0] + p1[0]) / 2 + off[0], (p0[1] + p1[1]) / 2 + off[1]
    ax.text(mx, my, letra, color=AZUL, fontsize=15, fontweight="bold", ha="center", va="center",
            bbox=dict(fc="white", ec="none", pad=1))


def painel(ax, titulo):
    ax.set_title(titulo, fontsize=12, fontweight="bold")
    ax.set_aspect("equal")
    ax.axis("off")


fig, axs = plt.subplots(2, 2, figsize=(15, 11))

# --- 1. parafuso do dedo, de lado
ax = axs[0, 0]
painel(ax, "1. Parafuso do dedo (de lado)")
ax.add_patch(Rectangle((0, 0), 4.5, 2.5, fc=ESCURO))           # cabeça
ax.add_patch(Rectangle((1.0, -6), 2.5, 6, fc="#777"))           # rosca
for y in range(-6, 0):
    ax.plot([1.0, 3.5], [y + 0.2, y + 0.7], color="#bbb", lw=1)
cota(ax, (1.0, -7), (3.5, -7), "A", off=(0, -0.7))
cota(ax, (4.6, 0), (4.6, -6), "B", off=(0.7, 0))
cota(ax, (0, 3.2), (4.5, 3.2), "C", off=(0, 0.7))
cota(ax, (-0.6, 0), (-0.6, 2.5), "D", off=(-0.7, 0))
ax.set_xlim(-3, 9); ax.set_ylim(-9, 5)

# --- 2. carro da pinça sem o dedo, visto de baixo
ax = axs[0, 1]
painel(ax, "2. Carro sem o dedo (visto de baixo) e pinça parada")
ax.add_patch(Circle((0, 0), 38, fc=CINZA, ec="#888"))
ax.text(0, 30, "corpo da pinça", ha="center", color="#555")
ax.add_patch(Rectangle((-10, -20), 20, 15.3, fc="#9a9a9a", ec="#555"))      # face do carro
for x in (-6, 6):
    ax.add_patch(Circle((x, -20 + 7.7), 1.5, fc="white", ec=ESCURO))
ax.add_patch(Rectangle((-10, 3), 20, 15.3, fc=ESCURO))                      # dedo que ficou
ax.text(0, 11, "dedo que ficou", ha="center", color="white", fontsize=9)
cota(ax, (-6, -10), (6, -10), "E", off=(0, 2.5))
cota(ax, (-10, -24), (10, -24), "F", off=(0, -2.5))
cota(ax, (13, -20), (13, -4.7), "G", off=(2.5, 0))
cota(ax, (-13, -20), (-13, -12.3), "H", off=(-2.5, 0))
cota(ax, (17, -4.7), (17, 3), "I", off=(2.8, 0))
ax.text(0, -33, "H: dos furos à aresta do carro do lado de fora\n"
        "I: do carro ao dedo que ficou, com a pinça parada", ha="center", fontsize=9, color="#444")
ax.set_xlim(-42, 42); ax.set_ylim(-42, 40)

# --- 3. dedo que tiraste, de lado (corte pelo furo)
ax = axs[1, 0]
painel(ax, "3. Dedo que tiraste (de lado, corte pelo furo) e furo do carro")
ax.add_patch(Rectangle((0, 0), 15.3, 7, fc=ESCURO))                         # base
ax.add_patch(Rectangle((11.1, -16.5), 4.2, 16.5, fc=ESCURO))                # lâmina
ax.add_patch(Rectangle((7.7 - 2.7, 0.9), 5.4, 6.1, fc="white", ec=ESCURO))  # caixa da cabeça
ax.add_patch(Rectangle((7.7 - 1.45, 0), 2.9, 0.9, fc="white", ec=ESCURO))   # passagem
cota(ax, (-1.2, 0), (-1.2, 7), "J", off=(-1.2, 0))
cota(ax, (4.0, 0), (4.0, 0.9), "K", off=(-1.3, 0))
cota(ax, (0, 8.2), (7.7, 8.2), "L", off=(0, 1.0))
cota(ax, (0, -18), (15.3, -18), "M", off=(0, -1.2))
cota(ax, (16.5, 0), (16.5, -16.5), "N", off=(1.3, 0))
ax.add_patch(Rectangle((2, 10.5), 10, 4, fc="#9a9a9a", ec="#555"))          # carro (corte)
ax.add_patch(Rectangle((7.7 - 1.25, 10.5), 2.5, 3, fc="white", ec=ESCURO))  # furo roscado
ax.text(13, 12.5, "carro", fontsize=9, color="#444", va="center")
cota(ax, (5.5, 10.5), (5.5, 13.5), "O", off=(-1.3, 0))
ax.text(7.7, -22, "K: plástico entre a cabeça e o carro   ·   O: fundo da rosca no carro\n"
        "(se não der para medir O: enfia o parafuso até ao fim e mede o que fica de fora)",
        ha="center", fontsize=9, color="#444")
ax.set_xlim(-5, 22); ax.set_ylim(-25, 16)

# --- 4. kit da IMU
ax = axs[1, 1]
painel(ax, "4. Kit da IMU (de frente e de lado)")
ax.add_patch(Rectangle((0, 0), 76, 59.3, fc="#e8eef6", ec="#555"))          # acrílico
ax.add_patch(Rectangle((4, 3), 68.6, 53.3, fc="#2f6f5e", ec="#1d4a3e"))     # placas
ax.add_patch(Rectangle((40, 30), 6, 6, fc=ESCURO))                          # chip
ax.text(43, 38.5, "chip IMU", ha="center", fontsize=9, color="white")
ax.add_patch(Rectangle((-6, 40), 10, 9, fc="#888"))                         # USB
ax.text(-1, 51, "USB", ha="center", fontsize=8)
cota(ax, (0, -4), (76, -4), "P", off=(0, -3))
cota(ax, (80, 0), (80, 59.3), "Q", off=(3.5, 0))
cota(ax, (0, 33), (40, 33), "R", off=(0, 2.5))
cota(ax, (43, 0), (43, 30), "S", off=(3, 0))
# vista de lado
ax.add_patch(Rectangle((96, 0), 3, 59.3, fc="#e8eef6", ec="#555"))
ax.add_patch(Rectangle((99, 3), 25, 53.3, fc="#2f6f5e", ec="#1d4a3e"))
ax.add_patch(Rectangle((101, 30), 2, 6, fc=ESCURO))
cota(ax, (96, -4), (124, -4), "T", off=(0, -3))
cota(ax, (96, 40), (101, 40), "U", off=(0, 3))
ax.text(110, 63, "de lado", ha="center", fontsize=9)
ax.text(55, -16, "R, S: do centro do chip às arestas do ACRÍLICO (as de fora)\n"
        "T: do acrílico à ponta mais alta das fichas   ·   U: do acrílico ao chip",
        ha="center", fontsize=9, color="#444")
ax.set_xlim(-10, 130); ax.set_ylim(-22, 68)

fig.suptitle("Medidas para o suporte da IMU (paquímetro, em mm)", fontsize=16, fontweight="bold")
fig.text(0.5, 0.01, "Tamanhos do desenho ilustrativos. Fotografa cada peça com a régua encostada.",
         ha="center", color="#555")
fig.tight_layout(rect=(0, 0.03, 1, 0.96))
out = Path(__file__).with_name("medidas_suporte.png")
fig.savefig(out, dpi=110)
print(out)
