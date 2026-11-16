"""Desenho das medidas do kit (placa da IMU, cabo Grove, Arduino Uno) para o suporte sem acrílico.

    .venv/bin/python docs/desenhar_medidas_kit.py     (a partir de Lab3/)
Gera docs/medidas_kit.png. Os tamanhos no desenho são ilustrativos.
"""
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch, Rectangle

AZUL = "#1f6fd1"
VERDE = "#2f6f5e"
ESCURO = "#2b2b2b"


def cota(ax, p0, p1, letra, off=(0, 0)):
    ax.add_patch(FancyArrowPatch(p0, p1, arrowstyle="<|-|>", mutation_scale=12, color=AZUL, lw=1.6))
    ax.text((p0[0] + p1[0]) / 2 + off[0], (p0[1] + p1[1]) / 2 + off[1], letra, color=AZUL,
            fontsize=15, fontweight="bold", ha="center", va="center", bbox=dict(fc="white", ec="none", pad=1))


def painel(ax, titulo):
    ax.set_title(titulo, fontsize=12, fontweight="bold")
    ax.set_aspect("equal")
    ax.axis("off")


fig, axs = plt.subplots(2, 2, figsize=(15, 10.5))

# --- 1. placa da IMU, de frente
ax = axs[0, 0]
painel(ax, "1. Placa da IMU (de frente)")
ax.add_patch(Rectangle((0, 0), 40, 20, fc=VERDE, ec="#1d4a3e"))
ax.add_patch(Rectangle((27, 8.6), 3, 2.6, fc=ESCURO))
ax.text(28.5, 12.8, "chip", ha="center", color="white", fontsize=9)
ax.add_patch(Rectangle((-1, 4), 6, 12, fc="#eeeeee", ec="#555"))
ax.text(2, 10, "Grove", ha="center", va="center", rotation=90, fontsize=9)
for c in [(40, 10), (10, 0), (10, 20)]:
    ax.add_patch(Circle(c, 1.1, fc="white", ec=ESCURO))
cota(ax, (0, -3), (40, -3), "A", off=(0, -2))
cota(ax, (43, 0), (43, 20), "B", off=(2.2, 0))
ax.text(20, -9, "confirmar ~40 × 20 mm   ·   o cabo sai pelo lado da ficha Grove?",
        ha="center", fontsize=9, color="#444")
ax.set_xlim(-6, 50); ax.set_ylim(-12, 25)

# --- 2. placa da IMU, de lado
ax = axs[0, 1]
painel(ax, "2. Placa da IMU (de lado)")
ax.add_patch(Rectangle((0, 0), 40, 1.6, fc=VERDE, ec="#1d4a3e"))
ax.add_patch(Rectangle((0, 1.6), 6, 5, fc="#eeeeee", ec="#555"))
ax.add_patch(Rectangle((27, 1.6), 3, 0.9, fc=ESCURO))
ax.text(3, 7.6, "ficha Grove", ha="center", fontsize=9)
ax.text(28.5, 3.4, "chip", ha="center", fontsize=9)
cota(ax, (43, 0), (43, 1.6), "C", off=(2.2, 0))
cota(ax, (-2.5, 0), (-2.5, 6.6), "D", off=(-2.2, 0))
ax.text(20, -3.5, "C: espessura da placa (só a placa verde)\n"
        "D: da face de baixo da placa ao topo da ficha Grove", ha="center", fontsize=9, color="#444")
ax.text(20, 11, "Há alguma coisa soldada na face de baixo da placa?", ha="center", fontsize=9, color="#444")
ax.set_xlim(-8, 50); ax.set_ylim(-7, 13)

# --- 3. cabo Grove
ax = axs[1, 0]
painel(ax, "3. Cabo Grove (IMU → Arduino)")
ax.add_patch(Rectangle((0, 0), 5, 4, fc="#eeeeee", ec="#555"))
ax.add_patch(Rectangle((55, 0), 5, 4, fc="#eeeeee", ec="#555"))
for k, cor in enumerate(["#d33", "#222", "#ddd", "#e8c400"]):
    ax.plot([5, 55], [0.8 + k * 0.8] * 2, color=cor, lw=2)
cota(ax, (0, -3), (60, -3), "E", off=(0, -2.5))
ax.text(30, -10, "E: comprimento total, de ficha a ficha", ha="center", fontsize=9, color="#444")
ax.set_xlim(-5, 65); ax.set_ylim(-13, 9)

# --- 4. Arduino Uno, de lado
ax = axs[1, 1]
painel(ax, "4. Arduino Uno sem o acrílico (de lado)")
ax.add_patch(Rectangle((0, 0), 68.6, 1.6, fc="#1f5fa0", ec="#123"))           # Uno
for x in (14, 66):
    ax.add_patch(Rectangle((x - 1.5, -8), 3, 8, fc="#c9a227", ec="#8a6d12"))  # espaçadores
    ax.add_patch(Rectangle((x - 0.8, -11), 1.6, 3, fc="#999"))                # parafuso por baixo
ax.add_patch(Rectangle((0, 3.5), 68.6, 1.6, fc=VERDE, ec="#1d4a3e"))          # shield
ax.add_patch(Rectangle((40, 5.1), 8, 6, fc="#eeeeee", ec="#555"))             # ficha mais alta
ax.add_patch(Rectangle((-4, 1.6), 12, 10, fc="#888"))                         # USB
ax.text(2, 13, "USB", ha="center", fontsize=9)
ax.text(34, 6.8, "shield Grove (se houver)", ha="center", fontsize=8, color="white")
cota(ax, (72, 0), (72, 11.1), "F", off=(2.5, 0))
cota(ax, (75, -8), (75, 0), "G", off=(2.5, 0))
ax.add_patch(Rectangle((24, -12), 2.4, 1.2, fc="#555"))                      # parafuso solto
ax.add_patch(Rectangle((24.6, -10.8), 1.2, 8, fc="#999"))
ax.text(25.2, -13.5, "parafuso M3", ha="center", fontsize=8)
cota(ax, (28.5, -10.8), (28.5, -2.8), "H", off=(2.5, 0))
ax.text(34, -21, "F: da face de baixo do Uno ao ponto mais alto (fichas, shield, USB)\n"
        "G: altura dos espaçadores debaixo do Uno   ·   H: comprimento dos parafusos M3 (sem cabeça)",
        ha="center", fontsize=9, color="#444")
ax.set_xlim(-8, 82); ax.set_ylim(-25, 16)

fig.suptitle("Medidas do kit para o suporte sem acrílico (paquímetro, em mm)", fontsize=16, fontweight="bold")
fig.text(0.5, 0.01, "Tamanhos do desenho ilustrativos. Fotografa cada peça com a régua encostada.",
         ha="center", color="#555")
fig.tight_layout(rect=(0, 0.03, 1, 0.96))
out = Path(__file__).with_name("medidas_kit.png")
fig.savefig(out, dpi=110)
print(out)
