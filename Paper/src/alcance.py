"""Mapa de alcance: área útil do bico (para baixo) a cada altura acima do chão.

Mesmo critério do desenho no chão (plano.zona): bico na vertical para baixo,
rotação em torno do bico livre, limites das juntas, margens às singularidades
(plano.MARGEM_A5, plano.MARGEM_A3) e sem colisões (chão, paredes, teto, placa e
o próprio robô; formas provisórias até ao CAD). Calcula as duas configurações do
cotovelo: S = 2 (A3 para baixo) e S = 0 (A3 para cima). Não verifica se a zona
se percorre de seguida (isso é o desenhar.py).

    .venv/bin/python src/alcance.py           # figura em resultados/alcance_bico_baixo.png
"""
from __future__ import annotations

from pathlib import Path

import numpy as np

import cena
import kuka as K
import plano as PL

PAPER = Path(__file__).resolve().parents[1]
FIG = PAPER / "resultados" / "alcance_bico_baixo.png"
PASSO = 50.0                                   # mm, grelha da planta
ALTURAS = np.arange(0.0, 2201.0, 100.0)        # mm, altura do bico acima do chão
NOMES = {2: "cotovelo em baixo", 0: "cotovelo em cima"}
PONTO_IMG = (1950.0, 3600.0)       # mm, ponto onde se mostra o robô nas duas configurações
ALT_IMG = (0.0, 300.0)             # mm, alturas do bico nas imagens


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    _, m, d = cena.modelo()
    a_cel = (PASSO / 1000) ** 2
    mapas = {S: [] for S in NOMES}
    s_ant = PL.S_FIXO
    for S in NOMES:
        PL.S_FIXO = S
        for z in ALTURAS:
            xs, ys, M = PL.zona(m, d, z=z, passo=PASSO)
            mapas[S].append(M)
    PL.S_FIXO = s_ant
    area = {S: np.array([M.sum() * a_cel for M in mapas[S]]) for S in NOMES}
    uniao = np.array([(a | b).sum() * a_cel for a, b in zip(mapas[2], mapas[0])])

    print("altura do bico (m) | área útil (m²): cotovelo em baixo | cotovelo em cima | as duas")
    for i, z in enumerate(ALTURAS):
        print(f"   {z / 1000:4.1f}            |   {area[2][i]:5.2f}         |   {area[0][i]:5.2f}      |  {uniao[i]:5.2f}")
    k = int(np.argmax(area[2]))
    print(f"Máximo com o cotovelo em baixo: {area[2][k]:.2f} m² com o bico a {ALTURAS[k] / 1000:.1f} m")

    mostrar = [0.0, 500.0, 1000.0, ALTURAS[k]]
    fig = plt.figure(figsize=(16, 11.5))
    cima, baixo = fig.subfigures(2, 1, height_ratios=[1.35, 1])
    gs = cima.add_gridspec(1, len(mostrar) + 1, width_ratios=[1] * len(mostrar) + [1.6])
    gb = baixo.add_gridspec(1, 4)
    rx, ry = K.ROBO_XY
    for j, z in enumerate(mostrar):
        ax = cima.add_subplot(gs[0, j])
        i = int(np.argmin(np.abs(ALTURAS - z)))
        img = np.ones(mapas[2][i].shape + (3,))
        img[mapas[2][i] & mapas[0][i]] = [0.40, 0.70, 0.40]
        img[mapas[2][i] & ~mapas[0][i]] = [0.95, 0.55, 0.10]
        img[mapas[0][i] & ~mapas[2][i]] = [0.20, 0.40, 0.90]
        ax.imshow(img, origin="lower", extent=[0, K.SALA[0] / 1000, 0, K.SALA[1] / 1000],
                  interpolation="nearest")
        ax.plot(rx / 1000, ry / 1000, "k+", ms=10)
        if j == 0:
            ax.plot(PONTO_IMG[0] / 1000, PONTO_IMG[1] / 1000, "kx", ms=7)
        ax.set_title(f"bico a {ALTURAS[i] / 1000:.1f} m\ncotovelo em baixo: {area[2][i]:.1f} m²", fontsize=10)
        ax.set_xlim(0, K.SALA[0] / 1000)
        ax.set_ylim(0, K.SALA[1] / 1000)
        ax.set_aspect("equal")
        ax.set_xlabel("x (m)")
        if j == 0:
            ax.set_ylabel("y (m)")
    ax = cima.add_subplot(gs[0, -1])
    ax.plot(ALTURAS / 1000, area[2], "o-", color="tab:orange", label=NOMES[2])
    ax.plot(ALTURAS / 1000, area[0], "s-", color="tab:blue", label=NOMES[0], ms=4)
    ax.plot(ALTURAS / 1000, uniao, ":", color="0.4", label="as duas (obriga a mudar de configuração)")
    ax.set_xlabel("altura do bico acima do chão (m)")
    ax.set_ylabel("área útil com o bico para baixo (m²)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)

    # o robô nas duas configurações, a chegar ao mesmo ponto (x nas plantas)
    import mujoco
    rend = mujoco.Renderer(m, 600, 640)
    cfg = [(z, S) for z in ALT_IMG for S in (2, 0)]
    for j, (z, S) in enumerate(cfg):
        ax = baixo.add_subplot(gb[0, j])
        ax.axis("off")
        PL.S_FIXO = S
        q = None
        for psi in PL.PSI:                                # pose válida com o punho mais perto de 60°
            qq, ok = PL.ik_s0(np.array([[*PONTO_IMG, z]]), psi)
            if ok[0] and not cena.colisoes(m, d, qq[0]) and (q is None or abs(qq[0, 4] - 60) < abs(q[4] - 60)):
                q = qq[0]
        PL.S_FIXO = s_ant
        cor = "tab:orange" if S == 2 else "tab:blue"
        if q is None:
            ax.set_title(f"{NOMES[S]}, bico a {z / 1000:.1f} m: não chega", fontsize=10, color=cor)
            continue
        d.qpos[:] = np.radians(q)
        mujoco.mj_forward(m, d)
        cam = mujoco.MjvCamera()
        cam.lookat[:] = [PONTO_IMG[0] / 1000, (PONTO_IMG[1] + K.ROBO_XY[1]) / 2000, 1.5]
        cam.distance, cam.azimuth, cam.elevation = 5.5, 0, -5
        rend.update_scene(d, camera=cam)
        ax.imshow(rend.render())
        ax.set_title(f"{NOMES[S]}, bico a {z / 1000:.1f} m\ncotovelo a {d.body('A3').xpos[2]:.1f} m do chão",
                     fontsize=10, color=cor)
    cima.suptitle(f"KR 240-2 C no teto a {K.ROBO_Z / 1000:.1f} m, em ({rx / 1000:.2f}; {ry / 1000:.2f}) m; sala 4 x 7 m; "
                  "bico sempre para baixo\nverde = as duas configurações chegam; laranja = só com o cotovelo em baixo; "
                  "azul = só com o cotovelo em cima", fontsize=11)
    baixo.suptitle("As duas configurações, vistas de lado, a chegar ao mesmo ponto (x na primeira planta)", fontsize=11)
    FIG.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIG, dpi=110)
    print(f"Figura: {FIG.relative_to(PAPER)}")


if __name__ == "__main__":
    main()
