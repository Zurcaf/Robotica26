"""Onde pôr o robô na sala para ter a maior mesa retangular.

Para cada posição do robô (x, y) calcula-se a zona útil (bico para baixo,
critério de plano.zona, configuração plano.S_FIXO) e o maior retângulo paralelo
às paredes que lá cabe, para duas peças:
  - H = 0: só a primeira camada (bico a 0 m);
  - H = HMAX: a peça mais alta possível; o ponto tem de servir a todas as
    alturas de 0 a HMAX (de 0,2 em 0,2 m).

Mudar o robô de sítio só muda as paredes: chão, teto, placa e o próprio robô dão
o mesmo. Por isso calcula-se uma vez, à volta do robô, que poses são válidas
(sem as paredes) e até onde o robô e a extrusora chegam em x e y em cada uma;
para cada posição basta ver que poses cabem entre as paredes (planos).

    .venv/bin/python src/posicao.py            # figura em resultados/posicao_robo.png
"""
from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

import cena
import kuka as K
import plano as PL

PAPER = Path(__file__).resolve().parents[1]
CACHE = PAPER / "cache"
FIG = PAPER / "resultados" / "posicao_robo.png"
PASSO = 50.0
RAIO = 3100.0                       # mm, maior distância do bico ao eixo do A1 que vale a pena ver
HMAX = 1600.0
ALTURAS = np.arange(0.0, HMAX + 1, 200.0)
PAREDES = ("parede_esq", "parede_dir", "parede_cima", "parede_baixo")
FOLGA_PLACA = K.PLACA / 2           # o centro do robô fica a pelo menos meia placa das paredes


def _extensoes(m, d, ids):
    """Até onde vão em x e y (mundo, mm) os geoms ids na pose atual: (xmin, xmax, ymin, ymax)."""
    p = d.geom_xpos[ids]
    R = d.geom_xmat[ids].reshape(-1, 3, 3)
    sz = m.geom_size[ids]
    t = m.geom_type[ids]
    meia = np.zeros((len(ids), 3))
    box = t == mujoco.mjtGeom.mjGEOM_BOX
    meia[box] = np.einsum("nij,nj->ni", np.abs(R[box]), sz[box])
    cil = (t == mujoco.mjtGeom.mjGEOM_CYLINDER) | (t == mujoco.mjtGeom.mjGEOM_CAPSULE)
    ax = R[cil][:, :, 2]
    h = sz[cil, 1] + np.where(t[cil] == mujoco.mjtGeom.mjGEOM_CAPSULE, sz[cil, 0], 0.0)
    meia[cil] = np.abs(ax) * h[:, None] + sz[cil, 0][:, None] * np.sqrt(np.clip(1 - ax ** 2, 0, 1))
    lo, hi = (p - meia) * 1000, (p + meia) * 1000
    return lo[:, 0].min(), hi[:, 0].max(), lo[:, 1].min(), hi[:, 1].max()


def precalcular(z, m, d):
    """Para a altura z: pontos relativos ao robô (N,2) e, para cada ponto e
    rotação, se a pose é válida sem paredes (N,G) e as extensões relativas (N,G,4)."""
    r = np.arange(-RAIO, RAIO + 1, PASSO) + PASSO / 2     # centros das células da grelha da sala
    RX, RY = np.meshgrid(r, r)
    rel = np.column_stack([RX.ravel(), RY.ravel()])
    rel = rel[np.hypot(rel[:, 0], rel[:, 1]) <= RAIO]
    c = np.array(K.ROBO_XY)
    p = np.column_stack([rel + c, np.full(len(rel), z)])
    ids = np.flatnonzero(np.isin(m.geom_bodyid, [m.body(n).id for n in
                                                  ("base", "A1", "A2", "A3", "A4", "A5", "A6", "flange")]))
    G = len(PL.PSI)
    V = np.zeros((len(rel), G), bool)
    E = np.full((len(rel), G, 4), np.nan, np.float32)
    for g, psi in enumerate(PL.PSI):
        Q, ok = PL.ik_s0(p, psi)
        for i in np.flatnonzero(ok):
            if not cena.colisoes(m, d, Q[i]):            # paredes desligadas em m
                V[i, g] = True
                E[i, g] = _extensoes(m, d, ids) - np.r_[c[0], c[0], c[1], c[1]]
    return rel, V, E


def zona_em(robo, rel, V, E):
    """Zona útil (pontos relativos válidos) com o robô em `robo` (mm) nesta sala."""
    X, Y = K.SALA[0], K.SALA[1]
    dentro = ((E[..., 0] >= -robo[0]) & (E[..., 1] <= X - robo[0])
              & (E[..., 2] >= -robo[1]) & (E[..., 3] <= Y - robo[1]))
    ok = (V & dentro).any(1)
    pw = rel + robo
    return ok & (pw[:, 0] > 0) & (pw[:, 0] < X) & (pw[:, 1] > 0) & (pw[:, 1] < Y)


def para_grelha(ok, rel, robo):
    """Pontos válidos -> grelha da sala (bool), com células de PASSO."""
    nx, ny = int(K.SALA[0] / PASSO), int(K.SALA[1] / PASSO)
    M = np.zeros((ny, nx), bool)
    pw = rel[ok] + robo
    j = np.floor(pw[:, 0] / PASSO).astype(int)
    i = np.floor(pw[:, 1] / PASSO).astype(int)
    v = (i >= 0) & (i < ny) & (j >= 0) & (j < nx)
    M[i[v], j[v]] = True
    return M


def maior_retangulo(M):
    """Maior retângulo de células True (alinhado com a grelha): (área em células,
    linha0, linha1, coluna0, coluna1), limites incluídos."""
    alt = np.zeros(M.shape[1], int)
    melhor = (0, 0, -1, 0, -1)
    for i in range(M.shape[0]):
        alt = np.where(M[i], alt + 1, 0)
        pilha = []
        for j in range(M.shape[1] + 1):
            h = alt[j] if j < M.shape[1] else 0
            ini = j
            while pilha and pilha[-1][1] >= h:
                ini, hh = pilha.pop()
                if hh * (j - ini) > melhor[0]:
                    melhor = (hh * (j - ini), i - hh + 1, i, ini, j - 1)
            pilha.append((ini, h))
    return melhor


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    s, m, d = cena.modelo()
    for n in PAREDES:                                      # as paredes entram depois, como planos
        m.geom_contype[m.geom(n).id] = 0
    CACHE.mkdir(exist_ok=True)
    pre = {}
    for z in ALTURAS:
        f = CACHE / f"pre_S{PL.S_FIXO}_z{int(z)}_p{int(PASSO)}.npz"
        if f.exists():
            a = np.load(f)
            pre[z] = (a["rel"], a["V"], a["E"])
        else:
            pre[z] = precalcular(z, m, d)
            np.savez_compressed(f, rel=pre[z][0], V=pre[z][1], E=pre[z][2])
        print(f"  pré-cálculo a {z / 1000:.1f} m feito")

    # verificação: na posição atual, igual ao cálculo direto (paredes no MuJoCo)
    _, m2, d2 = cena.modelo()
    xs, ys, Mdir = PL.zona(m2, d2, z=0.0, passo=PASSO)
    robo0 = np.array(K.ROBO_XY)
    M0 = para_grelha(zona_em(robo0, *pre[0.0]), pre[0.0][0], robo0)
    dif = (M0 != Mdir).sum()
    print(f"Verificação na posição atual, a 0 m: {dif} células diferentes em {Mdir.sum()} "
          f"(cálculo direto {Mdir.sum() * (PASSO / 1000) ** 2:.2f} m², por planos {M0.sum() * (PASSO / 1000) ** 2:.2f} m²)")

    def retangulos(robo):
        Ms = [para_grelha(zona_em(robo, *pre[z]), pre[z][0], robo) for z in ALTURAS]
        r0 = maior_retangulo(Ms[0])
        rH = maior_retangulo(np.logical_and.reduce(Ms))
        return Ms[0], np.logical_and.reduce(Ms), r0, rH

    px = np.arange(FOLGA_PLACA, K.SALA[0] - FOLGA_PLACA + 1, 100.0)
    py = np.arange(FOLGA_PLACA, K.SALA[1] - FOLGA_PLACA + 1, 100.0)
    A0 = np.zeros((len(py), len(px)))
    AH = np.zeros_like(A0)
    cel = (PASSO / 1000) ** 2
    for i, y in enumerate(py):
        for j, x in enumerate(px):
            _, _, r0, rH = retangulos(np.array([x, y]))
            A0[i, j], AH[i, j] = r0[0] * cel, rH[0] * cel
    _, _, r0a, rHa = retangulos(robo0)

    def dims(r):
        return (r[4] - r[3] + 1) * PASSO / 1000, (r[2] - r[1] + 1) * PASSO / 1000

    print(f"\nPosição atual ({robo0[0] / 1000:.2f}; {robo0[1] / 1000:.2f}) m: maior mesa "
          f"{dims(r0a)[0]:.2f} x {dims(r0a)[1]:.2f} m = {r0a[0] * cel:.2f} m² (H = 0); "
          f"{dims(rHa)[0]:.2f} x {dims(rHa)[1]:.2f} m = {rHa[0] * cel:.2f} m² (H = {HMAX / 1000:.1f} m)")
    melhores = {}
    for nome, A in (("H = 0", A0), (f"H = {HMAX / 1000:.1f} m", AH)):
        i, j = np.unravel_index(np.argmax(A), A.shape)
        robo = np.array([px[j], py[i]])
        _, _, r0, rH = retangulos(robo)
        r = r0 if nome == "H = 0" else rH
        melhores[nome] = (robo, r)
        print(f"Melhor para {nome}: robô em ({robo[0] / 1000:.2f}; {robo[1] / 1000:.2f}) m -> mesa "
              f"{dims(r)[0]:.2f} x {dims(r)[1]:.2f} m = {r[0] * cel:.2f} m²")

    fig, axs = plt.subplots(1, 4, figsize=(17, 6.3))
    ext = [px[0] / 1000 - 0.05, px[-1] / 1000 + 0.05, py[0] / 1000 - 0.05, py[-1] / 1000 + 0.05]
    for ax, A, nome in ((axs[0], A0, "H = 0"), (axs[1], AH, f"H = {HMAX / 1000:.1f} m")):
        im = ax.imshow(A, origin="lower", extent=ext, cmap="viridis")
        ax.set_xlim(0, K.SALA[0] / 1000)
        ax.set_ylim(0, K.SALA[1] / 1000)
        ax.plot(*robo0 / 1000, "w+", ms=12, mew=2)
        ax.plot(*melhores[nome][0] / 1000, "r*", ms=12)
        ax.set_title(f"maior mesa (m²), peça {nome}\n+ = atual, * = melhor", fontsize=10)
        ax.set_xlabel("x do robô (m)")
        ax.set_ylabel("y do robô (m)")
        ax.set_aspect("equal")
        fig.colorbar(im, ax=ax, fraction=0.05)
    for ax, (robo, r), titulo in ((axs[2], (robo0, rHa), "posição atual"),
                                  (axs[3], melhores[f"H = {HMAX / 1000:.1f} m"], "melhor posição")):
        _, MH, _, _ = retangulos(robo)
        ax.imshow(MH, origin="lower", extent=[0, K.SALA[0] / 1000, 0, K.SALA[1] / 1000], cmap="Greens",
                  vmin=0, vmax=1.6, interpolation="nearest")
        w, h = dims(r)
        ax.add_patch(plt.Rectangle((r[3] * PASSO / 1000, r[1] * PASSO / 1000), w, h, fill=False, ec="r", lw=2))
        ax.plot(*robo / 1000, "k+", ms=12)
        ax.set_title(f"{titulo}: robô em ({robo[0] / 1000:.2f}; {robo[1] / 1000:.2f}) m\n"
                     f"zona útil para H = {HMAX / 1000:.1f} m; mesa {w:.2f} x {h:.2f} m", fontsize=10)
        ax.set_xlim(0, K.SALA[0] / 1000)
        ax.set_ylim(0, K.SALA[1] / 1000)
        ax.set_aspect("equal")
        ax.set_xlabel("x (m)")
    fig.suptitle(f"KR 240-2 C a {K.ROBO_Z / 1000:.1f} m, bico para baixo, A3 para baixo; sala 4 x 7 m; "
                 "mesa retangular paralela às paredes", fontsize=11)
    fig.tight_layout()
    fig.savefig(FIG, dpi=110)
    print(f"Figura: {FIG.relative_to(PAPER)}")


if __name__ == "__main__":
    main()
