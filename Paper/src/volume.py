"""Altura da peça vs tamanho da mesa: a maior mesa onde se imprime uma peça de altura H
sem o robô bater na própria peça.

A peça é um bloco cheio do tamanho da mesa, do chão até à camada que se está a
imprimir (pelo lado seguro). Para cada H (de 0,2 em 0,2 m) procura-se o maior
retângulo, paralelo às paredes e a sul do robô, em que todos os pontos se
imprimem em todas as camadas de 0 a H com uma pose válida (plano.zona: bico para
baixo, limites, margens, sem colisões com sala e robô) que não toque na peça.

Como a peça depende do retângulo, guarda-se para cada pose a "sombra" do robô
abaixo da camada: retângulos xy por cima dos quais alguma peça do robô desce
abaixo da camada (troço do cilindro/cápsula abaixo de z, mais o raio; caixas
inteiras). Uma pose toca na peça se alguma sombra cruzar o retângulo da mesa.

    .venv/bin/python src/volume.py        # figura em resultados/volume_mesa.png
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
FIG = PAPER / "resultados" / "volume_mesa.png"
PASSO = 100.0                                   # mm, grelha dos pontos e dos lados da mesa
CAMADAS = np.arange(0.0, 1601.0, 200.0)         # mm, alturas das camadas verificadas
CORPOS = ("base", "A1", "A2", "A3", "A4", "A5", "A6", "flange")


def sombras(m, d, ids, z):
    """Retângulos xy (mm) das partes dos geoms ids abaixo da altura z (mm): (n,4)
    [x0, x1, y0, y1], NaN onde o geom está todo acima."""
    p = d.geom_xpos[ids] * 1000
    R = d.geom_xmat[ids].reshape(-1, 3, 3)
    sz = m.geom_size[ids] * 1000
    t = m.geom_type[ids]
    out = np.full((len(ids), 4), np.nan)
    for k in range(len(ids)):
        if t[k] == mujoco.mjtGeom.mjGEOM_BOX:
            meia = np.abs(R[k]) @ sz[k]
            if p[k, 2] - meia[2] < z:
                out[k] = [p[k, 0] - meia[0], p[k, 0] + meia[0], p[k, 1] - meia[1], p[k, 1] + meia[1]]
            continue
        r, h = sz[k, 0], sz[k, 1]
        ax = R[k][:, 2]
        a, b = p[k] - ax * h, p[k] + ax * h
        # quanto desce abaixo do eixo: cápsula, o raio; cilindro, só a aresta da base
        # (base plana): r * sin(inclinação do eixo em relação à vertical)
        rz = r if t[k] == mujoco.mjtGeom.mjGEOM_CAPSULE else r * np.sqrt(max(0.0, 1 - ax[2] ** 2))
        lim = z + rz
        if min(a[2], b[2]) >= lim:
            continue
        if abs(b[2] - a[2]) < 1e-9:
            t0, t1 = 0.0, 1.0
        else:
            tl = (lim - a[2]) / (b[2] - a[2])
            t0, t1 = (0.0, min(1.0, tl)) if b[2] > a[2] else (max(0.0, tl), 1.0)
        q0, q1 = a + (b - a) * t0, a + (b - a) * t1
        out[k] = [min(q0[0], q1[0]) - r, max(q0[0], q1[0]) + r, min(q0[1], q1[1]) - r, max(q0[1], q1[1]) + r]
    return out


def precalcular(m, d):
    """Pontos (centros das células a sul do robô), e por camada: válido (P,G) e sombras (P,G,n,4)."""
    xs = np.arange(PASSO / 2, K.SALA[0], PASSO)
    ys = np.arange(PASSO / 2, K.ROBO_XY[1], PASSO)
    GX, GY = np.meshgrid(xs, ys)
    pts = np.column_stack([GX.ravel(), GY.ravel()])
    ids = np.flatnonzero(np.isin(m.geom_bodyid, [m.body(n).id for n in CORPOS]))
    ids = ids[ids != m.geom("ponta_bico").id]
    dados = {}
    for z in CAMADAS:
        f = CACHE / f"vol_S{PL.S_FIXO}_z{int(z)}_p{int(PASSO)}_r{int(K.ROBO_XY[0])}_{int(K.ROBO_XY[1])}.npz"
        if f.exists():
            a = np.load(f)
            dados[z] = (a["V"], a["S"])
            continue
        G = len(PL.PSI)
        V = np.zeros((len(pts), G), bool)
        S = np.full((len(pts), G, len(ids), 4), np.nan, np.float32)
        p3 = np.column_stack([pts, np.full(len(pts), z)])
        for g, psi in enumerate(PL.PSI):
            Q, ok = PL.ik_s0(p3, psi)
            for i in np.flatnonzero(ok):
                if not cena.colisoes(m, d, Q[i]):
                    V[i, g] = True
                    S[i, g] = sombras(m, d, ids, z)
        np.savez_compressed(f, V=V, S=S)
        dados[z] = (V, S)
        print(f"  camada {z / 1000:.1f} m pré-calculada")
    return xs, ys, pts, dados


def cabe(rect, H, pts, dados):
    """A peça de altura H no retângulo [x0, x1, y0, y1] (mm) imprime-se toda?"""
    x0, x1, y0, y1 = rect
    dentro = (pts[:, 0] > x0) & (pts[:, 0] < x1) & (pts[:, 1] > y0) & (pts[:, 1] < y1)
    for z in CAMADAS[CAMADAS <= H + 1e-6]:
        V, S = dados[z]
        Vi, Si = V[dentro], S[dentro]
        if z > 0:
            toca = ((Si[..., 1] > x0) & (Si[..., 0] < x1) & (Si[..., 3] > y0) & (Si[..., 2] < y1))
            toca = np.where(np.isnan(Si[..., 0]), False, toca).any(-1)
            ok = (Vi & ~toca).any(1)
        else:
            ok = Vi.any(1)
        if not ok.all():
            return False
    return True


def maior_mesa(H, xs, ys, pts, dados):
    """Melhor retângulo (área, [x0, x1, y0, y1]) para a peça de altura H."""
    melhor = (0.0, None)
    bordas_y = np.r_[ys - PASSO / 2, ys[-1] + PASSO / 2]
    for a, y0 in enumerate(bordas_y):
        for y1 in bordas_y[a + 1:]:
            # maior intervalo de x em que todas as linhas têm pontos válidos sem a peça
            linhas = (pts[:, 1] > y0) & (pts[:, 1] < y1)
            col_ok = np.ones(len(xs), bool)
            for z in CAMADAS[CAMADAS <= H + 1e-6]:
                okp = dados[z][0].any(1)
                for y in np.unique(pts[linhas, 1]):
                    col_ok &= okp[pts[:, 1] == y]
            if not col_ok.any():
                continue
            idx = np.flatnonzero(col_ok)
            tro = np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)
            t = max(tro, key=len)
            x0, x1 = xs[t[0]] - PASSO / 2, xs[t[-1]] + PASSO / 2
            if (x1 - x0) * (y1 - y0) <= melhor[0]:
                continue
            # encolhe em x (centrado) até a peça caber
            lo, hi = 0, int(round((x1 - x0) / PASSO / 2))
            if not cabe([x0, x1, y0, y1], H, pts, dados):
                while lo < hi:                          # menor encolhimento que cabe
                    k = (lo + hi) // 2
                    if cabe([x0 + k * PASSO, x1 - k * PASSO, y0, y1], H, pts, dados):
                        hi = k
                    else:
                        lo = k + 1
                x0, x1 = x0 + lo * PASSO, x1 - lo * PASSO
                if x1 <= x0 or not cabe([x0, x1, y0, y1], H, pts, dados):
                    continue
            area = (x1 - x0) * (y1 - y0)
            if area > melhor[0]:
                melhor = (area, [x0, x1, y0, y1])
    return melhor


def confirmar(rect, H, passo=50.0):
    """Verificação direta no MuJoCo, com a peça como bloco: pontos da mesa sem pose por camada."""
    x0, x1, y0, y1 = rect
    gx, gy = np.meshgrid(np.arange(x0 + passo / 2, x1, passo), np.arange(y0 + passo / 2, y1, passo))
    P = np.column_stack([gx.ravel(), gy.ravel()])
    falhas = 0
    for z in np.arange(0.0, H + 1, 100.0):
        _, m, d = cena.modelo(peca=(x0, y0, x1, y1, z))
        p3 = np.column_stack([P, np.full(len(P), z)])
        Q, V = zip(*(PL.ik_s0(p3, psi) for psi in PL.PSI))
        Q, V = np.stack(Q, 1), np.stack(V, 1)
        for i in range(len(P)):
            if not any(not cena.colisoes(m, d, Q[i, g]) for g in np.flatnonzero(V[i])):
                falhas += 1
    return falhas


def main():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    CACHE.mkdir(exist_ok=True)
    _, m, d = cena.modelo()
    xs, ys, pts, dados = precalcular(m, d)
    Hs = CAMADAS[1:]
    res = []
    print("altura da peça | maior mesa (a sul do robô)        | área   | volume | verificação MuJoCo")
    for H in Hs:
        area, r = maior_mesa(H, xs, ys, pts, dados)
        if r is None:
            print(f"   {H / 1000:.1f} m      | nenhuma")
            res.append((H, 0.0, None))
            continue
        f = confirmar(r, H)
        res.append((H, area / 1e6, r))
        print(f"   {H / 1000:.1f} m      | {(r[1] - r[0]) / 1000:.1f} x {(r[3] - r[2]) / 1000:.1f} m "
              f"x {r[0] / 1000:.1f}-{r[1] / 1000:.1f}, y {r[2] / 1000:.1f}-{r[3] / 1000:.1f} | "
              f"{area / 1e6:5.2f} m² | {area / 1e6 * H / 1000:5.2f} m³ | {f} pontos sem pose")

    fig, axs = plt.subplots(1, 4, figsize=(17, 5.6), gridspec_kw=dict(width_ratios=[1.4, 1.4, 1, 1]))
    H = np.array([h for h, _, _ in res]) / 1000
    A = np.array([a for _, a, _ in res])
    axs[0].plot(H, A, "o-", color="tab:orange")
    axs[0].set_xlabel("altura da peça H (m)")
    axs[0].set_ylabel("maior mesa (m²)")
    axs[0].set_title("maior mesa em que a peça se imprime\nsem o robô lhe tocar", fontsize=10)
    axs[0].grid(alpha=0.3)
    axs[1].plot(H, A * H, "s-", color="tab:green")
    kv = int(np.argmax(A * H))
    axs[1].annotate(f"{(A * H)[kv]:.2f} m³ com H = {H[kv]:.1f} m", (H[kv], (A * H)[kv]),
                    textcoords="offset points", xytext=(-40, -25), fontsize=9)
    axs[1].set_xlabel("altura da peça H (m)")
    axs[1].set_ylabel("volume da peça (bloco) (m³)")
    axs[1].set_title("volume = mesa x altura", fontsize=10)
    axs[1].grid(alpha=0.3)
    for ax, Hm in zip(axs[2:], (0.4, H[kv])):
        i = int(np.argmin(np.abs(H - Hm)))
        _, a, r = res[i]
        ax.add_patch(plt.Rectangle((0, 0), K.SALA[0] / 1000, K.SALA[1] / 1000, fill=False, ec="0.3"))
        if r is not None:
            ax.add_patch(plt.Rectangle((r[0] / 1000, r[2] / 1000), (r[1] - r[0]) / 1000, (r[3] - r[2]) / 1000,
                                       fc=(0.95, 0.55, 0.1, 0.5), ec="tab:orange", lw=2))
        ax.plot(K.ROBO_XY[0] / 1000, K.ROBO_XY[1] / 1000, "k+", ms=12)
        ax.set_xlim(-0.1, K.SALA[0] / 1000 + 0.1)
        ax.set_ylim(-0.1, K.SALA[1] / 1000 + 0.1)
        ax.set_aspect("equal")
        ax.set_title(f"peça de {H[i]:.1f} m: mesa {a:.2f} m²", fontsize=10)
        ax.set_xlabel("x (m)")
    fig.suptitle(f"KR 240-2 C no teto em ({K.ROBO_XY[0] / 1000:.2f}; {K.ROBO_XY[1] / 1000:.2f}) m, cotovelo em baixo; "
                 "peça = bloco cheio do tamanho da mesa; mesa a sul do robô, paralela às paredes", fontsize=11)
    fig.tight_layout()
    fig.savefig(FIG, dpi=110)
    print(f"Figura: {FIG.relative_to(PAPER)}")


if __name__ == "__main__":
    main()
