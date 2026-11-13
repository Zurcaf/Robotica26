"""
Tarefa 4 (discussão) — A diferença entre a nossa FK e a do controlador é geométrica?

Uso (a partir da pasta acima de src/):
    python src/ajuste_dh.py

Ajusta, por mínimos quadrados amortecidos, pequenas correções aos 23 parâmetros
D-H (tudo menos o offset de J1, que é só a escolha do zero) de modo a que a
nossa FK reproduza a FK do controlador nos 60 pontos de data/ik_compare.csv.
Depois testa os parâmetros ajustados em dados que não entraram no ajuste: as
juntas e a pose medidas durante um segundo swing executado no robô
(data/run_v3_lento2.csv), noutra zona do espaço de trabalho.

Se o erro desce nos dois conjuntos, a diferença vem de o controlador usar
parâmetros geométricos ligeiramente diferentes dos nominais do PDF (calibração),
e não de imprecisão da IK. Os valores ajustados não são a calibração do robô:
com poses de uma só trajetória vários parâmetros compensam-se uns aos outros
(o ajuste é mal condicionado), por isso só o resíduo interessa.
"""
import os

import numpy as np

from kinematics import DH, R_from_rpy, link_transform

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "data")


def fk_dh(dh, q_deg):
    q, T = np.radians(q_deg), np.eye(4)
    for i, (off, d, alpha, a) in enumerate(dh):
        T = T @ link_transform(np.radians(alpha), a, q[i] + np.radians(off), d)
    return T


def erro(dh, Q, P):
    e = np.array([np.linalg.norm(fk_dh(dh, q)[:3, 3] - p[:3]) for q, p in zip(Q, P)])
    return e.mean(), e.max()


def ajustar(Q, P, iters=30, lam=1e-4, w_rot=50.0):
    livres = np.argwhere(np.ones((6, 4), bool))[1:]          # tudo menos o offset de J1

    def montar(x):
        dh = DH.copy()
        for (i, j), xi in zip(livres, x):
            dh[i, j] += xi
        return dh

    def residuo(x):
        dh, r = montar(x), []
        for q, p in zip(Q, P):
            T = fk_dh(dh, q)
            r += [T[:3, 3] - p[:3], w_rot * (T[:3, :3] - R_from_rpy(*p[3:])).ravel()]
        return np.concatenate(r)

    x = np.zeros(len(livres))
    for _ in range(iters):
        r0, h = residuo(x), 1e-5
        J = np.column_stack([(residuo(x + h * np.eye(len(x))[k]) - r0) / h for k in range(len(x))])
        x -= np.linalg.solve(J.T @ J + lam * np.eye(len(x)), J.T @ r0)
    return montar(x)


def main():
    r = np.loadtxt(os.path.join(DATA, "ik_compare.csv"), delimiter=",")
    Q4, P4 = r[:, 7:13], r[:, 19:25]                         # IK do robô, FK do robô (tcp_offset = 0)
    v = np.loadtxt(os.path.join(DATA, "run_v3_lento2.csv"), delimiter=",")[::10]
    Q3, P3 = v[:, 7:13], v[:, 13:19]                         # juntas e pose medidas no segundo swing
    dh = ajustar(Q4, P4)
    print("  nossa FK vs FK do controlador [mm]        média   máx")
    for nome, Q, P in (("Tarefa 4 (60 pontos, usados no ajuste)  ", Q4, P4), ("segundo swing (205 poses, fora do ajuste)", Q3, P3)):
        a, b = erro(DH, Q, P), erro(dh, Q, P)
        print(f"  {nome}: D-H nominal {a[0]:.2f} / {a[1]:.2f}   ->   D-H ajustado {b[0]:.2f} / {b[1]:.2f}")
    print("  correções ajustadas (colunas: offset [deg], d [mm], alpha [deg], a [mm]) — só indicativas:")
    print(np.round(dh - DH, 2))


if __name__ == "__main__":
    main()
