"""O robô desenha no chão (z = 0) a maior zona útil, com o bico sempre para baixo.

A zona e o caminho vêm de plano.py (configuração fixa, margens às
singularidades, sem colisões, rotação da extrusora planeada para o caminho todo).
O tempo de cada troço é o da junta mais lenta à velocidade da ficha (kuka.VEL):
abranda onde for preciso.

    .venv/bin/python src/desenhar.py                 # calcula, verifica e abre o MuJoCo
    .venv/bin/python src/desenhar.py --sem-janela    # só os números
    .venv/bin/python src/desenhar.py --linhas 300    # distância entre linhas (mm)
"""
from __future__ import annotations

import argparse

import mujoco
import mujoco.viewer
import numpy as np

import cena
import kuka as K
import plano as PL


def tempos(qs):
    dt = np.max(np.abs(np.diff(qs, axis=0)) / K.VEL, axis=1)
    return np.r_[0.0, np.cumsum(dt)]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--linhas", type=float, default=200.0, help="distância entre linhas, mm (200)")
    ap.add_argument("--sem-janela", action="store_true")
    ap.add_argument("--robo", type=float, nargs=2, metavar=("X", "Y"), help="posição do robô na sala, mm")
    ap.add_argument("--mesa", type=float, nargs=4, metavar=("X0", "Y0", "X1", "Y1"),
                    help="desenhar só este retângulo (mm), em vez da zona toda")
    ap.add_argument("--z", type=float, nargs="+", default=[0.0],
                    help="altura(s) do bico, mm; várias = uma camada por altura, de baixo para cima")
    a = ap.parse_args()
    if a.robo:
        K.ROBO_XY = tuple(a.robo)

    _, m, d = cena.modelo()
    camadas = []
    for z in a.z:
        xs, ys, M = PL.zona(m, d, z=z)
        if a.mesa:
            x0, y0, x1, y1 = a.mesa
            GX, GY = np.meshgrid(xs, ys)
            R = (GX >= x0) & (GX <= x1) & (GY >= y0) & (GY <= y1)
            fora = (R & ~M).sum()
            if fora:
                print(f"Aviso: a {z / 1000:.1f} m, {fora} células da mesa ficam fora da zona útil")
            M = R
        camadas.append(PL.linhas(xs, ys, M, a.linhas, z=z))
    area = M.sum() * ((xs[1] - xs[0]) / 1000) ** 2
    # entre camadas: sobe na vertical no fim de uma e anda a direito até ao início da seguinte
    pts = camadas[0]
    for c in camadas[1:]:
        sobe = PL._reta(pts[-1], np.r_[pts[-1][:2], c[0][2]])
        anda = PL._reta(sobe[-1], c[0])
        pts = np.vstack([pts, sobe, anda, c])
    qs, partes, _ = PL.planear(pts, m, d)
    v = PL.verificar(qs, pts, m, d)
    t = tempos(qs)
    print(f"Robô em ({K.ROBO_XY[0] / 1000:.2f}; {K.ROBO_XY[1] / 1000:.2f}) m; "
          f"{'mesa' if a.mesa else 'zona útil'} (bico para baixo, S = {PL.S_FIXO}, A5 >= {PL.MARGEM_A5:.0f}°, "
          f"cotovelo a >= {PL.MARGEM_A3:.0f}° de esticado, sem colisões): {area:.1f} m², camadas a {[z / 1000 for z in a.z]} m")
    print(f"Caminho: {len(pts)} pontos, linhas a {a.linhas:.0f} mm, {t[-1]:.0f} s")
    print(f"Partes (sítios sem transição suave): {len(partes)}"
          + (f" em {[tuple(pts[i][:2].round()) for i in partes[:5]]}" if partes else ""))
    print(f"Maior salto de uma junta entre pontos: {v['salto']:.1f}° (limite {PL.DQ_MAX:.0f}°); "
          f"juntas fora dos limites: {v['fora'] or 'nenhuma'}; S usados: {v['status']}; A5 mínimo {v['a5_min']:.1f}°")
    print(f"Bico no sítio: erro máximo {v['erro']:.1e} mm; pontos sem pose: {v['sem_pose']}")
    print(f"Movimentos com colisão: {len(v['colisoes'])}"
          + (f", ex. t = {t[v['colisoes'][0][0]]:.1f} s {v['colisoes'][0][1]}" if v["colisoes"] else ""))
    if a.sem_janela:
        return

    # cena com o caminho desenhado no chão: um cilindro por troço reto
    s = cena.construir()
    w = s.worldbody
    i0 = 0
    for i in range(1, len(pts)):
        fim = i == len(pts) - 1
        if fim or np.linalg.norm(np.cross(pts[i] - pts[i0], pts[i + 1] - pts[i0])) > 1e-3:
            if np.linalg.norm(pts[i] - pts[i0]) > 1:
                w.add_geom(type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[0.008],
                           fromto=np.r_[pts[i0], pts[i]] * 1e-3 + [0, 0, 0.002, 0, 0, 0.002],
                           rgba=[0.1, 0.5, 0.9, 1], contype=0, conaffinity=0)
            i0 = i
    if a.mesa:                                            # volume imprimível, transparente
        x0, y0, x1, y1 = (v * 1e-3 for v in a.mesa)
        zt = max(max(a.z) * 1e-3, 0.01)
        w.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[(x1 - x0) / 2, (y1 - y0) / 2, zt / 2],
                   pos=[(x0 + x1) / 2, (y0 + y1) / 2, zt / 2], rgba=[0.2, 0.8, 0.3, 0.15],
                   contype=0, conaffinity=0)
    m = s.compile()
    d = mujoco.MjData(m)
    # ida de casa ao início e volta, em juntas
    q_all = np.vstack([np.linspace(K.CASA, qs[0], 60), qs, np.linspace(qs[-1], K.CASA, 60)])
    t = tempos(q_all)
    qrad = np.radians(q_all)
    T = t[-1] + 2.0
    cor = m.geom_rgba.copy()

    def passo(model, data):
        tt = data.time % T
        data.qpos[:] = [np.interp(tt, t, qrad[:, j]) for j in range(6)]
        data.qvel[:] = 0.0
        model.geom_rgba[:] = cor                          # vermelho = em colisão
        mujoco.mj_kinematics(model, data)
        mujoco.mj_collision(model, data)
        for c in data.contact[:data.ncon]:
            if c.dist < 0:
                for g in (c.geom1, c.geom2):
                    if model.geom_bodyid[g] != 0:
                        model.geom_rgba[g] = [1, 0, 0, 1]

    mujoco.set_mjcb_control(passo)
    try:
        d.qpos[:] = qrad[0]
        mujoco.viewer.launch(m, d)
    finally:
        mujoco.set_mjcb_control(None)


if __name__ == "__main__":
    main()
