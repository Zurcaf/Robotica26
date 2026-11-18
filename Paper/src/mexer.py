"""Ver o KR 240-2 C a mexer no MuJoCo, com os limites e as velocidades das juntas.

    .venv/bin/python src/mexer.py            # demonstração: cada junta de um limite ao outro
    .venv/bin/python src/mexer.py --barras   # mexer à mão, com as barras do separador Control

As juntas andam no máximo à velocidade da ficha com 240 kg (kuka.VEL), em graus
por segundo, e não passam dos limites do controlador (kuka.LIM). No modo
--barras, cada barra é o alvo de uma junta (em radianos; o viewer não mostra
graus) e a junta vai até lá à velocidade máxima. É cinemática: sem gravidade,
massas nem colisões.

No macOS a janela abre com o python normal da .venv (como nos Labs 2 e 3).
"""
from __future__ import annotations

import argparse

import mujoco
import mujoco.viewer
import numpy as np

import cena
import kuka as K

VMAX = np.radians(K.VEL)


def plano_demo():
    """Lista de alvos (graus): cada junta vai a um limite, ao outro, e volta a casa."""
    alvos = [K.CASA.copy()]
    for i in range(6):
        lo, hi = K.LIM[i]
        if i in (3, 5):                       # A4 e A6 rodam ±350°: basta meia volta para se ver
            lo, hi = -180.0, 180.0
        for v in (lo, hi, K.CASA[i]):
            q = alvos[-1].copy()
            q[i] = v
            alvos.append(q)
    # uma pose de "impressão": bico para baixo, à frente do robô
    alvos += [np.array([0.0, -60.0, 100.0, 0.0, 50.0, 0.0]), np.array([60.0, -40.0, 70.0, 0.0, 60.0, 0.0]),
              np.array([-60.0, -40.0, 70.0, 0.0, 60.0, 0.0]), K.CASA.copy()]
    return [np.radians(np.clip(a, K.LIM[:, 0], K.LIM[:, 1])) for a in alvos]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--barras", action="store_true", help="mexer à mão com as barras (separador Control)")
    a = ap.parse_args()
    _, m, d = cena.modelo()
    mujoco.mj_resetDataKeyframe(m, d, 0)
    alvos = plano_demo()
    estado = {"k": 0}

    def passo(model, data):
        dt = model.opt.timestep
        if a.barras:
            alvo = data.ctrl.copy()
        else:
            alvo = alvos[estado["k"]]
            if np.abs(alvo - data.qpos).max() < 1e-4:
                estado["k"] = (estado["k"] + 1) % len(alvos)
        dq = np.clip(alvo - data.qpos, -VMAX * dt, VMAX * dt)
        data.qpos[:] = np.clip(data.qpos + dq, np.radians(K.LIM[:, 0]), np.radians(K.LIM[:, 1]))
        data.qvel[:] = 0.0

    mujoco.set_mjcb_control(passo)
    try:
        mujoco.viewer.launch(m, d)
    finally:
        mujoco.set_mjcb_control(None)


if __name__ == "__main__":
    main()
