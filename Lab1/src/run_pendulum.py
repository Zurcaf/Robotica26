"""
Corre o pêndulo simples e mede o período de oscilação.
Uso:
    python src/run_pendulum.py            # só mede o período
    python src/run_pendulum.py --view     # abre o viewer interativo
"""
import sys
import math
import numpy as np
import mujoco
import mujoco.viewer

MODEL = "models/00_pendulum.xml"

def measure_period(model, data, theta0=0.1, t_max=10.0):
    """Larga o pêndulo de theta0 rad e conta cruzamentos por zero."""
    mujoco.mj_resetData(model, data)
    data.qpos[0] = theta0
    crossings = []
    prev = data.qpos[0]
    while data.time < t_max:
        mujoco.mj_step(model, data)
        cur = data.qpos[0]
        if prev < 0 <= cur:          # cruzamento ascendente
            crossings.append(data.time)
        prev = cur
    if len(crossings) < 2:
        return float("nan")
    return float(np.mean(np.diff(crossings)))

def main():
    model = mujoco.MjModel.from_xml_path(MODEL)
    data = mujoco.MjData(model)

    if "--view" in sys.argv:
        data.qpos[0] = 0.8
        mujoco.viewer.launch(model, data)
        return

    L, g = 1.0, 9.81
    T_theory = 2 * math.pi * math.sqrt(L / g)
    T_sim = measure_period(model, data)
    print(f"Período teórico (pequenas oscilações): {T_theory:.4f} s")
    print(f"Período simulado:                      {T_sim:.4f} s")
    print(f"Erro relativo:                         {abs(T_sim - T_theory) / T_theory * 100:.2f} %")

if __name__ == "__main__":
    main()
