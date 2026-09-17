"""
Pêndulo a bater numa bola. Larga o pêndulo de um ângulo inicial,
deixa-o bater na bola e reporta onde a bola parou.
Uso:
    python src/run_pendulum_ball.py            # corre e imprime resultado
    python src/run_pendulum_ball.py --view     # viewer interativo
"""
import sys
import numpy as np
import mujoco
import mujoco.viewer

MODEL = "models/01_pendulum_ball.xml"

def run(model, data, theta0=1.2, t_max=5.0):
    """theta0 positivo = pêndulo levantado do lado -x, oposto à bola (que está em +x)."""
    mujoco.mj_resetData(model, data)
    data.qpos[0] = theta0
    ball = model.body("ball").id
    v_max = 0.0
    while data.time < t_max:
        mujoco.mj_step(model, data)
        v = np.linalg.norm(data.cvel[ball][3:])
        v_max = max(v_max, v)
    return data.xpos[ball].copy(), v_max

def main():
    model = mujoco.MjModel.from_xml_path(MODEL)
    data = mujoco.MjData(model)

    if "--view" in sys.argv:
        data.qpos[0] = 1.2
        mujoco.viewer.launch(model, data)
        return

    pos, v_max = run(model, data)
    print(f"Bola parou em x={pos[0]:.3f} m, y={pos[1]:.3f} m")
    print(f"Velocidade máxima da bola: {v_max:.2f} m/s")

if __name__ == "__main__":
    main()
