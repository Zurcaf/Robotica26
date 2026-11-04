"""
Tarefa 3 — O Lite 6 segue a trajetória de swing da Tarefa 2.

Uso (a partir de Lab2/):
    python src/run_robot.py                    # ENSAIO: verifica e mostra o que faria, não mexe
    python src/run_robot.py --go               # executa no robô (pede confirmação)
    python src/run_robot.py --go --metodo servo --lento 2

Métodos (ambos com as funções do SDK):
  pontos  set_position() com blending (radius) em modo 0: o controlador
          interpola entre waypoints subamostrados. Mais robusto.
  servo   set_servo_cartesian() em modo 1, um ponto a cada dt: segue o perfil
          temporal à risca (é o que preserva a aceleração do downswing).

--lento N estica o tempo N vezes (velocidades / N) — começar SEMPRE com N >= 3.
Em servo grava-se a pose medida em data/run_servo.csv, para comparar com a
de referência no relatório.

SEGURANÇA: a trajetória só é enviada se check_feasibility() passar. Ter sempre
a mão no botão de emergência.
"""
import argparse
import os
import time

import numpy as np

from robot import check, connect, pose_sdk
from trajectory import check_feasibility, joint_trajectory, swing_trajectory

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "data")
APPROACH_SPEED = 50.0  # mm/s, para ir até ao início da trajetória


def run_pontos(arm, traj, every):
    idx = list(range(0, len(traj["t"]), every)) + [len(traj["t"]) - 1]
    t, pos = traj["t"], traj["pos"]
    for a, b in zip(idx[:-1], idx[1:]):
        speed = np.linalg.norm(pos[b] - pos[a]) / (t[b] - t[a])
        x, y, z, r, p, yw = pose_sdk(pos[b], traj["rpy"][b])
        check(arm.set_position(x, y, z, r, p, yw, radius=5, speed=max(speed, 1.0), wait=False), "set_position")
    arm.set_position(*pose_sdk(pos[-1], traj["rpy"][-1]), speed=APPROACH_SPEED, wait=True)


def run_servo(arm, traj):
    arm.set_mode(1)
    arm.set_state(0)
    time.sleep(0.5)
    dt = traj["t"][1] - traj["t"][0]
    log = []
    t0 = time.perf_counter()
    for k, (pos, rpy) in enumerate(zip(traj["pos"], traj["rpy"])):
        check(arm.set_servo_cartesian(pose_sdk(pos, rpy)), "set_servo_cartesian")
        code, meas = arm.get_position()
        log.append([time.perf_counter() - t0, *pose_sdk(pos, rpy), *meas])
        while time.perf_counter() - t0 < (k + 1) * dt:
            pass
    arm.set_mode(0)
    arm.set_state(0)
    os.makedirs(DATA, exist_ok=True)
    out = os.path.join(DATA, "run_servo.csv")
    np.savetxt(out, np.array(log), delimiter=",", fmt="%.4f",
               header="t,x_ref,y_ref,z_ref,roll_ref,pitch_ref,yaw_ref,x,y,z,roll,pitch,yaw")
    print(f"  pose medida gravada em {os.path.normpath(out)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--go", action="store_true", help="mexe mesmo o robô")
    ap.add_argument("--metodo", choices=["pontos", "servo"], default="pontos")
    ap.add_argument("--lento", type=float, default=3.0, help="fator de abrandamento (>= 1)")
    ap.add_argument("--ip", default=None)
    args = ap.parse_args()

    from trajectory import DEFAULTS
    p = {k: DEFAULTS[k] * args.lento for k in ("t_back", "t_down")}
    traj = swing_trajectory(p)
    qs, conv = joint_trajectory(traj)
    ok, _ = check_feasibility(traj, qs)
    if not (ok and conv):
        raise SystemExit("  trajetória NÃO viável — não é enviada ao robô")

    start = pose_sdk(traj["pos"][0], traj["rpy"][0])
    print(f"  início: {np.round(start, 1)}   duração: {traj['t'][-1]:.1f} s   método: {args.metodo}")
    if not args.go:
        print("  ENSAIO — nada foi enviado. Usa --go para executar.")
        return
    if input("  Robô livre e emergência à mão? Escreve 'sim': ").strip().lower() != "sim":
        return

    arm = connect(args.ip)
    try:
        check(arm.set_position(*start, speed=APPROACH_SPEED, wait=True), "set_position (início)")
        if args.metodo == "pontos":
            run_pontos(arm, traj, every=10)
        else:
            run_servo(arm, traj)
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
