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

from robot import check, connect, controller_tcp
from trajectory import DEFAULTS, check_feasibility, command_poses, joint_trajectory, swing_trajectory

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "data")
APPROACH_SPEED = 50.0  # mm/s, para ir até ao início da trajetória


def run_pontos(arm, traj, cmds, every):
    idx = list(range(0, len(traj["t"]), every)) + [len(traj["t"]) - 1]
    t, pos = traj["t"], traj["pos"]
    for a, b in zip(idx[:-1], idx[1:]):
        speed = np.linalg.norm(pos[b] - pos[a]) / (t[b] - t[a])
        check(arm.set_position(*cmds[b], radius=5, speed=max(speed, 1.0), wait=False), "set_position")
    arm.set_position(*cmds[-1], speed=APPROACH_SPEED, wait=True)


def run_servo(arm, traj, cmds):
    arm.set_mode(1)
    arm.set_state(0)
    time.sleep(0.5)
    dt = traj["t"][1] - traj["t"][0]
    log = []
    t0 = time.perf_counter()
    for k, cmd in enumerate(cmds):
        check(arm.set_servo_cartesian(cmd), "set_servo_cartesian")
        code, meas = arm.get_position()
        log.append([time.perf_counter() - t0, *cmd, *meas])
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
    ap.add_argument("--garra", type=float, default=DEFAULTS["garra"], help="mm, flange -> ponta (0 = sem pinça)")
    ap.add_argument("--ip", default=None)
    args = ap.parse_args()

    p = {k: DEFAULTS[k] * args.lento for k in ("t_back", "t_down")}
    traj = swing_trajectory(dict(p, garra=args.garra))
    qs, conv = joint_trajectory(traj)
    ok, _ = check_feasibility(traj, qs)
    if not (ok and conv):
        raise SystemExit("  trajetória NÃO viável — não é enviada ao robô")

    print(f"  pinça: {args.garra} mm   duração: {traj['t'][-1]:.1f} s   método: {args.metodo}")
    print(f"  ponta da pinça no início: {np.round(traj['pos'][0], 1)} mm (= bola)")
    if not args.go:
        flange = command_poses(traj)[0]
        print(f"  pose a enviar se o controlador NÃO tiver tcp_offset (flange): {np.round(flange, 1)}")
        print("  ENSAIO — nada foi enviado. Usa --go para executar.")
        return

    arm = connect(args.ip)
    try:
        # o SDK move o TCP configurado no controlador: converte para esse ponto
        tcp = np.round(arm.tcp_offset[:6], 2)
        cmds = command_poses(traj, controller_tcp(arm))
        print(f"  tcp_offset do controlador: {tcp}")
        if abs(tcp[2]) > 1e-6 and abs(tcp[2] - args.garra) > 1.0:
            print(f"  AVISO: o controlador usa z = {tcp[2]} mm e o código assume pinça de {args.garra} mm."
                  " A ponta vai passar na bola na mesma, mas confirma qual é a ferramenta montada.")
        print(f"  primeira pose enviada: {np.round(cmds[0], 1)}")
        if input("  Robô livre e emergência à mão? Escreve 'sim': ").strip().lower() != "sim":
            return
        check(arm.set_position(*cmds[0], speed=APPROACH_SPEED, wait=True), "set_position (início)")
        if args.metodo == "pontos":
            run_pontos(arm, traj, cmds, every=10)
        else:
            run_servo(arm, traj, cmds)
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
