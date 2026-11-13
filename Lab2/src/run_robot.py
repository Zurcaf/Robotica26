"""
Tarefa 3 — O Lite 6 segue a trajetória de swing da Tarefa 2.

Comando usado no relatório (trajetória final):
    python src/run_robot.py --go --metodo servo --lento 2 --golfista --perto   # metade da velocidade
    python src/run_robot.py --go --metodo servo --lento 1 --golfista --perto   # velocidade planeada

Uso (a partir de Lab2/):
    python src/run_robot.py                    # ENSAIO: verifica e mostra o que faria, não mexe
    python src/run_robot.py --go               # executa no robô (pede confirmação)
    python src/run_robot.py --go --metodo servo --lento 2
    python src/run_robot.py --go --metodo servo --lento 2 --golfista   # swing GOLFISTA (juntas)

Métodos (ambos com as funções do SDK):
  pontos  set_position() com blending (radius) em modo 0: o controlador
          interpola entre waypoints subamostrados. Mais robusto.
  servo   set_servo_cartesian() em modo 1, um ponto a cada dt: segue o perfil
          temporal à risca (é o que preserva a aceleração do downswing).

--lento N estica o tempo N vezes (velocidades / N) — começar SEMPRE com N >= 3.
Em servo grava-se a pose medida em data/run_servo_lento<N>_phi<fim>.csv (ou
..._golfista.csv; nunca escreve por cima: se já existir, acrescenta a hora),
para comparar com a de referência no relatório.

Lições do lab (5/11, data/LAB_2026-11-05.md):
  - C31 (deteção de colisão) no fim do follow-through com phi_end = 70 ->
    phi_end passou a 50 por omissão;
  - C24 (velocidade acima do limite) com `pontos` a --lento 1: o blending entre
    waypoints pede mais às juntas do que o plano -> `pontos` só com --lento >= 2;
  - na primeira versão o robô atrasava-se à velocidade planeada; não era um
    limite do controlador (robot_info.py: limites = fabricante) mas a forma da
    trajetória. A final (--golfista --perto) foi seguida até 306 mm/s.

SEGURANÇA: a trajetória só é enviada se check_feasibility() passar. Ter sempre
a mão no botão de emergência.
"""
import argparse
import os
import time

import numpy as np

from robot import check, connect, controller_tcp
from trajectory import DEFAULTS, GOLFISTA, GOLFISTA_PERTO, check_feasibility, command_poses, joint_trajectory, swing_trajectory

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "data")
APPROACH_SPEED = 50.0  # mm/s, velocidade do último ponto (fim da trajetória)
APPROACH_JOINT_SPEED = 20.0  # deg/s, aproximação junta a junta ao início
PONTOS_LENTO_MIN = 2.0       # abaixo disto o método pontos deu C24 no lab


def run_pontos(arm, traj, cmds, every):
    idx = list(range(0, len(traj["t"]), every)) + [len(traj["t"]) - 1]
    t, pos = traj["t"], traj["pos"]
    for a, b in zip(idx[:-1], idx[1:]):
        speed = np.linalg.norm(pos[b] - pos[a]) / (t[b] - t[a])
        check(arm.set_position(*cmds[b], radius=5, speed=max(speed, 1.0), wait=False), "set_position")
    arm.set_position(*cmds[-1], speed=APPROACH_SPEED, wait=True)


def run_servo(arm, traj, cmds, nome="run_servo"):
    arm.set_mode(1)
    arm.set_state(0)
    time.sleep(0.5)
    dt = traj["t"][1] - traj["t"][0]
    log = []
    t0 = time.perf_counter()
    try:
        for k, cmd in enumerate(cmds):
            code = arm.set_servo_cartesian(cmd)
            if code != 0:
                # o código do SDK (1 = "robô em erro") não diz a causa: mostrar a do controlador
                print(f"  PAROU no ponto {k}/{len(cmds)} (t = {traj['t'][k]:.2f} s de {traj['t'][-1]:.2f} s)")
                print(f"  código SDK {code} | erro do controlador {arm.error_code} | aviso {arm.warn_code}")
                print(f"  pose pedida: {np.round(cmd, 1)}")
                print(f"  juntas atuais: {np.round(arm.get_servo_angle()[1][:6], 1)}")
                break
            code, meas = arm.get_position()
            log.append([time.perf_counter() - t0, *cmd, *meas])
            while time.perf_counter() - t0 < (k + 1) * dt:
                pass
    finally:
        # grava sempre o que foi recolhido, mesmo que o robô pare a meio
        if log:
            os.makedirs(DATA, exist_ok=True)
            out = os.path.join(DATA, f"{nome}.csv")
            np.savetxt(out, np.array(log), delimiter=",", fmt="%.4f",
                       header="t,x_ref,y_ref,z_ref,roll_ref,pitch_ref,yaw_ref,x,y,z,roll,pitch,yaw")
            print(f"  {len(log)} poses medidas gravadas em {os.path.normpath(out)}")
        arm.set_mode(0)
        arm.set_state(0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--go", action="store_true", help="mexe mesmo o robô")
    ap.add_argument("--metodo", choices=["pontos", "servo"], default="pontos")
    ap.add_argument("--lento", type=float, default=3.0, help="fator de abrandamento (>= 1)")
    ap.add_argument("--garra", type=float, default=DEFAULTS["garra"], help="mm, flange -> ponta (0 = sem pinça)")
    ap.add_argument("--phi-end", type=float, default=DEFAULTS["phi_end"],
                    help="graus, fim do follow-through (no lab o C31 aparecia em phi ~69)")
    ap.add_argument("--k-phi", type=float, default=DEFAULTS["k_phi"], help="0..1, quanto do arco roda a pinça")
    ap.add_argument("--golfista", action="store_true", help="modo juntas com o preset trajectory.GOLFISTA")
    ap.add_argument("--perto", action="store_true", help="com --golfista: GOLFISTA_PERTO (cotovelo dobrado, bola a 347 mm)")
    ap.add_argument("--enderecio", type=float, nargs=6, metavar="J", default=None,
                    help="com --golfista: pose de endereço [graus] lida do Studio com a ponta na bola (J4 e J6 passam a 0)")
    ap.add_argument("--ip", default=None)
    args = ap.parse_args()

    if args.metodo == "pontos" and args.lento < PONTOS_LENTO_MIN:
        raise SystemExit(f"  o método pontos deu C24 (velocidade) a --lento 1 no lab: usa --lento >= "
                         f"{PONTOS_LENTO_MIN:g}, ou --metodo servo para a velocidade máxima")

    p = {k: DEFAULTS[k] * args.lento for k in ("t_back", "t_down")}
    p = dict(p, garra=args.garra, phi_end=args.phi_end, k_phi=args.k_phi)
    if args.golfista:
        g = GOLFISTA_PERTO if args.perto else GOLFISTA
        if args.enderecio is not None:
            q0 = [args.enderecio[0], args.enderecio[1], args.enderecio[2], 0.0, args.enderecio[4], 0.0]
            g = dict(g, juntas=dict(g["juntas"], q_enderecio=tuple(q0)))
        p = {**p, **g, "t_back": g["t_back"] * args.lento, "t_down": g["t_down"] * args.lento}
    traj = swing_trajectory(p)
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
        print(f"  aproximação junta a junta a {APPROACH_JOINT_SPEED:.0f} deg/s até q = {np.round(qs[0], 1)}")
        if input("  Robô livre e emergência à mão? Escreve 'sim': ").strip().lower() != "sim":
            return
        # o robô pode estar em qualquer pose: ir em linha reta no espaço pode passar
        # por sítios estranhos, por isso a aproximação é no espaço das juntas
        check(arm.set_servo_angle(angle=list(qs[0]), speed=APPROACH_JOINT_SPEED, wait=True), "set_servo_angle (aproximação)")
        check(arm.set_position(*cmds[0], speed=10.0, wait=True), "set_position (início)")
        if args.metodo == "pontos":
            run_pontos(arm, traj, cmds, every=10)
        else:
            tag = ("golfista_perto" if args.perto else "golfista") if args.golfista else f"phi{args.phi_end:g}"
            if args.enderecio is not None:
                tag += "_tampa"
            if args.k_phi != 1.0:
                tag += f"_kphi{args.k_phi:g}"
            nome = f"run_servo_lento{args.lento:g}_{tag}"
            if os.path.exists(os.path.join(DATA, f"{nome}.csv")):   # nunca escrever por cima
                nome += time.strftime("_%H%M%S")
            run_servo(arm, traj, cmds, nome=nome)
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
