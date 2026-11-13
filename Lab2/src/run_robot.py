"""
Tarefa 3 — O Lite 6 segue a trajetória de swing da Tarefa 2.

Uso (a partir da pasta acima de src/):
    python src/run_robot.py --metodo servo --lento 2 --golfista --perto        # só verifica, não mexe
    python src/run_robot.py --go --metodo servo --lento 2 --golfista --perto   # metade da velocidade
    python src/run_robot.py --go --metodo servo --lento 1 --golfista --perto   # velocidade planeada

Sem --go o script gera e verifica a trajetória e mostra o que enviaria, sem se
ligar ao robô. Com --go liga-se, pede confirmação, leva o robô à pose inicial
junta a junta e executa o swing.

Métodos (ambos só com funções do SDK):
  servo   set_servo_cartesian() em modo 1, uma pose a cada 10 ms. O perfil
          temporal é o nosso, incluindo a aceleração constante do downswing.
          É o método usado no relatório.
  pontos  set_position() em modo 0, um ponto em cada dez, com concordância
          entre eles; o controlador interpola e impõe o seu próprio perfil.

--lento N estica o tempo N vezes, mantendo o caminho. Convém começar com N >= 2.
Em modo servo a pose medida em cada ciclo (get_position) é gravada em
data/run_servo_lento<N>_<trajetória>.csv, sem nunca escrever por cima de um
ensaio anterior.

Segurança: a trajetória só é enviada se check_feasibility() passar. Ter sempre
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
APPROACH_SPEED = 50.0        # mm/s, velocidade do último ponto (fim da trajetória)
APPROACH_JOINT_SPEED = 20.0  # graus/s, aproximação junta a junta à pose inicial
# Com o método pontos à velocidade planeada, a concordância entre pontos pedia
# às juntas mais velocidade do que a trajetória e o controlador parava (C24).
PONTOS_LENTO_MIN = 2.0


def run_pontos(arm, traj, cmds, every):
    """Envia um ponto em cada `every`, com a velocidade média do troço até ele."""
    idx = list(range(0, len(traj["t"]), every)) + [len(traj["t"]) - 1]
    t, pos = traj["t"], traj["pos"]
    for a, b in zip(idx[:-1], idx[1:]):
        speed = np.linalg.norm(pos[b] - pos[a]) / (t[b] - t[a])
        check(arm.set_position(*cmds[b], radius=5, speed=max(speed, 1.0), wait=False), "set_position")
    arm.set_position(*cmds[-1], speed=APPROACH_SPEED, wait=True)


def run_servo(arm, traj, cmds, nome="run_servo"):
    """Envia uma pose a cada dt e grava, em cada ciclo, a pose pedida e a medida."""
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
                pass                    # espera ativa até ao instante seguinte
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
    ap.add_argument("--phi-end", type=float, default=DEFAULTS["phi_end"], help="graus, fim do follow-through")
    ap.add_argument("--k-phi", type=float, default=DEFAULTS["k_phi"], help="0..1, quanto do arco roda a pinça")
    ap.add_argument("--golfista", action="store_true", help="swing desenhado nas juntas (preset GOLFISTA)")
    ap.add_argument("--perto", action="store_true", help="com --golfista: preset GOLFISTA_PERTO, o do relatório")
    ap.add_argument("--enderecio", type=float, nargs=6, metavar="J", default=None,
                    help="com --golfista: pose de endereço [graus] lida no robô com a ponta na bola (J4 e J6 passam a 0)")
    ap.add_argument("--ip", default=None)
    args = ap.parse_args()

    if args.metodo == "pontos" and args.lento < PONTOS_LENTO_MIN:
        raise SystemExit(f"  o método pontos só é seguro com --lento >= {PONTOS_LENTO_MIN:g}; "
                         "para a velocidade planeada usa --metodo servo")

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
        # O robô pode estar numa pose qualquer e uma reta no espaço cartesiano pode
        # passar por sítios imprevistos; a aproximação faz-se no espaço das juntas.
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
            if os.path.exists(os.path.join(DATA, f"{nome}.csv")):   # não escrever por cima
                nome += time.strftime("_%H%M%S")
            run_servo(arm, traj, cmds, nome=nome)
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
