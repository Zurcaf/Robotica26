"""
Swing V2 no robô: pega na caneta (o taco) e faz o swing junta a junta.

Uso (a partir de Lab2/):
    python src/run_robot_v2.py                     # ENSAIO: verifica tudo e mostra o plano, não mexe
    python src/run_robot_v2.py --go --lento 3      # pega na caneta + swing 3x mais lento
    python src/run_robot_v2.py --go --lento 1      # velocidade planeada
    python src/run_robot_v2.py --go --sem-caneta   # não mexe na pinça (caneta já presa, ou sem caneta)
    python src/run_robot_v2.py --soltar            # vai à pose de carregamento e abre a pinça

Sequência com --go:
  1. junta a junta (20 deg/s) até à POSE DE CARREGAR: braço para a frente,
     pinça na horizontal à altura do peito
  2. abre a pinça -> pões a caneta entre os dedos (a sair para a frente,
     `--taco` mm para lá das pontas) -> Enter -> fecha a pinça
  3. junta a junta até ao início do swing (endereço = pose de impacto)
  4. swing em modo servo (modo 1): um set_servo_angle_j a cada dt.
     Grava q pedido, q medido e pose medida em data/run_v2_lento<N>.csv
  5. volta à pose de carregar; pergunta se abre a pinça

Junta a junta não há IK nem o limite cartesiano (~220 mm/s) que apanhámos
com set_servo_cartesian; os limites são os do fabricante (verificados antes).
"""
import argparse
import os
import time

import numpy as np

from robot import check, connect
from swing_v2 import DEFAULTS, Q_LOAD, check_transition, check_v2, swing_v2, transition

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "data")
JOINT_SPEED_MOVE = 20.0  # deg/s nas deslocações


def move_joints(arm, q, what):
    check(arm.set_servo_angle(angle=[float(v) for v in q], speed=JOINT_SPEED_MOVE, wait=True), what)


def pegar_caneta(arm):
    move_joints(arm, Q_LOAD, "ir para a pose de carregar")
    check(arm.open_lite6_gripper(), "abrir pinça")
    time.sleep(1.0)
    input("  Põe a caneta entre os dedos (a sair para a frente) e carrega Enter... ")
    check(arm.close_lite6_gripper(), "fechar pinça")
    time.sleep(1.5)
    input("  Confirma que a caneta está bem presa (puxa levemente) e carrega Enter... ")


def soltar_caneta(arm, perguntar=True):
    move_joints(arm, Q_LOAD, "ir para a pose de carregar")
    if perguntar and input("  Abrir a pinça para tirar a caneta? (s/n) ").strip().lower() != "s":
        return
    input("  Segura a caneta e carrega Enter para abrir... ")
    check(arm.open_lite6_gripper(), "abrir pinça")
    time.sleep(1.0)
    arm.stop_lite6_gripper()


def run_swing(arm, tr, nome):
    arm.set_mode(1)
    arm.set_state(0)
    time.sleep(0.5)
    t, qs = tr["t"], tr["qs"]
    dt = t[1] - t[0]
    log = []
    t0 = time.perf_counter()
    try:
        for k, q in enumerate(qs):
            code = arm.set_servo_angle_j([float(v) for v in q], is_radian=False)
            if code != 0:
                print(f"  PAROU no ponto {k}/{len(qs)} (t = {t[k]:.2f} s de {t[-1]:.2f} s)")
                print(f"  código SDK {code} | erro do controlador {arm.error_code} | aviso {arm.warn_code}")
                print(f"  juntas pedidas: {np.round(q, 1)}")
                break
            _, qm = arm.get_servo_angle()
            _, pm = arm.get_position()
            log.append([time.perf_counter() - t0, *q, *qm[:6], *pm[:6]])
            while time.perf_counter() - t0 < (k + 1) * dt:
                pass
    finally:
        if log:
            os.makedirs(DATA, exist_ok=True)
            out = os.path.join(DATA, f"{nome}.csv")
            hdr = ("t," + ",".join(f"q{i}_ref" for i in range(1, 7)) + "," + ",".join(f"q{i}" for i in range(1, 7))
                   + ",x,y,z,roll,pitch,yaw")
            np.savetxt(out, np.array(log), delimiter=",", fmt="%.4f", header=hdr)
            print(f"  {len(log)} amostras gravadas em {os.path.normpath(out)}")
        arm.set_mode(0)
        arm.set_state(0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--go", action="store_true", help="mexe mesmo o robô")
    ap.add_argument("--lento", type=float, default=3.0, help="fator de abrandamento (>= 1)")
    ap.add_argument("--taco", type=float, default=DEFAULTS["taco"], help="mm de caneta para lá das pontas dos dedos")
    ap.add_argument("--sem-caneta", action="store_true", help="não abre/fecha a pinça")
    ap.add_argument("--soltar", action="store_true", help="só vai à pose de carregar e abre a pinça")
    ap.add_argument("--ip", default=None)
    args = ap.parse_args()

    if args.soltar:
        arm = connect(args.ip)
        try:
            soltar_caneta(arm, perguntar=False)
        finally:
            arm.disconnect()
        return

    tr = swing_v2(dict(taco=args.taco, t_back=DEFAULTS["t_back"] * args.lento, t_down=DEFAULTS["t_down"] * args.lento))
    ok, _ = check_v2(tr)
    ok &= check_transition(transition(Q_LOAD, tr["qs"][0], args.taco), "carregar -> início")
    ok &= check_transition(transition(tr["qs"][-1], Q_LOAD, args.taco), "fim -> carregar")
    if not ok:
        raise SystemExit("  swing V2 NÃO viável — não é enviado ao robô")

    print(f"  caneta: {args.taco} mm   duração: {tr['t'][-1]:.1f} s   bola em {np.round(tr['bola'], 1)} mm")
    print(f"  pose de carregar q = {Q_LOAD}")
    print(f"  início do swing  q = {np.round(tr['qs'][0], 1)}")
    if not args.go:
        print("  ENSAIO — nada foi enviado. Usa --go para executar.")
        return

    arm = connect(args.ip)
    try:
        if input("  Robô livre e emergência à mão? Escreve 'sim': ").strip().lower() != "sim":
            return
        if not args.sem_caneta:
            pegar_caneta(arm)
        move_joints(arm, tr["qs"][0], "ir para o início do swing")
        input("  Pronto para o swing. Carrega Enter para começar... ")
        run_swing(arm, tr, f"run_v2_lento{args.lento:g}")
        if not args.sem_caneta:
            soltar_caneta(arm)
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
