"""
Swing V3 no robô (o swing do colega, refinado), enviado junta a junta.

Uso (a partir de Lab2/):
    python src/run_robot_v3.py                  # ENSAIO: verifica tudo e mostra o plano, não mexe
    python src/run_robot_v3.py --go --lento 2   # executa 2x mais lento (começar assim)
    python src/run_robot_v3.py --go --lento 1   # tempos ajustados aos limites

Sequência com --go:
  1. aproximação por etapas a partir do ZERO, junta a junta a 20 deg/s (o
     endereço tem o pulso "virado", J4 = -231 deg): esticar o braço -> rodar o
     pulso longe da base (junto ao zero dá C22) -> inclinar o pulso -> endereço
  2. Enter -> swing em modo servo (modo 1), um set_servo_angle_j a cada dt
  3. grava q pedido, q medido e pose medida em data/run_v3_lento<N>.csv
     (nunca por cima; grava mesmo que o robô pare, e mostra o erro)
"""
import argparse
import os
import time

import numpy as np

from robot import check, connect
from swing_v3 import FOLGA_PONTA_MESA, Q_BOLA_LAB, check_v3, from_joints, parametros, swing_v3

import config as C

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "data")
JOINT_SPEED_MOVE = 20.0  # deg/s


# pose aberta onde se roda o pulso: braço esticado para a frente, pinça a >= 35 cm
# do eixo da base. Junto ao zero o controlador dá C22 (autocolisão) mal o J4
# começa a rodar (lab 6/11: parou com J4 = -6 deg) — a pinça está encostada à base.
J2_ABERTO, J3_ABERTO = 20.0, 100.0


def etapas(q_atual, q_alvo):
    """
    Aproximação a partir do zero (o inverso do ir_para_zero.py, que nunca deu erro):
      1. esticar o antebraço (J3)   2. inclinar o ombro (J2)
      3. rodar o pulso (J4, J6) já longe da base   4. inclinar o pulso (J5)
      5. braço até ao endereço (J1, J2, J3)
    Mover o braço primeiro com a pinça virada para baixo punha a ponta 2 cm abaixo
    da mesa; rodar o pulso junto ao zero dava C22.
    """
    a = np.array(q_atual, float)
    b = np.array(q_alvo, float)
    e1 = a.copy(); e1[2] = max(a[2], J3_ABERTO)
    e2 = e1.copy(); e2[1] = J2_ABERTO
    e3 = e2.copy(); e3[3] = b[3]; e3[5] = b[5]
    e4 = e3.copy(); e4[4] = b[4]
    return [("esticar o antebraço (J3)", e1), ("inclinar o ombro (J2)", e2),
            ("rodar o pulso (J4, J6)", e3), ("inclinar o pulso (J5)", e4), ("braço até ao endereço (J1-J3)", b)]


R_PULSO_MIN = 250.0  # mm: a pinça tem de estar a esta distância do eixo da base enquanto o pulso roda


def check_etapas(q_atual, q_alvo):
    """
    Mesa ao longo da aproximação (interpolação linear nas juntas, como o
    set_servo_angle) e distância da pinça à base enquanto o pulso (J4/J6) roda.
    """
    pts, q, r_pulso = [], np.array(q_atual, float), np.inf
    for _, alvo in etapas(q_atual, q_alvo):
        seg = [q + s * (alvo - q) for s in np.linspace(0, 1, 40)]
        if abs(alvo[3] - q[3]) > 1 or abs(alvo[5] - q[5]) > 1:     # etapa que roda o pulso
            t_ = from_joints(np.arange(40), np.array(seg))
            r_pulso = min(r_pulso, np.hypot(*t_["pos_flange"][:, :2].T).min(), np.hypot(*t_["pos"][:, :2].T).min())
        pts += seg
        q = alvo
    tr = from_joints(np.arange(len(pts)), np.array(pts))
    z_robo = min(tr["juntas"][:, :, 2].min(), tr["pos_flange"][:, 2].min())
    return z_robo, tr["pos"][:, 2].min(), r_pulso


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
            if os.path.exists(out):
                out = os.path.join(DATA, f"{nome}{time.strftime('_%H%M%S')}.csv")
            hdr = ("t," + ",".join(f"q{i}_ref" for i in range(1, 7)) + "," + ",".join(f"q{i}" for i in range(1, 7))
                   + ",x,y,z,roll,pitch,yaw")
            np.savetxt(out, np.array(log), delimiter=",", fmt="%.4f", header=hdr)
            print(f"  {len(log)} amostras gravadas em {os.path.normpath(out)}")
        arm.set_mode(0)
        arm.set_state(0)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--go", action="store_true", help="mexe mesmo o robô")
    ap.add_argument("--lento", type=float, default=2.0, help="fator de abrandamento (>= 1)")
    ap.add_argument("--bola-demo", action="store_true",
                    help="bola onde a demonstração passava (ensaios das 12:5x de 6/11); por omissão: a bola do lab")
    ap.add_argument("--ip", default=None)
    args = ap.parse_args()

    p = parametros(bola_demo=args.bola_demo)
    print(f"  bola: {'demonstração' if args.bola_demo else 'marcada no lab, q = ' + str(Q_BOLA_LAB)}")
    for k in ("t_back", "t_down", "t_follow"):
        p[k] *= args.lento
    tr = swing_v3(p)
    ok, _ = check_v3(tr)
    z_robo, z_ponta, r_pulso = check_etapas(np.zeros(6), tr["qs"][0])
    ok_ap = (z_robo >= C.TABLE_Z + C.TABLE_CLEARANCE and z_ponta >= C.TABLE_Z + FOLGA_PONTA_MESA
             and r_pulso >= R_PULSO_MIN)
    print(f"  aproximação a partir do zero: robô z mín {z_robo:.1f} mm, ponta z mín {z_ponta:.1f} mm, "
          f"pinça a {r_pulso:.0f} mm da base ao rodar o pulso (mín {R_PULSO_MIN:.0f})  {'OK' if ok_ap else 'FALHA'}")
    if not (ok and ok_ap):
        raise SystemExit("  swing V3 NÃO viável — não é enviado ao robô")

    print(f"  duração {tr['t'][-1]:.1f} s (backswing {p['t_back']:.1f}, downswing {p['t_down']:.1f}, "
          f"follow-through {p['t_follow']:.1f}); bola em {np.round(tr['bola'], 1)} mm")
    print(f"  endereço q = {np.round(tr['qs'][0], 1)}")
    if not args.go:
        print("  ENSAIO — nada foi enviado. Usa --go para executar. Começa com o robô no zero (src/ir_para_zero.py).")
        return

    arm = connect(args.ip)
    try:
        q_atual = np.array(arm.get_servo_angle()[1][:6], float)
        print(f"  juntas agora: {np.round(q_atual, 1)}")
        if np.abs(q_atual).max() > 5.0:
            print("  AVISO: o robô não está no zero; a aproximação foi verificada a partir do zero."
                  " Recomendo correr primeiro src/ir_para_zero.py.")
        if input("  Robô livre, nada à frente dele e emergência à mão? Escreve 'sim': ").strip().lower() != "sim":
            return
        for nome, alvo in etapas(q_atual, tr["qs"][0]):
            print(f"  aproximação: {nome}...")
            check(arm.set_servo_angle(angle=[float(v) for v in alvo], speed=JOINT_SPEED_MOVE, wait=True), nome)
        input("  No endereço (a ponta está no sítio da bola). Carrega Enter para o swing... ")
        run_swing(arm, tr, f"run_v3_{'' if args.bola_demo else 'bolalab_'}lento{args.lento:g}")
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
