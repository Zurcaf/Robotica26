"""
Leva o braço à posição zero (todas as juntas a 0) por etapas, para evitar o
erro C22 (autocolisão) que o move_gohome dá quando parte de certas poses.

Uso (a partir da pasta acima de src/):
    python src/ir_para_zero.py           # mostra o plano e pede confirmação
    python src/ir_para_zero.py --ip 192.168.1.158

Porquê: move_gohome roda as 6 juntas ao mesmo tempo, em linha reta no espaço
das juntas. A pose zero tem o braço dobrado (antebraço encostado ao braço,
pinça ao lado da base); a meio do caminho a pinça/antebraço podem passar pelo
próprio robô e o controlador recusa (C22). Por etapas:
  1. esticar o antebraço   (J3 >= 60 deg, o resto parado)
  2. endireitar o pulso    (J4 = J5 = J6 = 0)
  3. alinhar base e ombro  (J1 = J2 = 0)
  4. dobrar até ao zero    (J3 = 0)
"""
import argparse

import numpy as np

import config as C

SPEED = 15.0  # deg/s


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default=None)
    args = ap.parse_args()

    from xarm.wrapper import XArmAPI

    arm = XArmAPI(args.ip or C.ROBOT_IP, is_radian=False)
    try:
        arm.clean_warn()
        arm.clean_error()
        arm.motion_enable(True)
        arm.set_mode(0)
        arm.set_state(0)
        q = np.array(arm.get_servo_angle()[1][:6], dtype=float)
        print(f"  juntas agora: {np.round(q, 1)}   (erro {arm.error_code})")

        etapas = []
        q1 = q.copy()
        q1[2] = max(q[2], 60.0)
        etapas.append(("esticar o antebraço (J3)", q1))
        q2 = q1.copy()
        q2[3:6] = 0.0
        etapas.append(("endireitar o pulso (J4, J5, J6)", q2))
        q3 = q2.copy()
        q3[0:2] = 0.0
        etapas.append(("alinhar base e ombro (J1, J2)", q3))
        etapas.append(("dobrar até ao zero (J3)", np.zeros(6)))

        for nome, alvo in etapas:
            print(f"  {nome:<34} -> {np.round(alvo, 1)}")
        print("  Na pose zero a pinça fica virada para baixo, ~7 cm acima da mesa, junto à base.")
        if input("  Nada debaixo do braço e emergência à mão? Escreve 'sim': ").strip().lower() != "sim":
            return

        for nome, alvo in etapas:
            print(f"  {nome}...")
            code = arm.set_servo_angle(angle=[float(v) for v in alvo], speed=SPEED, wait=True)
            if code != 0 or arm.error_code != 0:
                print(f"  PAROU em '{nome}': código {code}, erro do controlador {arm.error_code}")
                print(f"  juntas: {np.round(arm.get_servo_angle()[1][:6], 1)}")
                if arm.error_code == 22:
                    print("  C22 outra vez: no UFactory Studio, Live Control -> Manual Mode e arrasta o braço"
                          " à mão para uma pose aberta (braço esticado para a frente); depois corre isto de novo.")
                return
        print(f"  em zero: {np.round(arm.get_servo_angle()[1][:6], 2)}")
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
