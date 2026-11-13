"""
Leva o robô a uma pose de juntas, junta a junta, devagar (para fotos / endereço).

Uso (a partir de Lab2/):
    python src/ir_para_pose.py --enderecio-perto      # endereço do GOLFISTA_PERTO [0, 48.8, 80, 0, 29.5, 0]
    python src/ir_para_pose.py --q 0 48.8 80 0 29.5 0  # qualquer pose [graus]

Vai em linha reta no espaço das juntas (como a aproximação do run_robot.py):
a partir do ZERO é seguro; de outra pose, ir primeiro ao zero (ir_para_zero.py).
"""
import argparse

import numpy as np

from robot import check, connect
from trajectory import GOLFISTA_PERTO

SPEED = 20.0  # deg/s


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--q", type=float, nargs=6, help="6 ângulos [graus]")
    ap.add_argument("--enderecio-perto", action="store_true")
    ap.add_argument("--ip", default=None)
    args = ap.parse_args()
    q = list(GOLFISTA_PERTO["juntas"]["q_enderecio"]) if args.enderecio_perto else args.q
    if q is None:
        raise SystemExit("  indica --q ou --enderecio-perto")
    arm = connect(args.ip)
    try:
        atual = np.round(arm.get_servo_angle()[1][:6], 1)
        print(f"  juntas atuais: {atual}")
        print(f"  destino      : {np.round(q, 1)}  a {SPEED:.0f} deg/s")
        if input("  Robô livre e emergência à mão? Escreve 'sim': ").strip().lower() != "sim":
            return
        check(arm.set_servo_angle(angle=q, speed=SPEED, wait=True), "set_servo_angle")
        print(f"  chegou: {np.round(arm.get_servo_angle()[1][:6], 1)}")
    finally:
        arm.disconnect()


if __name__ == "__main__":
    main()
