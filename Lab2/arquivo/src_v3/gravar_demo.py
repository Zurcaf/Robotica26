"""
Grava uma demonstração feita à mão: põe o robô em modo manual (arrastar com as
mãos) e regista juntas e pose a 100 Hz até carregares Enter.

Uso (a partir de Lab2/):
    python src/gravar_demo.py               # grava em data/demo_<hora>.csv
    python src/gravar_demo.py --nome swing1

Substitui ler os números de um vídeo do UFactory Studio: dá ~100 amostras por
segundo, com 0.1 deg de resolução, prontas para a V3 (src/swing_v3.py).

ATENÇÃO: em modo manual o robô fica "solto" — segura-o antes de começar e não
o largues no ar. Fecha o UFactory Studio (só um programa pode estar ligado).
"""
import argparse
import os
import threading
import time

import numpy as np

from robot import connect

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "..", "data")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--nome", default=None)
    ap.add_argument("--hz", type=float, default=100.0)
    ap.add_argument("--ip", default=None)
    args = ap.parse_args()

    arm = connect(args.ip)
    parar = threading.Event()
    log = []
    try:
        input("  Segura o braço com as duas mãos e carrega Enter para o pôr em modo manual... ")
        arm.set_mode(2)      # 2 = modo manual (arrastar à mão)
        arm.set_state(0)
        input("  Modo manual. Carrega Enter para COMEÇAR a gravar... ")
        threading.Thread(target=lambda: (input("  A gravar... carrega Enter para PARAR. "), parar.set()),
                         daemon=True).start()
        t0 = time.perf_counter()
        k = 0
        while not parar.is_set():
            _, q = arm.get_servo_angle()
            _, pose = arm.get_position()
            log.append([time.perf_counter() - t0, *q[:6], *pose[:6]])
            k += 1
            while time.perf_counter() - t0 < k / args.hz:
                pass
    finally:
        try:
            input("  Segura o braço: carrega Enter para sair do modo manual (o robô fica parado)... ")
        except EOFError:
            pass
        arm.set_mode(0)
        arm.set_state(0)
        arm.disconnect()
        if log:
            os.makedirs(DATA, exist_ok=True)
            nome = args.nome or time.strftime("demo_%Y%m%d_%H%M%S")
            out = os.path.join(DATA, f"{nome}.csv")
            np.savetxt(out, np.array(log), delimiter=",", fmt="%.4f",
                       header="t,j1,j2,j3,j4,j5,j6,x,y,z,roll,pitch,yaw")
            print(f"  {len(log)} amostras ({log[-1][0]:.1f} s) gravadas em {os.path.normpath(out)}")


if __name__ == "__main__":
    main()
