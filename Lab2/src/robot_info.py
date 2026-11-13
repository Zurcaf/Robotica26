"""
Lê e imprime a configuração do controlador do Lite 6, sem mexer o robô.

Uso (a partir da pasta acima de src/):
    python src/robot_info.py            # imprime no terminal e grava data/robot_info.txt

Regista a versão do firmware, o desvio de ferramenta (tcp_offset), a carga
declarada, os limites de velocidade e aceleração configurados, a sensibilidade
de colisão e os parâmetros D-H guardados no controlador. Foi o que permitiu,
nas Tarefas 3 e 4, pôr de parte um limite de velocidade escondido e uma carga
mal configurada, e explicar a diferença para a nossa cinemática.
"""
import argparse
import os
from datetime import datetime

from robot import connect

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "data", "robot_info.txt")

# (nome a imprimir, atributo/método do XArmAPI)
ITEMS = [
    ("firmware / versão", "version"),
    ("número de série", "sn"),
    ("modo / estado", ("mode", "state")),
    ("erro / aviso", ("error_code", "warn_code")),
    ("tcp_offset [mm, deg]", "tcp_offset"),
    ("tcp_load [kg, mm]", "tcp_load"),
    ("limite vel. TCP [mm/s] (min, max)", "tcp_speed_limit"),
    ("limite acel. TCP [mm/s2]", "tcp_acc_limit"),
    ("limite vel. juntas [deg/s]", "joint_speed_limit"),
    ("limite acel. juntas [deg/s2]", "joint_acc_limit"),
    ("tcp_jerk / joint_jerk", ("tcp_jerk", "joint_jerk")),
    ("sensibilidade colisão (0-5)", "collision_sensitivity"),
    ("sensibilidade teach", "teach_sensitivity"),
    ("direção gravidade", "gravity_direction"),
    ("modo reduzido", "reduced_mode"),
    ("juntas atuais [deg]", "angles"),
    ("pose atual [mm, deg]", "position"),
]


def read(arm, attr):
    try:
        v = getattr(arm, attr)
        return v() if callable(v) else v
    except Exception as e:  # atributo pode não existir neste SDK/firmware
        return f"(indisponível: {type(e).__name__})"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ip", default=None)
    args = ap.parse_args()

    arm = connect(args.ip, enable_motion=False)
    lines = [f"# Lite 6 — configuração lida em {datetime.now():%Y-%m-%d %H:%M}"]
    try:
        for nome, attr in ITEMS:
            vals = [read(arm, a) for a in (attr if isinstance(attr, tuple) else (attr,))]
            lines.append(f"{nome:<36}: {' / '.join(str(v) for v in vals)}")
        # parâmetros DH do controlador (firmware recente): se existir, compara-se
        # com a tabela nominal do PDF (explica o desvio de ~1.5 mm da Tarefa 4)
        try:
            code, dh = arm.get_dh_params()
            lines.append(f"{'DH do controlador (code ' + str(code) + ')':<36}: {dh}")
        except Exception as e:
            lines.append(f"{'DH do controlador':<36}: (indisponível: {type(e).__name__})")
    finally:
        arm.disconnect()

    txt = "\n".join(lines)
    print(txt)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w", encoding="utf-8") as f:
        f.write(txt + "\n")
    print(f"\n  gravado em {os.path.normpath(OUT)}")


if __name__ == "__main__":
    main()
