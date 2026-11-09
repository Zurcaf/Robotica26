"""
Tarefa 4 — Precisão da IK do robô: IK do SDK -> a nossa FK (Tarefa 1) -> erro.

Uso (a partir de Lab2/):
    python src/ik_compare.py --n 60          # precisa do robô ligado (a IK corre no controlador)
    python src/ik_compare.py --from-csv      # refaz gráficos a partir de data/ik_compare.csv

Para N pontos espalhados por toda a trajetória da Tarefa 2:
  1. q_sdk   = arm.get_inverse_kinematics(pose_ref)        (IK do robô)
  2. pose_fk = fk_pose(q_sdk)                               (a nossa FK)
  3. erro de posição |p_fk - p_ref| [mm] e de orientação [deg]
Também se pede arm.get_forward_kinematics(q_sdk) — se coincidir com a nossa FK,
a tabela DH da Tarefa 1 fica validada contra o controlador.

As poses referem-se ao TCP que o controlador está a usar (arm.tcp_offset, p.ex.
a ponta da pinça): a nossa FK compõe-se com esse mesmo offset (fk(q)·T_tcp),
para comparar o mesmo ponto. O offset fica gravado nas últimas colunas do CSV.

Não mexe o robô (não ativa os motores).
"""
import argparse
import os

import numpy as np

from kinematics import R_from_rpy, fk_pose, orientation_error_deg
from robot import check, connect, controller_tcp
from trajectory import command_poses, swing_trajectory

HERE = os.path.dirname(os.path.abspath(__file__))
CSV = os.path.join(HERE, "..", "data", "ik_compare.csv")
REPORT = os.path.join(HERE, "..", "report")
HEADER = ("t,x,y,z,roll,pitch,yaw,q1,q2,q3,q4,q5,q6,"
          "x_fk,y_fk,z_fk,roll_fk,pitch_fk,yaw_fk,x_sdk,y_sdk,z_sdk,roll_sdk,pitch_sdk,yaw_sdk,"
          "tcp_x,tcp_y,tcp_z,tcp_roll,tcp_pitch,tcp_yaw")


def collect(n, ip):
    traj = swing_trajectory()
    idx = np.linspace(0, len(traj["t"]) - 1, n).astype(int)
    arm = connect(ip, enable_motion=False)
    rows = []
    try:
        tcp = list(arm.tcp_offset[:6])
        T_ctrl = controller_tcp(arm)
        print(f"  tcp_offset do controlador: {np.round(tcp, 2)}")
        cmds = command_poses(traj, T_ctrl)
        for k in idx:
            ref = cmds[k]
            code, q = arm.get_inverse_kinematics(ref, input_is_radian=False, return_is_radian=False)
            check(code, f"get_inverse_kinematics (ponto {k})")
            code, sdk = arm.get_forward_kinematics(q, input_is_radian=False, return_is_radian=False)
            check(code, "get_forward_kinematics")
            rows.append([traj["t"][k], *ref, *q[:6], *fk_pose(q[:6], T_ctrl), *sdk[:6], *tcp])
    finally:
        arm.disconnect()
    os.makedirs(os.path.dirname(CSV), exist_ok=True)
    np.savetxt(CSV, np.array(rows), delimiter=",", fmt="%.6f", header=HEADER)
    print(f"  {len(rows)} pontos gravados em {os.path.normpath(CSV)}")
    return np.array(rows)


def analyse(rows):
    import matplotlib.pyplot as plt

    t, ref, q, pfk, psdk = rows[:, 0], rows[:, 1:7], rows[:, 7:13], rows[:, 13:19], rows[:, 19:25]
    e_pos = np.linalg.norm(pfk[:, :3] - ref[:, :3], axis=1)
    e_rot = np.array([orientation_error_deg(R_from_rpy(*a[3:]), R_from_rpy(*b[3:])) for a, b in zip(pfk, ref)])
    e_fk = np.linalg.norm(pfk[:, :3] - psdk[:, :3], axis=1)

    print(f"  IK do robô + nossa FK vs referência: posição  média {e_pos.mean():.4f}  máx {e_pos.max():.4f} mm")
    print(f"                                       orient.  média {e_rot.mean():.4f}  máx {e_rot.max():.4f} deg")
    print(f"  nossa FK vs FK do controlador      : posição  máx {e_fk.max():.4f} mm")

    d = pfk[:, :3] - ref[:, :3]
    e_sdk = np.linalg.norm(psdk[:, :3] - ref[:, :3], axis=1)
    virado = q[:, 4] < 0                       # solução de pulso virado (J5 < 0)
    print(f"  FK do robô (IK do robô) vs referência: posição  máx {e_sdk.max():.4f} mm")
    print(f"  viés da nossa FK (x, y, z)         : {np.round(d.mean(axis=0), 2)} mm")
    print(f"  pulso virado (J5 < 0)              : {virado.sum()} de {len(q)} pontos")

    os.makedirs(REPORT, exist_ok=True)
    cor = ["#2a78d6", "#eb6834", "#1baf7a"]
    fig, ax = plt.subplots(3, 1, figsize=(8, 7), sharex=True)
    ax[0].plot(t, e_pos, "o-", ms=3, lw=2, color=cor[0], label="IK do robô + nossa FK")
    ax[0].plot(t, e_sdk, "o-", ms=3, lw=2, color=cor[1], label="IK do robô + FK do robô")
    ax[0].plot(t[virado], e_pos[virado], "o", ms=7, mfc="none", mec="k", mew=0.9, label="pulso virado (J5 < 0)")
    ax[0].set_ylabel("erro de posição [mm]")
    ax[0].legend(fontsize=8, frameon=False, ncol=3, loc="lower center", bbox_to_anchor=(0.5, 1.0))
    for i, n in enumerate("xyz"):
        ax[1].plot(t, d[:, i], "o-", ms=3, lw=2, color=cor[i], label=f"$\\Delta {n}$")
    ax[1].axhline(0, color="k", lw=0.6)
    ax[1].set_ylabel("nossa FK − pedido [mm]")
    ax[1].legend(fontsize=8, frameon=False, ncol=3, loc="upper right")
    ax[2].plot(t, e_rot, "o-", ms=3, lw=2, color=cor[0])
    ax[2].set_ylabel("erro de orientação [deg]")
    ax[2].set_xlabel("t [s]")
    for a in ax:
        a.grid(alpha=0.25, lw=0.5)
        a.spines[["top", "right"]].set_visible(False)
    fig.tight_layout()
    out = os.path.join(REPORT, "ik_erros.png")
    fig.savefig(out, dpi=170)
    print(f"  gráfico em {os.path.normpath(out)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--n", type=int, default=60, help="número de pontos da trajetória")
    ap.add_argument("--from-csv", action="store_true")
    ap.add_argument("--ip", default=None)
    args = ap.parse_args()
    rows = np.loadtxt(CSV, delimiter=",") if args.from_csv else collect(args.n, args.ip)
    analyse(rows)


if __name__ == "__main__":
    main()
