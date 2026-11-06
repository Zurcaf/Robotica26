"""
Tarefa 2 — Trajetória de swing para o Lite 6.

Uso (a partir de Lab2/):
    python src/trajectory.py            # gera, verifica viabilidade e imprime o resumo
    python src/trajectory.py --plot     # + gráficos em report/traj_*.png
    python src/trajectory.py --t-down 1.0 --raio 180

ANALOGIA COM O LAB 1
--------------------
No Lab 1 o swing era um pêndulo duplo: braço (ombro) + pulso. Aqui a garra faz
de cabeça do taco:

  * "braço"  -> a garra percorre um ARCO de raio R num plano vertical, centrado
                num "ombro" virtual por cima da bola; phi = 0 na bola.
  * "pulso"  -> a orientação da garra roda phi + psi em torno da normal ao
                plano; psi é o "lag" do pulso, armado no topo e solto tarde
                (f_release), como no Lab 1.

PINÇA
-----
A "cabeça do taco" é a PONTA DA PINÇA (TCP), não o flange: traj["pos"] é a
ponta, que passa exatamente na bola. O flange fica `garra` mm atrás, ao longo
do z da ferramenta (traj["pos_flange"]) — é o flange que a IK resolve e é a
ponta + o flange que se verificam contra a mesa e a base.

Fases (iguais ao Lab 1):
  1. backswing   phi: 0 -> phi_top (quíntico, parte e chega parado)
  2. downswing   phi: phi_top -> 0 com aceleração constante,
                 phi(u) = phi_top (1 - (u/T)^2)  => velocidade máxima na bola
  3. follow-thr. desaceleração constante até parar em phi_end

Parâmetros por omissão em DEFAULTS. Ajusta-os até check_feasibility() passar
com folga (velocidades, alcance, limites das juntas, mesa e base).
"""
import argparse
import os

import numpy as np

import config as C
from kinematics import R_from_rpy, fk, ik_numeric, pose_from_T, rpy_from_R, tool_transform
from robot import pose_sdk

HERE = os.path.dirname(os.path.abspath(__file__))
REPORT = os.path.join(HERE, "..", "report")

DEFAULTS = dict(
    # bola a 190 mm (e não 250): com a pinça o flange fica ~60 mm mais longe da
    # base e a 250 mm o topo do swing saía do alcance.
    bola=(190.0, 0.0, 80.0),     # mm, posição fixa da bola (referencial da base)
    direcao=(0.0, 1.0, 0.0),     # direção do alvo (horizontal)
    raio=150.0,                  # mm, raio do arco ("comprimento do braço+taco")
    # topo/fim limitados por J5 (limite 124 deg): com -100/80 e pinça, J5 chegava a 124
    phi_top=-90.0,               # graus, ângulo no topo do backswing
    # fim do follow-through: com 70 deg o robô parava SEMPRE com C31 (deteção de
    # colisão) em phi ~69 deg, braço esticado para o lado (lab 5/11); com 50 deg
    # correu completo a lento 1, 1.5 e 2 (data/LAB_2026-11-05.md)
    phi_end=50.0,                # graus, ângulo final do follow-through
    psi_top=30.0,                # graus, "pulso armado" no topo
    f_release=0.6,               # fração do downswing em que o pulso solta
    t_back=2.0,                  # s, duração do backswing
    t_down=2.2,                  # s, duração do downswing (limitado pela vel. de J6)
    # orientação da garra na bola: para baixo, inclinada 45 deg. A pique
    # (180, 0, 0) põe J5 = 0 => singularidade do pulso, J4/J6 a ~1900 deg/s.
    rpy_enderecio=(180.0, 45.0, 0.0),
    garra=C.GRIPPER_TCP_Z,       # mm, flange -> ponta da pinça (0 = sem pinça)
    dt=0.01,                     # s, período de amostragem (100 Hz)
)


def quintic(s):
    """Perfil 0->1 com velocidade e aceleração nulas nos extremos."""
    s = np.clip(s, 0.0, 1.0)
    return 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5


def axis_rotation(n, ang):
    """Rodrigues: rotação de ang (rad) em torno do eixo unitário n."""
    n = n / np.linalg.norm(n)
    K = np.array([[0, -n[2], n[1]], [n[2], 0, -n[0]], [-n[1], n[0], 0]])
    return np.eye(3) + np.sin(ang) * K + (1 - np.cos(ang)) * K @ K


def swing_angles(p):
    """Devolve t, phi(t), psi(t) [rad]."""
    phi_top, phi_end = np.radians(p["phi_top"]), np.radians(p["phi_end"])
    psi_top = np.radians(p["psi_top"])
    Tb, Td, dt = p["t_back"], p["t_down"], p["dt"]

    w_imp = -2.0 * phi_top / Td                  # velocidade angular na bola
    T_f = 2.0 * phi_end / w_imp                  # desaceleração constante até parar
    t = np.arange(0.0, Tb + Td + T_f + dt / 2, dt)
    phi = np.zeros_like(t)
    psi = np.zeros_like(t)

    b = t <= Tb
    phi[b] = phi_top * quintic(t[b] / Tb)
    psi[b] = psi_top * quintic(t[b] / Tb)

    d = (t > Tb) & (t <= Tb + Td)
    u = t[d] - Tb
    phi[d] = phi_top * (1.0 - (u / Td) ** 2)
    t_rel = p["f_release"] * Td
    psi[d] = psi_top * (1.0 - quintic((u - t_rel) / (Td - t_rel)))

    f = t > Tb + Td
    u = t[f] - Tb - Td
    a = w_imp / T_f
    phi[f] = w_imp * u - 0.5 * a * u ** 2
    return t, phi, psi


def swing_trajectory(p=None):
    """
    Trajetória da ponta da pinça no espaço cartesiano.
    Devolve dict com t [s], pos (N,3) [mm] da ponta, pos_flange (N,3) [mm],
    rpy (N,3) [graus], R (N,3,3), phi, psi.
    """
    p = {**DEFAULTS, **(p or {})}
    t, phi, psi = swing_angles(p)
    B = np.asarray(p["bola"], float)
    d = np.asarray(p["direcao"], float)
    d /= np.linalg.norm(d)
    ez = np.array([0.0, 0.0, 1.0])
    n = np.cross(d, ez)                          # normal ao plano do swing
    centro = B + p["raio"] * ez

    pos = centro + p["raio"] * (np.sin(phi)[:, None] * d - np.cos(phi)[:, None] * ez)
    R0 = R_from_rpy(*p["rpy_enderecio"])
    # sinal: com phi < 0 (backswing) a garra roda "para trás", como o taco
    R = np.array([axis_rotation(-n, ph + ps) @ R0 for ph, ps in zip(phi, psi)])
    rpy = np.array([rpy_from_R(Ri) for Ri in R])
    pos_flange = pos - p["garra"] * R[:, :, 2]   # recua ao longo do z da ferramenta
    return dict(t=t, pos=pos, pos_flange=pos_flange, R=R,
                rpy=np.unwrap(rpy, period=360.0, axis=0), phi=phi, psi=psi, params=p)


def command_poses(traj, T_ctrl=None):
    """
    Poses [x, y, z, roll, pitch, yaw] a enviar ao controlador, para o TCP que ele
    estiver a usar (T_ctrl = robot.controller_tcp(arm); None = flange, sem offset).
    T_ponta = T_flange·T_garra  =>  T_enviar = T_ponta·T_garra⁻¹·T_ctrl.
    """
    T_ctrl = np.eye(4) if T_ctrl is None else T_ctrl
    M = np.linalg.inv(tool_transform(traj["params"]["garra"])) @ T_ctrl
    out = []
    for pos, R in zip(traj["pos"], traj["R"]):
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R, pos
        P = pose_from_T(T @ M)
        out.append(pose_sdk(P[:3], P[3:]))
    return out


def joint_trajectory(traj, q0=None):
    """IK numérica do FLANGE ponto a ponto (semente = ponto anterior => continuidade)."""
    q = np.zeros(6) if q0 is None else np.asarray(q0, float)
    qs, ok = [], True
    for pos, R in zip(traj["pos_flange"], traj["R"]):
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = R, pos
        q, conv = ik_numeric(T, q)
        ok &= conv
        qs.append(q)
    return np.array(qs), ok


def check_feasibility(traj, qs, verbose=True):
    """
    Verifica (i) velocidades, (ii) alcance, (iii) limites das juntas + colisões.
    Mesa e base: verifica a ponta da pinça, o flange e todas as juntas de J2 em diante.
    """
    t, pos, flange = traj["t"], traj["pos"], traj["pos_flange"]
    sf = C.SAFETY_FACTOR
    v_tcp = np.linalg.norm(np.gradient(pos, t, axis=0), axis=1)
    # com tcp_offset = 0 (o robô do lab) o controlador comanda o FLANGE, que anda
    # mais depressa do que a ponta por causa da rotação da garra
    v_flange = np.linalg.norm(np.gradient(flange, t, axis=0), axis=1)
    qd = np.gradient(qs, t, axis=0)
    ombro = fk(np.zeros(6), all_frames=True)[2][:3, 3]     # origem de {2}
    alcance = np.linalg.norm(flange - ombro, axis=1)       # o alcance é do flange
    lo = np.array([l for l, _ in C.JOINT_LIMITS])
    hi = np.array([h for _, h in C.JOINT_LIMITS])
    z_juntas = min(min(T[2, 3] for T in fk(q, all_frames=True)[2:]) for q in qs)
    z_min = min(pos[:, 2].min(), flange[:, 2].min(), z_juntas)
    r_xy = min(np.linalg.norm(pos[:, :2], axis=1).min(), np.linalg.norm(flange[:, :2], axis=1).min())

    checks = {
        "vel. TCP": (v_tcp.max(), sf * C.TCP_SPEED_MAX, "mm/s"),
        "vel. flange": (v_flange.max(), sf * C.TCP_SPEED_MAX, "mm/s"),
        "vel. juntas": (np.abs(qd).max(), sf * C.JOINT_SPEED_MAX, "deg/s"),
        "alcance": (alcance.max(), sf * C.REACH, "mm"),
    }
    res = {k: v[0] <= v[1] for k, v in checks.items()}
    m = C.JOINT_LIMIT_MARGIN
    res["limites juntas"] = bool(np.all((qs >= lo + m) & (qs <= hi - m)))
    folga = np.minimum(qs - lo, hi - qs).min(axis=0)       # folga mínima de cada junta
    res["acima da mesa"] = bool(z_min >= C.TABLE_Z + C.TABLE_CLEARANCE)
    res["fora da base"] = bool(r_xy >= C.BASE_RADIUS)

    if verbose:
        for k, (val, lim, un) in checks.items():
            print(f"  {k:<16}: max {val:8.1f} {un:<6} (limite c/ margem {lim:.1f})  {'OK' if res[k] else 'FALHA'}")
        j = folga.argmin()
        print(f"  {'limites juntas':<16}: menor folga {folga[j]:.1f} deg (J{j + 1}, mínimo {m:.0f})  "
              f"{'OK' if res['limites juntas'] else 'FALHA'}")
        print(f"  {'acima da mesa':<16}: z min {z_min:.1f} mm (ponta {pos[:, 2].min():.1f}, flange "
              f"{flange[:, 2].min():.1f}, juntas {z_juntas:.1f})  {'OK' if res['acima da mesa'] else 'FALHA'}")
        print(f"  {'fora da base':<16}: r min {r_xy:.1f} mm  {'OK' if res['fora da base'] else 'FALHA'}")
        if v_flange.max() > C.CART_SPEED_OBSERVED:
            print(f"  AVISO: o flange chega a {v_flange.max():.0f} mm/s; no lab o controlador não passou de "
                  f"~{C.CART_SPEED_OBSERVED:.0f} mm/s em modo servo cartesiano -> o robô vai ficar para trás "
                  f"(usar --lento >= {v_flange.max() / C.CART_SPEED_OBSERVED:.1f} para seguir à risca)")
    return all(res.values()), dict(v_tcp=v_tcp, v_flange=v_flange, qd=qd, alcance=alcance)


def plot(traj, qs, extra):
    import matplotlib.pyplot as plt

    os.makedirs(REPORT, exist_ok=True)
    t, pos = traj["t"], traj["pos"]

    fig, ax = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    ax[0].plot(t, pos)
    ax[0].set_ylabel("posição [mm]")
    ax[0].legend(["x", "y", "z"])
    ax[1].plot(t, traj["rpy"])
    ax[1].set_ylabel("orientação [deg]")
    ax[1].legend(["roll", "pitch", "yaw"])
    ax[2].plot(t, extra["v_tcp"])
    ax[2].axhline(C.TCP_SPEED_MAX, color="r", ls="--", label="limite fabricante")
    ax[2].set_ylabel("|v| TCP [mm/s]")
    ax[2].set_xlabel("t [s]")
    ax[2].legend()
    fig.tight_layout()
    fig.savefig(os.path.join(REPORT, "traj_cartesiana.png"), dpi=150)

    fig, ax = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    ax[0].plot(t, qs)
    ax[0].set_ylabel("q [deg]")
    ax[0].legend([f"J{i + 1}" for i in range(6)], ncol=6, fontsize=8)
    ax[1].plot(t, extra["qd"])
    ax[1].axhline(C.JOINT_SPEED_MAX, color="r", ls="--")
    ax[1].axhline(-C.JOINT_SPEED_MAX, color="r", ls="--")
    ax[1].set_ylabel("dq/dt [deg/s]")
    ax[1].set_xlabel("t [s]")
    fig.tight_layout()
    fig.savefig(os.path.join(REPORT, "traj_juntas.png"), dpi=150)

    fig = plt.figure(figsize=(7, 7))
    a3 = fig.add_subplot(projection="3d")
    a3.plot(*pos.T, label="ponta da pinça")
    a3.plot(*traj["pos_flange"].T, "--", label="flange")
    for k in range(0, len(pos), 25):             # a pinça, de 1/4 em 1/4 s
        a3.plot(*np.array([traj["pos_flange"][k], pos[k]]).T, color="gray", lw=0.8)
    a3.scatter(*traj["params"]["bola"], color="k", s=40, label="bola")
    th = np.linspace(0, 2 * np.pi, 40)
    for z in (0.0, 243.3):
        a3.plot(C.BASE_RADIUS * np.cos(th), C.BASE_RADIUS * np.sin(th), z, "gray")
    a3.set_xlabel("x [mm]")
    a3.set_ylabel("y [mm]")
    a3.set_zlabel("z [mm]")
    a3.legend()
    fig.savefig(os.path.join(REPORT, "traj_3d.png"), dpi=150)
    print(f"  gráficos em {os.path.normpath(REPORT)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--t-down", type=float, default=DEFAULTS["t_down"])
    ap.add_argument("--raio", type=float, default=DEFAULTS["raio"])
    ap.add_argument("--garra", type=float, default=DEFAULTS["garra"], help="mm, flange -> ponta (0 = sem pinça)")
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()

    traj = swing_trajectory(dict(t_down=args.t_down, raio=args.raio, garra=args.garra))
    qs, conv = joint_trajectory(traj)
    print(f"  {len(traj['t'])} pontos, {traj['t'][-1]:.2f} s, IK numérica {'convergiu' if conv else 'NÃO convergiu em todos os pontos'}")
    ok, extra = check_feasibility(traj, qs)
    print(f"  => trajetória {'VIÁVEL' if ok and conv else 'NÃO VIÁVEL'}")
    if args.plot:
        plot(traj, qs, extra)


if __name__ == "__main__":
    main()
