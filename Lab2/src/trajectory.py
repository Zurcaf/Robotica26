"""
Tarefa 2 — Trajetória de swing para o Lite 6.

Uso (a partir da pasta acima de src/):
    python src/trajectory.py --golfista --perto          # trajetória do relatório
    python src/trajectory.py --golfista --perto --plot   # + gráficos em report/
    python src/trajectory.py                             # primeira versão (arco cartesiano)

Em todos os casos a trajetória é gerada, verificada (velocidades, aceleração das
juntas, alcance, limites das juntas, mesa e base) e resumida no terminal.

Trajetória do relatório (preset GOLFISTA_PERTO)
-----------------------------------------------
O swing é desenhado nas juntas, como o golfista do Lab 1: a base (J1) faz de
tronco, o ombro (J2) levanta o braço e J5 faz de pulso. O cotovelo fica fixo e
J4 = J6 = 0. A posição e a orientação da pinça em função do tempo, que é o que
o enunciado pede, saem da cinemática direta destes ângulos.

Primeira versão (DEFAULTS, sem opções)
--------------------------------------
A ponta da pinça percorre um arco vertical por cima da bola e a pinça inclina
em torno de um eixo perpendicular a si própria e à direção do swing, para não
girar sobre si mesma. As juntas obtêm-se com a IK numérica de kinematics.py.

Nos dois casos as fases e as leis de movimento são as do Lab 1, com phi o
ângulo do "braço" (0 na bola) e psi o "pulso", que arma no topo e solta tarde:
  1. backswing       phi: 0 -> phi_top, polinómio de grau 5 (parte e chega parado)
  2. downswing       phi: phi_top -> 0 com aceleração constante,
                     phi(u) = phi_top (1 - (u/T)^2), velocidade máxima na bola
  3. follow-through  desaceleração constante até parar em phi_end

A ponta da pinça (traj["pos"]) é que passa na bola; o flange fica `garra` mm
atrás, ao longo do z da ferramenta (traj["pos_flange"]).
"""
import argparse
import os

import numpy as np

import config as C
from kinematics import R_from_rpy, fk, ik_numeric, pose_from_T, rpy_from_R, tool_transform
from robot import pose_sdk

HERE = os.path.dirname(os.path.abspath(__file__))
REPORT = os.path.join(HERE, "..", "report")

# Parâmetros da primeira versão (arco cartesiano). Os presets GOLFISTA abaixo
# substituem só o que muda.
DEFAULTS = dict(
    # Bola a 190 mm da base: com a pinça montada, o topo do swing saía do
    # alcance com a bola mais longe.
    bola=(190.0, 0.0, 80.0),     # mm, posição fixa da bola (referencial da base)
    direcao=(0.0, 1.0, 0.0),     # direção do swing (horizontal)
    raio=150.0,                  # mm, raio do arco, centrado por cima da bola
    phi_top=-90.0,               # graus, ângulo no topo do backswing
    # Com 70 graus o controlador parava no fim do follow-through com um erro de
    # deteção de colisão (C31); com 50 nunca aconteceu.
    phi_end=50.0,                # graus, fim do follow-through
    psi_top=30.0,                # graus, pulso armado no topo
    f_release=0.6,               # fração do downswing em que o pulso solta
    # Quanto da rotação do arco entra na orientação da pinça (1 = toda).
    k_phi=1.0,
    juntas=None,                 # modo "juntas" (presets GOLFISTA); None = arco cartesiano
    t_back=2.0,                  # s, duração do backswing
    t_down=2.2,                  # s, duração do downswing
    # Orientação da pinça na bola: para baixo e inclinada 45 graus. A pique
    # (180, 0, 0) J5 ficaria a 0, que é uma singularidade do pulso.
    rpy_enderecio=(180.0, 45.0, 0.0),
    garra=C.GRIPPER_TCP_Z,       # mm, flange -> ponta da pinça (0 = sem pinça)
    dt=0.01,                     # s, período de amostragem (100 Hz)
)

# Modo "juntas", com o braço quase esticado (J3 = 110 graus) e a bola a 390 mm.
# Cada junta segue um ganho sobre phi, a partir da pose de endereço:
#   J1 = q1 + a_tronco·phi,   J2 = q2 - b_ombro·(1 - cos phi),   J5 = q5 + a_pulso·psi
GOLFISTA = dict(
    juntas=dict(
        q_enderecio=(0.0, 65.5, 110.0, 0.0, 50.0, 0.0),   # ponta em (390, 0, 80), pinça vertical
        a_tronco=-0.8,     # backswing para +y, impacto ao longo de -y
        b_ombro=35.0,      # o braço sobe no backswing e no follow-through
        b_cotovelo=0.0,
        a_pulso=-1.0,
    ),
    phi_top=-90.0, phi_end=50.0,
    t_back=4.5, t_down=4.0,      # backswing mais lento: a velocidade máxima fica no impacto
)

# Trajetória do relatório: cotovelo mais dobrado (J3 = 80 graus) e bola a 347 mm,
# longe da singularidade do braço esticado. O pulso arma no sentido que afasta J5
# de zero (fica entre 29.5 e 59.5 graus); com a_pulso = -1 passava pela
# singularidade do pulso no topo do backswing.
GOLFISTA_PERTO = dict(
    GOLFISTA,
    juntas=dict(GOLFISTA["juntas"], q_enderecio=(0.0, 48.8, 80.0, 0.0, 29.5, 0.0), a_pulso=1.0),
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
    Trajetória da pinça. p são alterações aos DEFAULTS (por exemplo um preset).
    Devolve um dict com t [s], pos e pos_flange (N,3) [mm], R (N,3,3),
    rpy (N,3) [graus], phi, psi e, no modo juntas, qs (N,6) [graus].
    """
    p = {**DEFAULTS, **(p or {})}
    t, phi, psi = swing_angles(p)
    if p["juntas"] is not None:
        return _swing_juntas(p, t, phi, psi)
    B = np.asarray(p["bola"], float)
    d = np.asarray(p["direcao"], float)
    d /= np.linalg.norm(d)
    centro, raio = B + p["raio"] * np.array([0.0, 0.0, 1.0]), p["raio"]
    u = (B - centro) / raio                      # do centro do arco para a bola (para baixo)

    pos = centro + raio * (np.sin(phi)[:, None] * d + np.cos(phi)[:, None] * u)
    R0 = R_from_rpy(*p["rpy_enderecio"])
    # A pinça roda em torno de um eixo perpendicular a si própria e à direção do
    # swing: inclina para o lado, como um taco, sem girar sobre o seu eixo.
    n2 = np.cross(R0[:, 2], d)
    n2 /= np.linalg.norm(n2)
    R = np.array([axis_rotation(n2, p["k_phi"] * ph + ps) @ R0 for ph, ps in zip(phi, psi)])
    rpy = np.array([rpy_from_R(Ri) for Ri in R])
    pos_flange = pos - p["garra"] * R[:, :, 2]   # recua ao longo do z da ferramenta
    return dict(t=t, pos=pos, pos_flange=pos_flange, R=R, centro=centro, raio=raio,
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


def _swing_juntas(p, t, phi, psi):
    """Modo "juntas": ângulos das juntas a partir de phi e psi -> FK -> trajetória da pinça."""
    j = p["juntas"]
    phd = np.degrees(phi)
    qs = np.tile(np.asarray(j["q_enderecio"], float), (len(t), 1))
    qs[:, 0] += j["a_tronco"] * phd                               # tronco
    qs[:, 1] -= j["b_ombro"] * (1.0 - np.cos(phi))                # ombro
    qs[:, 2] -= j.get("b_cotovelo", 0.0) * (1.0 - np.cos(phi))    # cotovelo (0 nos presets)
    qs[:, 4] += j.get("a_pulso", 0.0) * np.degrees(psi)           # pulso
    Tg = tool_transform(p["garra"])
    Tf = np.array([fk(q) for q in qs])
    Tp = Tf @ Tg
    pos, pos_flange, R = Tp[:, :3, 3], Tf[:, :3, 3], Tp[:, :3, :3]
    rpy = np.array([rpy_from_R(Ri) for Ri in R])
    k0 = int(np.argmin(np.abs(phi)))               # phi = 0 é a bola
    p = dict(p, bola=tuple(pos[k0]))
    return dict(t=t, pos=pos, pos_flange=pos_flange, R=R, centro=None, raio=None, qs=qs,
                rpy=np.unwrap(rpy, period=360.0, axis=0), phi=phi, psi=psi, params=p)


def joint_trajectory(traj, q0=None):
    """
    Juntas ao longo da trajetória. No modo juntas já são conhecidas; na primeira
    versão resolve-se a IK numérica do flange ponto a ponto, partindo sempre da
    solução anterior para as juntas variarem de forma contínua.
    """
    if traj.get("qs") is not None:
        return traj["qs"].copy(), bool(traj.get("ik_ok", True))
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
    As três condições do enunciado: (i) velocidades (e aceleração das juntas),
    (ii) alcance e (iii) limites das juntas, mais a distância à mesa e à base
    (ponta, flange e origens dos referenciais de J2 em diante). As derivadas
    calculam-se por diferenças centrais.
    """
    t, pos, flange = traj["t"], traj["pos"], traj["pos_flange"]
    sf = C.SAFETY_FACTOR
    v_tcp = np.linalg.norm(np.gradient(pos, t, axis=0), axis=1)
    # Sem desvio de ferramenta configurado, o controlador comanda o flange, que
    # pode andar mais depressa do que a ponta; verificam-se os dois.
    v_flange = np.linalg.norm(np.gradient(flange, t, axis=0), axis=1)
    qd = np.gradient(qs, t, axis=0)
    qdd = np.gradient(qd, t, axis=0)
    ombro = fk(np.zeros(6), all_frames=True)[2][:3, 3]     # origem de {2}
    pulso = np.array([fk(q, all_frames=True)[5][:3, 3] for q in qs])   # centro do pulso (origem de {5})
    alcance = np.linalg.norm(pulso - ombro, axis=1)
    lo = np.array([l for l, _ in C.JOINT_LIMITS])
    hi = np.array([h for _, h in C.JOINT_LIMITS])
    z_juntas = min(min(T[2, 3] for T in fk(q, all_frames=True)[2:]) for q in qs)
    z_min = min(pos[:, 2].min(), flange[:, 2].min(), z_juntas)
    r_xy = min(np.linalg.norm(pos[:, :2], axis=1).min(), np.linalg.norm(flange[:, :2], axis=1).min())

    checks = {
        "vel. TCP": (v_tcp.max(), sf * C.TCP_SPEED_MAX, "mm/s"),
        "vel. flange": (v_flange.max(), sf * C.TCP_SPEED_MAX, "mm/s"),
        "vel. juntas": (np.abs(qd).max(), sf * C.JOINT_SPEED_MAX, "deg/s"),
        "acel. juntas": (np.abs(qdd).max(), sf * C.JOINT_ACC_MAX, "deg/s2"),
        "alcance": (alcance.max(), C.REACH_FACTOR * C.REACH, "mm"),
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
    return all(res.values()), dict(v_tcp=v_tcp, v_flange=v_flange, qd=qd, qdd=qdd, alcance=alcance)


def plot(traj, qs, extra, sufixo=""):
    """Gráficos da trajetória em report/traj{sufixo}_*.png."""
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
    ax[2].plot(t, extra["v_tcp"], label="ponta da pinça")
    ax[2].plot(t, extra["v_flange"], "--", label="flange")
    ax[2].axhline(C.TCP_SPEED_MAX, color="r", ls="--", label="limite fabricante")
    ax[2].set_ylabel("|v| [mm/s]")
    ax[2].set_xlabel("t [s]")
    ax[2].legend()
    fig.tight_layout()
    fig.savefig(os.path.join(REPORT, f"traj{sufixo}_cartesiana.png"), dpi=150)

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
    fig.savefig(os.path.join(REPORT, f"traj{sufixo}_juntas.png"), dpi=150)

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
    fig.savefig(os.path.join(REPORT, f"traj{sufixo}_3d.png"), dpi=150)
    print(f"  gráficos em {os.path.normpath(REPORT)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--t-down", type=float, default=DEFAULTS["t_down"])
    ap.add_argument("--raio", type=float, default=DEFAULTS["raio"])
    ap.add_argument("--garra", type=float, default=DEFAULTS["garra"], help="mm, flange -> ponta (0 = sem pinça)")
    ap.add_argument("--phi-end", type=float, default=DEFAULTS["phi_end"])
    ap.add_argument("--k-phi", type=float, default=DEFAULTS["k_phi"], help="0..1, quanto do arco roda a pinça")
    ap.add_argument("--golfista", action="store_true", help="swing desenhado nas juntas (preset GOLFISTA)")
    ap.add_argument("--perto", action="store_true", help="com --golfista: preset GOLFISTA_PERTO, o do relatório")
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()

    p = dict(t_down=args.t_down, raio=args.raio, garra=args.garra, phi_end=args.phi_end, k_phi=args.k_phi)
    preset = GOLFISTA_PERTO if args.perto else GOLFISTA
    traj = swing_trajectory({**p, **preset} if args.golfista else p)
    qs, conv = joint_trajectory(traj)
    print(f"  {len(traj['t'])} pontos, {traj['t'][-1]:.2f} s, juntas {'obtidas' if conv else 'NÃO obtidas em todos os pontos'}")
    ok, extra = check_feasibility(traj, qs)
    print(f"  => trajetória {'VIÁVEL' if ok and conv else 'NÃO VIÁVEL'}")
    if args.plot:
        plot(traj, qs, extra, ("_golfista_perto" if args.perto else "_golfista") if args.golfista else "")


if __name__ == "__main__":
    main()
