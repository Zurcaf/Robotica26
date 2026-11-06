"""
Swing V2 — o robô é o BRAÇO do golfista, a pinça são as MÃOS e uma caneta
presa na pinça é o TACO.

Uso (a partir de Lab2/):
    python src/swing_v2.py                  # gera, verifica e imprime o resumo
    python src/swing_v2.py --plot           # + gráficos em report/v2_*.png
    python src/swing_v2.py --taco 120 --t-down 1.6

PÊNDULO DUPLO COM JUNTAS REAIS (como o V13 do Lab 1)
----------------------------------------------------
Com J1 = J4 = J6 = 0 os eixos de J2, J3 e J5 são paralelos (horizontais, ao
longo de y): o Lite 6 fica um braço PLANO no plano vertical x-z.

    golfista                     Lite 6
    tronco inclinado             J2  (fixa durante o swing)
    ombro                        J3  (ativa)
    braço                        antebraço J3 -> J5 (243.6 mm)
    pulso: arma e solta          J5  (ativa)
    mãos                         pinça (TCP a 83.6 mm do flange)
    taco                         caneta, `taco` mm para lá das pontas dos dedos

Ângulos no plano, medidos a partir da vertical para BAIXO, + para a frente
(para longe da base) — derivados da FK:
    braço  A = 20.9 + q3 - q2        (20.9 deg = desvio de 87 mm do antebraço)
    taco   C = q3 - q2 - q5           =>  pulso direito (taco no prolongamento
                                         do braço) quando q5 = -20.9

O "corpo" do robô está por trás/por baixo do ombro, por isso o backswing é
para a FRENTE e para cima (A cresce) e o follow-through, de volta para o corpo,
é curto: J3 >= -3.5 deg (antebraço encostado ao braço). O pulso continua a
rodar no follow-through, como as mãos de um golfista.

Perfis (os do Lab 1), u = tempo desde o topo, T = t_down:
  J3  backswing quíntico; downswing com aceleração constante
      q3(u) = q3_imp + (q3_top - q3_imp)(1 - (u/T)^2)  -> velocidade máx. na bola
  J5  arma no backswing (quíntico); no downswing fica armada até f_release·T
      e solta com aceleração constante, chegando a q5_imp NO MESMO instante
      que J3 (é isto que põe a cabeça do taco na bola) e à velocidade máxima
  follow-through: as duas desaceleram a ritmo constante e param juntas.

A trajetória é calculada no espaço das juntas e enviada ao robô junta a junta
(set_servo_angle_j): não há IK, e não se apanha o limite cartesiano (~220 mm/s)
medido no laboratório. Posição/orientação da pinça e da caneta saem da FK.
"""
import argparse
import os

import numpy as np

import config as C
from kinematics import fk, rpy_from_R, tool_transform

HERE = os.path.dirname(os.path.abspath(__file__))
REPORT = os.path.join(HERE, "..", "report")

# Encontrados por procura aleatória + ajuste do tempo (ver o notebook swing_v2):
# tronco inclinado 43 deg sobre a bola, braço a balançar 47 deg, pulso a armar 45 deg.
# O que limita a velocidade é a PINÇA (500 mm/s do fabricante, 400 c/ margem):
# t_down é o mais curto que a respeita.
DEFAULTS = dict(
    taco=120.0,          # mm, caneta para lá das pontas dos dedos (medir no lab)
    q2=43.0,             # graus, J2 = inclinação do tronco (fixa)
    q3_imp=100.0,        # graus, J3 no impacto  (braço A = 20.9 + q3 - q2)
    q5_imp=58.0,         # graus, J5 no impacto
    back=47.0,           # graus que o braço sobe no backswing (J3)
    cock=45.0,           # graus que o pulso arma no topo (J5)
    q3_end=82.0,         # graus, J3 no fim do follow-through
    f_release=0.4,       # fração do downswing em que o pulso solta
    t_back=2.6,          # s, backswing
    t_down=2.1,          # s, downswing
    dt=0.01,             # s, 100 Hz
)

# carregar a caneta: braço para a frente, pinça na horizontal à altura do peito
Q_LOAD = [0.0, 0.0, 60.0, 0.0, -30.0, 0.0]

# raios (mm) para as colisões, no plano x-z — estimativas conservadoras
R_BASE, R_BRACO, R_ANTEBRACO, R_PINCA, R_CANETA = 65.0, 45.0, 40.0, 30.0, 6.0
FOLGA_COLISAO = 15.0     # mm entre superfícies
FOLGA_CANETA_MESA = 10.0  # mm, a ponta da caneta pode descer mais do que o robô


def quintic(s):
    s = np.clip(s, 0.0, 1.0)
    return 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5


def joint_profiles(p):
    """t, q (N,6) em graus. Só J3 e J5 variam."""
    Tb, Td, dt = p["t_back"], p["t_down"], p["dt"]
    q3i, q5i = p["q3_imp"], p["q5_imp"]
    q3t, q5t = q3i + p["back"], q5i - p["cock"]   # armar = o taco fica para trás do braço

    w3 = -2.0 * (q3t - q3i) / Td                   # deg/s no impacto
    t_rel = p["f_release"] * Td
    w5 = 2.0 * (q5i - q5t) / (Td - t_rel)          # solta com aceleração constante
    Tf = 2.0 * (p["q3_end"] - q3i) / w3            # J3 pára em q3_end
    t = np.arange(0.0, Tb + Td + Tf + dt / 2, dt)
    q3 = np.empty_like(t)
    q5 = np.empty_like(t)

    b = t <= Tb
    q3[b] = q3i + p["back"] * quintic(t[b] / Tb)
    q5[b] = q5i - p["cock"] * quintic(t[b] / Tb)

    d = (t > Tb) & (t <= Tb + Td)
    u = t[d] - Tb
    q3[d] = q3i + (q3t - q3i) * (1.0 - (u / Td) ** 2)
    s = np.clip((u - t_rel) / (Td - t_rel), 0.0, 1.0)
    q5[d] = q5t + (q5i - q5t) * s ** 2

    f = t > Tb + Td
    u = t[f] - Tb - Td
    q3[f] = q3i + w3 * u - 0.5 * (w3 / Tf) * u ** 2
    q5[f] = q5i + w5 * u - 0.5 * (w5 / Tf) * u ** 2

    q = np.zeros((len(t), 6))
    q[:, 1] = p["q2"]
    q[:, 2] = q3
    q[:, 4] = q5
    return t, q, dict(q3_top=q3t, q5_top=q5t, w3=w3, w5=w5, T_follow=Tf)


def swing_v2(p=None):
    """
    Trajetória completa. Devolve dict com t, qs (N,6), pos = ponta da caneta
    (cabeça do taco), pos_tcp = pontas dos dedos, pos_flange, R, rpy, e os
    pontos do braço (J2, J3, J5) para desenhar e para as colisões.
    """
    p = {**DEFAULTS, **(p or {})}
    t, qs, info = joint_profiles(p)
    tr = from_joints(t, qs, p["taco"])
    k_imp = int(np.argmin(abs(t - p["t_back"] - p["t_down"])))
    tr.update(k_imp=k_imp, params=p, info=info, bola=tr["pos"][k_imp])
    return tr   # a bola é onde a cabeça do taco passa no impacto


def transition(q_a, q_b, taco, speed=20.0, dt=0.05):
    """Movimento junta a junta (linear) de q_a para q_b, como o set_servo_angle faz."""
    q_a, q_b = np.asarray(q_a, float), np.asarray(q_b, float)
    T = max(np.abs(q_b - q_a).max() / speed, dt)
    t = np.arange(0.0, T + dt / 2, dt)
    qs = q_a + np.outer(t / T, q_b - q_a)
    return from_joints(t, qs, taco)


def check_transition(tr, nome, verbose=True):
    """Mesa e colisões numa deslocação (sem verificar velocidades: é lenta)."""
    z_robo = np.min([tr["J3"][:, 2], tr["J5"][:, 2], tr["pos_flange"][:, 2], tr["pos_tcp"][:, 2]])
    z_caneta = tr["pos"][:, 2].min()
    col = min(v.min() for v in collisions(tr).values())
    ok = (z_robo >= C.TABLE_Z + C.TABLE_CLEARANCE and z_caneta >= C.TABLE_Z + FOLGA_CANETA_MESA
          and col >= FOLGA_COLISAO)
    if verbose:
        print(f"  {nome:<26}: robô z mín {z_robo:6.1f}, caneta z mín {z_caneta:6.1f}, folga colisão {col:6.1f} mm  {'OK' if ok else 'FALHA'}")
    return ok


def from_joints(t, qs, taco):
    """FK de toda a cadeia para cada q: pontos do braço, pinça e caneta."""
    L_tcp, L_head = C.GRIPPER_TCP_Z, C.GRIPPER_TCP_Z + taco
    J2, J3, J5, FL, TCP, HEAD, R = ([] for _ in range(7))
    for q in qs:
        Ts = fk(q, all_frames=True)
        J2.append(Ts[2][:3, 3])
        J3.append(Ts[3][:3, 3])
        J5.append(Ts[5][:3, 3])
        FL.append(Ts[6][:3, 3])
        TCP.append((Ts[6] @ tool_transform(L_tcp))[:3, 3])
        HEAD.append((Ts[6] @ tool_transform(L_head))[:3, 3])
        R.append(Ts[6][:3, :3])
    R = np.array(R)
    return dict(t=t, qs=np.asarray(qs), J2=np.array(J2), J3=np.array(J3), J5=np.array(J5),
                pos_flange=np.array(FL), pos_tcp=np.array(TCP), pos=np.array(HEAD),
                R=R, rpy=np.unwrap([rpy_from_R(r) for r in R], period=360.0, axis=0))


def seg_dist(p1, p2, q1, q2):
    """Distância mínima entre os segmentos p1-p2 e q1-q2 (3D)."""
    d1, d2, r = p2 - p1, q2 - q1, p1 - q1
    a, e, f = d1 @ d1, d2 @ d2, d2 @ r
    if a < 1e-9 and e < 1e-9:   # dois pontos
        return np.linalg.norm(r)
    if a < 1e-9:                # p é um ponto (p.ex. caneta de comprimento 0)
        return np.linalg.norm(p1 - (q1 + d2 * np.clip(f / e, 0, 1)))
    c, b = d1 @ r, d1 @ d2
    if e < 1e-9:                # q é um ponto
        return np.linalg.norm(q1 - (p1 + d1 * np.clip(-c / a, 0, 1)))
    den = a * e - b * b
    s = np.clip((b * f - c * e) / den, 0, 1) if den > 1e-12 else 0.0
    tt = (b * s + f) / e
    if tt < 0:
        tt, s = 0.0, np.clip(-c / a, 0, 1)
    elif tt > 1:
        tt, s = 1.0, np.clip((b - c) / a, 0, 1)
    return np.linalg.norm((p1 + d1 * s) - (q1 + d2 * tt))


def collisions(tr):
    """Folga (mm, entre superfícies) ao longo do tempo para cada par perigoso."""
    base0, base1 = np.zeros(3), np.array([0.0, 0.0, 243.3])
    pares = {
        "caneta-base": [], "caneta-braço": [], "pinça-base": [], "pinça-braço": [],
        "antebraço-base": [],
    }
    for k in range(len(tr["t"])):
        J2, J3, J5 = tr["J2"][k], tr["J3"][k], tr["J5"][k]
        fl, tcp, head = tr["pos_flange"][k], tr["pos_tcp"][k], tr["pos"][k]
        pares["caneta-base"].append(seg_dist(tcp, head, base0, base1) - R_CANETA - R_BASE)
        pares["caneta-braço"].append(seg_dist(tcp, head, J2, J3) - R_CANETA - R_BRACO)
        pares["pinça-base"].append(seg_dist(fl, tcp, base0, base1) - R_PINCA - R_BASE)
        pares["pinça-braço"].append(seg_dist(fl, tcp, J2, J3) - R_PINCA - R_BRACO)
        pares["antebraço-base"].append(seg_dist(J3, J5, base0, base1) - R_ANTEBRACO - R_BASE)
    return {k: np.array(v) for k, v in pares.items()}


def check_v2(tr, verbose=True):
    """(i) velocidades, (ii) alcance, (iii) limites das juntas, mesa e colisões."""
    t, qs = tr["t"], tr["qs"]
    sf, m = C.SAFETY_FACTOR, C.JOINT_LIMIT_MARGIN
    qd = np.gradient(qs, t, axis=0)
    # o limite de 500 mm/s é do efetor final (a pinça); a ponta da caneta pode ir
    # mais depressa — é essa a vantagem do taco
    v_fl = np.linalg.norm(np.gradient(tr["pos_tcp"], t, axis=0), axis=1)
    v_head = np.linalg.norm(np.gradient(tr["pos"], t, axis=0), axis=1)
    lo = np.array([l for l, _ in C.JOINT_LIMITS])
    hi = np.array([h for _, h in C.JOINT_LIMITS])
    folga_j = np.minimum(qs - lo, hi - qs).min(axis=0)
    alcance = np.linalg.norm(tr["pos_flange"] - tr["J2"], axis=1)
    z_robo = np.min([tr["J3"][:, 2], tr["J5"][:, 2], tr["pos_flange"][:, 2], tr["pos_tcp"][:, 2]])
    z_caneta = tr["pos"][:, 2].min()
    col = collisions(tr)
    col_min = {k: v.min() for k, v in col.items()}

    # quanto a cabeça do taco ainda desce depois do impacto (golpe descendente,
    # como com um ferro); a segurança é dada por "caneta acima da mesa"
    desce = tr["pos"][tr["k_imp"], 2] - z_caneta
    res = {
        "vel. juntas": np.abs(qd).max() <= sf * C.JOINT_SPEED_MAX,
        "vel. flange": v_fl.max() <= sf * C.TCP_SPEED_MAX,
        "limites juntas": folga_j.min() >= m,
        "robô acima da mesa": z_robo >= C.TABLE_Z + C.TABLE_CLEARANCE,
        "caneta acima da mesa": z_caneta >= C.TABLE_Z + FOLGA_CANETA_MESA,
        "colisões": min(col_min.values()) >= FOLGA_COLISAO,
    }
    if verbose:
        j = int(np.abs(qd).max(axis=0).argmax())
        jl = int(folga_j.argmin())
        print(f"  vel. juntas        : máx {np.abs(qd).max():6.1f} deg/s (J{j + 1}; limite c/ margem {sf * C.JOINT_SPEED_MAX:.0f})  {'OK' if res['vel. juntas'] else 'FALHA'}")
        print(f"  vel. pinça (TCP)   : máx {v_fl.max():6.1f} mm/s (limite c/ margem {sf * C.TCP_SPEED_MAX:.0f})  {'OK' if res['vel. flange'] else 'FALHA'}")
        print(f"  golpe (info)       : a cabeça ainda desce {desce:5.1f} mm depois do impacto (descendente, como um ferro)")
        print(f"  alcance (info)     : flange a {alcance.max():6.1f} mm de J2 — trajetória em juntas, sempre alcançável")
        print(f"  limites juntas     : menor folga {folga_j.min():5.1f} deg (J{jl + 1}, mínimo {m:.0f})  {'OK' if res['limites juntas'] else 'FALHA'}")
        print(f"  robô acima da mesa : z mín {z_robo:6.1f} mm (mínimo {C.TABLE_CLEARANCE:.0f})  {'OK' if res['robô acima da mesa'] else 'FALHA'}")
        print(f"  caneta acima mesa  : z mín {z_caneta:6.1f} mm (mínimo {FOLGA_CANETA_MESA:.0f})  {'OK' if res['caneta acima da mesa'] else 'FALHA'}")
        pior = min(col_min, key=col_min.get)
        print(f"  colisões           : menor folga {col_min[pior]:6.1f} mm ({pior}, mínimo {FOLGA_COLISAO:.0f})  {'OK' if res['colisões'] else 'FALHA'}")
        print(f"  cabeça do taco     : {v_head[tr['k_imp']]:6.0f} mm/s no impacto (máx {v_head.max():.0f}); bola em {np.round(tr['bola'], 1)}")
    return all(res.values()), dict(qd=qd, v_flange=v_fl, v_head=v_head, alcance=alcance,
                                   folga_juntas=folga_j, col=col, res=res)


def plot(tr, ex):
    import matplotlib.pyplot as plt

    os.makedirs(REPORT, exist_ok=True)
    t, k = tr["t"], tr["k_imp"]

    fig, ax = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    for j in (2, 4):
        ax[0].plot(t, tr["qs"][:, j], label=f"J{j + 1}")
        ax[1].plot(t, ex["qd"][:, j], label=f"J{j + 1}")
    lim = C.SAFETY_FACTOR * C.JOINT_SPEED_MAX
    ax[1].axhline(lim, color="r", ls="--", label="limite c/ margem")
    ax[1].axhline(-lim, color="r", ls="--")
    ax[2].plot(t, ex["v_head"], label="cabeça do taco (caneta)")
    ax[2].plot(t, ex["v_flange"], label="flange")
    ax[0].set_ylabel("q [deg]")
    ax[1].set_ylabel("dq/dt [deg/s]")
    ax[2].set_ylabel("velocidade [mm/s]")
    ax[2].set_xlabel("t [s]")
    for a in ax:
        a.axvline(t[k], color="gray", ls=":")
        a.legend(fontsize=8)
        a.grid(alpha=.3)
    fig.tight_layout()
    fig.savefig(os.path.join(REPORT, "v2_juntas.png"), dpi=150)

    fig, ax = plt.subplots(figsize=(8, 7))
    draw_side(ax, tr, frames=np.linspace(0, len(t) - 1, 12).astype(int))
    fig.savefig(os.path.join(REPORT, "v2_estroboscopio.png"), dpi=150)

    fig, ax = plt.subplots(figsize=(8, 3.5))
    for nome, v in ex["col"].items():
        ax.plot(t, v, label=nome)
    ax.axhline(FOLGA_COLISAO, color="r", ls="--", label="folga mínima")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("folga [mm]")
    ax.legend(fontsize=8, ncol=3)
    ax.grid(alpha=.3)
    fig.tight_layout()
    fig.savefig(os.path.join(REPORT, "v2_colisoes.png"), dpi=150)
    print(f"  gráficos em {os.path.normpath(REPORT)}")


def draw_side(ax, tr, frames, alpha_min=0.15):
    """Vista de lado (plano x-z): robô, pinça e caneta em vários instantes."""
    ax.add_patch(__import__("matplotlib").patches.Rectangle((-65, 0), 130, 243.3, color="0.85"))
    ax.axhline(C.TABLE_Z, color="saddlebrown", lw=3)
    ax.plot(*tr["pos"][:, [0, 2]].T, color="tab:blue", lw=1, label="cabeça do taco")
    for i, k in enumerate(frames):
        a = alpha_min + (1 - alpha_min) * i / max(len(frames) - 1, 1)
        chain = np.array([tr["J2"][k], tr["J3"][k], tr["J5"][k], tr["pos_flange"][k]])
        ax.plot(chain[:, 0], chain[:, 2], "-o", color="k", ms=3, lw=2, alpha=a)
        ax.plot(*np.array([tr["pos_flange"][k], tr["pos_tcp"][k]])[:, [0, 2]].T, color="tab:orange", lw=5, alpha=a)
        ax.plot(*np.array([tr["pos_tcp"][k], tr["pos"][k]])[:, [0, 2]].T, color="tab:red", lw=2, alpha=a)
    ax.scatter(tr["bola"][0], tr["bola"][2], s=120, color="w", edgecolor="k", zorder=5, label="bola")
    ax.set_aspect("equal")
    ax.set_xlabel("x [mm]")
    ax.set_ylabel("z [mm]")
    ax.grid(alpha=.3)
    ax.legend(loc="upper left", fontsize=8)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--taco", type=float, default=DEFAULTS["taco"], help="mm de caneta para lá dos dedos")
    ap.add_argument("--t-down", type=float, default=DEFAULTS["t_down"])
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()
    tr = swing_v2(dict(taco=args.taco, t_down=args.t_down))
    print(f"  {len(tr['t'])} pontos, {tr['t'][-1]:.2f} s (impacto em t = {tr['t'][tr['k_imp']]:.2f} s)")
    ok, ex = check_v2(tr)
    print("  deslocações com a caneta presa (junta a junta, 20 deg/s):")
    ok &= check_transition(transition(Q_LOAD, tr["qs"][0], args.taco), "carregar -> início")
    ok &= check_transition(transition(tr["qs"][-1], Q_LOAD, args.taco), "fim -> carregar")
    print(f"  => swing V2 {'VIÁVEL' if ok else 'NÃO VIÁVEL'}")
    if args.plot:
        plot(tr, ex)


if __name__ == "__main__":
    main()
