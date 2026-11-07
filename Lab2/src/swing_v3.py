"""
Swing V3 — o swing demonstrado à mão pelo colega, refinado com os perfis do Lab 1.

Uso (a partir de Lab2/):
    python src/swing_v3.py                 # gera, verifica e compara com a demonstração
    python src/swing_v3.py --plot          # + gráficos em report/v3_*.png
    python src/swing_v3.py --t-down 3.0

DE ONDE VEM
-----------
data/demo_colega_2026-11-06.csv: o robô movido à mão (Manual Mode), lido do
vídeo do UFactory Studio. O que a demonstração mostra:
  - J1 (base) é o "tronco" a rodar: 124 deg — o motor principal do swing;
  - J2 (ombro) baixa o braço no downswing; J3 só se mexe no follow-through;
  - J5 é o "pulso": arma no topo e solta até ao impacto (o lag do Lab 1);
  - J4 fica parado (~-228 deg) e J6 roda devagar: a pinça roda COM o arco,
    como um taco, sem girar sobre si própria;
  - a ponta faz um arco à volta do robô e passa rente à mesa à frente dele.

O QUE SE REFINA
---------------
Da demonstração tiram-se só três POSES-CHAVE (endereço/impacto, topo e fim) e
o caminho entre elas é refeito com as leis do Lab 1, em todas as juntas ao mesmo
tempo:
  1. backswing   endereço -> topo, quíntico (parte e chega parado)
  2. downswing   topo -> impacto com ACELERAÇÃO CONSTANTE e o MESMO T em
                 todas as juntas: chegam juntas ao impacto, à velocidade máxima;
                 o pulso (J5) fica armado até f_release·T e solta depois
                 (também a acelerar), chegando ao impacto no mesmo instante
  3. follow-through  impacto -> fim com um quíntico que parte da velocidade e
                 aceleração do impacto e chega parado ao fim
É enviado junta a junta (set_servo_angle_j): sem IK, sem limite cartesiano.
"""
import argparse
import os

import numpy as np

import config as C
from kinematics import fk, rpy_from_R, tool_transform

HERE = os.path.dirname(os.path.abspath(__file__))
REPORT = os.path.join(HERE, "..", "report")
DEMO = os.path.join(HERE, "..", "data", "demo_colega_2026-11-06.csv")

# poses-chave da demonstração [deg]
Q_TOPO = [74.1, 14.5, 65.3, -227.9, 95.0, 176.5]          # t = 0.47 s
Q_IMPACTO = [-10.7, 63.7, 64.4, -231.0, 55.3, 143.8]      # t = 2.80 s (ponto mais baixo)
Q_FIM = [-50.3, 54.8, 112.3, -225.5, 41.9, 126.9]         # t = 3.97 s

DEFAULTS = dict(
    q_topo=Q_TOPO,
    q_impacto=Q_IMPACTO,
    q_fim=Q_FIM,
    dj2_impacto=-2.0,    # graus a somar a J2 no impacto: sobe a ponta (a demo passava a 2 cm da mesa)
    # juntas que FAZEM o swing (chegam ao impacto à velocidade máxima, como no Lab 1):
    # base J1, pulso J5 e rotação da pinça J6. As outras (J2, J3, J4) só ajustam a
    # postura e chegam PARADAS ao impacto — na demo J2 e J3 estão parados no fundo.
    juntas_swing=(0, 4, 5),
    f_release=0.5,       # fração do downswing em que o pulso (J5) solta
    t_back=3.0,          # s, endereço -> topo
    t_down=3.0,          # s, topo -> impacto (o mais curto que mantém a pinça <= 400 mm/s)
    t_follow=2.0,        # s, impacto -> fim
    dt=0.01,             # s
)

FOLGA_PONTA_MESA = 20.0   # mm, a ponta pode descer mais do que o resto do robô (passa na bola);
                          # a nossa FK difere ~2 mm do robô real, daí a margem

# "Sítio da bola" marcado no robô (6/11): o braço levado à mão até à bola, lido no
# UFactory Studio. A bola é onde fica a ponta da pinça nesta pose.
Q_BOLA_LAB = [-87.7, 63.8, 89.0, -153.9, -5.0, 65.6]


def ponta(q):
    """Posição da ponta da pinça para as juntas q [deg]."""
    return (fk(q) @ tool_transform(C.GRIPPER_TCP_Z))[:3, 3]


def ik5(p_flange, z_pinca, q0, iters=300):
    """
    IK com 5 restrições: posição do flange + DIREÇÃO do eixo da pinça (a rotação
    da pinça sobre o próprio eixo fica livre). Mínimos quadrados amortecidos.
    """
    lo = np.array([l for l, _ in C.JOINT_LIMITS]); hi = np.array([h for _, h in C.JOINT_LIMITS])
    q = np.array(q0, float)

    def err(qq):
        T_ = fk(qq)
        return np.concatenate([p_flange - T_[:3, 3], 100.0 * np.cross(T_[:3, 2], z_pinca)])

    for _ in range(iters):
        e = err(q)
        if np.linalg.norm(e[:3]) < 1e-3 and np.linalg.norm(e[3:]) < 1e-3:
            return q, True
        J = np.zeros((6, 6))
        for j in range(6):
            dq = np.zeros(6); dq[j] = 1e-4
            J[:, j] = -(err(q + dq) - e) / 1e-4
        q = np.clip(q + J.T @ np.linalg.solve(J @ J.T + np.eye(6), e), lo, hi)
    return q, False


def bola_em(bola, p=None):
    """
    Reorienta a V3 para a ponta da pinça passar em `bola` no impacto:
      1. roda o swing inteiro na base (J1) até ficar virado para a bola;
      2. recalcula a pose de impacto (IK de 5 restrições) com a ponta na bola e a
         pinça inclinada como na demonstração (rodada do mesmo ângulo);
      3. topo e fim rodam com a base, o resto da postura fica igual;
      4. o pulso (J5) acaba de soltar NO impacto (chega parado): a ponta passa na
         bola no ponto mais baixo do arco. Com o pulso ainda a soltar, a ponta ia a
         descer a 100 mm/s e, com a bola do lab a 27 mm da mesa, chegava a 6 mm dela.
    Daí em diante a bola é fixa: ajustar() só mexe nos tempos.
    """
    p = {**DEFAULTS, **(p or {})}
    tr0 = swing_v3(dict(p, dj2_impacto=0.0))
    qa0, b0 = tr0["q_impacto"], tr0["bola"]
    bola = np.asarray(bola, float)
    dang = np.arctan2(bola[1], bola[0]) - np.arctan2(b0[1], b0[0])
    c, s = np.cos(dang), np.sin(dang)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    z_alvo = Rz @ fk(qa0)[:3, 2]                         # eixo da pinça rodado com a base
    seed = np.array(qa0, float); seed[0] += np.degrees(dang)
    qa, ok = ik5(bola - C.GRIPPER_TCP_Z * z_alvo, z_alvo, seed)
    if not ok:
        raise ValueError(f"a ponta não consegue chegar a {np.round(bola, 1)} com a pinça inclinada como na V3")
    rodar = lambda q: [q[0] + np.degrees(dang)] + list(q[1:])
    return dict(p, q_topo=rodar(p["q_topo"]), q_fim=rodar(p["q_fim"]), q_impacto=list(qa),
                dj2_impacto=0.0, bola_fixa=True, juntas_swing=(0, 5))


def parametros(bola_demo=False):
    """Parâmetros finais: bola do lab (Q_BOLA_LAB) por omissão, ou a da demonstração."""
    return ajustar() if bola_demo else ajustar(bola_em(ponta(Q_BOLA_LAB)))


def quintic_bc(q0, v0, a0, q1, v1, a1, T, t):
    """Polinómio de grau 5 com posição, velocidade e aceleração dadas nos dois extremos."""
    t = np.asarray(t, float)[:, None]
    A = np.array([[T ** 3, T ** 4, T ** 5],
                  [3 * T ** 2, 4 * T ** 3, 5 * T ** 4],
                  [6 * T, 12 * T ** 2, 20 * T ** 3]])
    b = np.stack([q1 - (q0 + v0 * T + a0 * T ** 2 / 2), v1 - (v0 + a0 * T), a1 - a0])
    c3, c4, c5 = np.linalg.solve(A, b)
    return q0 + v0 * t + a0 * t ** 2 / 2 + c3 * t ** 3 + c4 * t ** 4 + c5 * t ** 5


def joint_profiles(p):
    qa = np.array(p["q_impacto"], float)
    qa[1] += p["dj2_impacto"]
    qt, qf = np.array(p["q_topo"], float), np.array(p["q_fim"], float)
    Tb, Td, Tf, dt = p["t_back"], p["t_down"], p["t_follow"], p["dt"]
    z6 = np.zeros(6)

    t = np.arange(0.0, Tb + Td + Tf + dt / 2, dt)
    q = np.empty((len(t), 6))

    b = t <= Tb                                                 # 1. backswing
    q[b] = quintic_bc(qa, z6, z6, qt, z6, z6, Tb, t[b])

    sw = np.zeros(6, bool)
    sw[list(p["juntas_swing"])] = True
    d = (t > Tb) & (t <= Tb + Td)                               # 2. downswing
    u = (t[d] - Tb)[:, None]
    q[d] = quintic_bc(qt, z6, z6, qa, z6, z6, Td, u[:, 0])      # postura: chega parada
    acel = qt + (qa - qt) * (u / Td) ** 2                       # swing: aceleração constante
    q[d][:, sw] = acel[:, sw]
    q[np.ix_(d, sw)] = acel[:, sw]
    v_imp = np.where(sw, 2 * (qa - qt) / Td, 0.0)               # velocidade no impacto
    a_imp = np.where(sw, 2 * (qa - qt) / Td ** 2, 0.0)
    t_rel = p["f_release"] * Td
    if sw[4]:                                                   # pulso: armado até t_rel, solta a acelerar
        s = np.clip((u[:, 0] - t_rel) / (Td - t_rel), 0, 1)
        q[d, 4] = qt[4] + (qa[4] - qt[4]) * s ** 2
        v_imp[4] = 2 * (qa[4] - qt[4]) / (Td - t_rel)
        a_imp[4] = 2 * (qa[4] - qt[4]) / (Td - t_rel) ** 2
    else:                                                       # pulso: armado até t_rel, acaba de soltar NO impacto
        s = np.clip((u[:, 0] - t_rel) / (Td - t_rel), 0, 1)
        q[d, 4] = qt[4] + (qa[4] - qt[4]) * (10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5)
    f = t > Tb + Td                                             # 3. follow-through
    q[f] = quintic_bc(qa, v_imp, a_imp, qf, z6, z6, Tf, t[f] - Tb - Td)
    return t, q, qa


def from_joints(t, qs):
    """FK: flange, pontas dos dedos (TCP da pinça) e juntas, para cada q."""
    FL, TCP, J, R = [], [], [], []
    for q in qs:
        Ts = fk(q, all_frames=True)
        FL.append(Ts[6][:3, 3])
        TCP.append((Ts[6] @ tool_transform(C.GRIPPER_TCP_Z))[:3, 3])
        J.append([T_[:3, 3] for T_ in Ts[2:]])
        R.append(Ts[6][:3, :3])
    return dict(t=np.asarray(t), qs=np.asarray(qs), pos_flange=np.array(FL), pos=np.array(TCP),
                juntas=np.array(J), R=np.array(R),
                rpy=np.unwrap([rpy_from_R(r) for r in R], period=360.0, axis=0))


def swing_v3(p=None):
    p = {**DEFAULTS, **(p or {})}
    t, qs, qa = joint_profiles(p)
    tr = from_joints(t, qs)
    k = int(np.argmin(abs(t - p["t_back"] - p["t_down"])))
    tr.update(params=p, k_imp=k, bola=tr["pos"][k], q_impacto=qa)
    return tr


def fase_maximos(tr):
    """Velocidade máxima da pinça [mm/s] e das juntas [deg/s] em cada fase."""
    t, p = tr["t"], tr["params"]
    v = np.linalg.norm(np.gradient(tr["pos"], t, axis=0), axis=1)
    qd = np.abs(np.gradient(tr["qs"], t, axis=0)).max(axis=1)
    Tb, Td = p["t_back"], p["t_down"]
    fases = {"t_back": t <= Tb, "t_down": (t > Tb) & (t <= Tb + Td), "t_follow": t > Tb + Td}
    return {k: (v[m].max(), qd[m].max()) for k, m in fases.items()}


# fração do limite da pinça (já com a margem de 0.8) que cada fase usa no máximo
ALVO_FASE = dict(t_back=0.75, t_down=0.97, t_follow=0.9)


def ajustar(p=None, verbose=False):
    """
    Ajusta o movimento aos limites do robô:
      - cada fase dura o mínimo para a pinça não passar de ALVO_FASE × 400 mm/s e
        as juntas de 144 deg/s (a velocidade escala com 1/T): o downswing chega à
        bola no máximo permitido; o backswing é mais lento, como num golfista;
      - sobe o impacto (J2) até a ponta nunca descer abaixo da folga à mesa.
    """
    p = {**DEFAULTS, **(p or {})}
    v_lim = C.SAFETY_FACTOR * C.TCP_SPEED_MAX
    q_lim = C.SAFETY_FACTOR * C.JOINT_SPEED_MAX
    for _ in range(12):
        for _ in range(8):
            tr = swing_v3(dict(p, dt=0.02))
            mx = fase_maximos(tr)
            v_imp = np.linalg.norm(np.gradient(tr["pos"], tr["t"], axis=0), axis=1)[tr["k_imp"]]
            mudou = False
            for k, (v, qd) in mx.items():
                alvo = ALVO_FASE[k] * v_lim
                if k == "t_follow":             # começa à velocidade do impacto: não pode ser menos
                    alvo = max(alvo, 1.02 * v_imp)
                r = max(v / alvo, qd / q_lim)
                # o downswing ajusta-se nos dois sentidos (o mais rápido possível);
                # as outras fases só se esticam se passarem do limite
                if k == "t_down" or r > 1.0:
                    novo = float(np.clip(p[k] * r * 1.01, 0.3, 30.0))
                    mudou |= abs(novo - p[k]) > 0.01
                    p[k] = novo
            if not mudou:
                break
        tr = swing_v3(p)
        z_ponta = tr["pos"][:, 2].min()
        if z_ponta >= C.TABLE_Z + FOLGA_PONTA_MESA or p.get("bola_fixa"):
            break                               # com a bola fixa não se pode subir o impacto
        p["dj2_impacto"] -= 0.5                 # sobe a ponta e volta a ajustar os tempos
    if verbose:
        print(f"  tempos ajustados: backswing {p['t_back']:.2f} s, downswing {p['t_down']:.2f} s, "
              f"follow-through {p['t_follow']:.2f} s; J2 no impacto {p['dj2_impacto']:+.1f} deg")
    return p


def check_v3(tr, verbose=True):
    t, qs, sf, m = tr["t"], tr["qs"], C.SAFETY_FACTOR, C.JOINT_LIMIT_MARGIN
    qd = np.gradient(qs, t, axis=0)
    v_tcp = np.linalg.norm(np.gradient(tr["pos"], t, axis=0), axis=1)
    v_fl = np.linalg.norm(np.gradient(tr["pos_flange"], t, axis=0), axis=1)
    lo = np.array([l for l, _ in C.JOINT_LIMITS])
    hi = np.array([h for _, h in C.JOINT_LIMITS])
    folga = np.minimum(qs - lo, hi - qs).min(axis=0)
    z_robo = min(tr["juntas"][:, :, 2].min(), tr["pos_flange"][:, 2].min())
    z_ponta = tr["pos"][:, 2].min()
    r_base = np.linalg.norm(np.vstack([tr["pos"], tr["pos_flange"]])[:, :2], axis=1).min()
    desce = tr["pos"][tr["k_imp"], 2] - z_ponta
    res = {
        "vel. juntas": np.abs(qd).max() <= sf * C.JOINT_SPEED_MAX,
        "vel. pinça": v_tcp.max() <= sf * C.TCP_SPEED_MAX,
        "limites juntas": folga.min() >= m,
        "robô acima da mesa": z_robo >= C.TABLE_Z + C.TABLE_CLEARANCE,
        "ponta acima da mesa": z_ponta >= C.TABLE_Z + FOLGA_PONTA_MESA,
        "fora da base": r_base >= C.BASE_RADIUS,
    }
    if verbose:
        j = int(np.abs(qd).max(axis=0).argmax()); jl = int(folga.argmin())
        ok = lambda k: "OK" if res[k] else "FALHA"
        print(f"  vel. juntas        : máx {np.abs(qd).max():6.1f} deg/s (J{j + 1}; limite c/ margem {sf * C.JOINT_SPEED_MAX:.0f})  {ok('vel. juntas')}")
        print(f"  vel. pinça (TCP)   : máx {v_tcp.max():6.1f} mm/s (limite c/ margem {sf * C.TCP_SPEED_MAX:.0f})  {ok('vel. pinça')}")
        print(f"  vel. flange (info) : máx {v_fl.max():6.1f} mm/s")
        print(f"  limites juntas     : menor folga {folga.min():5.1f} deg (J{jl + 1}, mínimo {m:.0f})  {ok('limites juntas')}")
        print(f"  robô acima da mesa : z mín {z_robo:6.1f} mm (mínimo {C.TABLE_CLEARANCE:.0f})  {ok('robô acima da mesa')}")
        print(f"  ponta acima mesa   : z mín {z_ponta:6.1f} mm (mínimo {FOLGA_PONTA_MESA:.0f}); desce {desce:.1f} mm depois do impacto  {ok('ponta acima da mesa')}")
        print(f"  fora da base       : r mín {r_base:6.1f} mm  {ok('fora da base')}")
        print(f"  impacto            : ponta a {v_tcp[tr['k_imp']]:.0f} mm/s, bola em {np.round(tr['bola'], 1)}")
    return all(res.values()), dict(qd=qd, v_tcp=v_tcp, v_flange=v_fl, res=res)


def load_demo():
    d = np.loadtxt(DEMO, delimiter=",", comments="#", skiprows=6)
    return from_joints(d[:, 0], d[:, 1:7])


def plot(tr, ex, sufixo=""):
    import matplotlib.pyplot as plt

    os.makedirs(REPORT, exist_ok=True)
    demo = load_demo()
    rot = tr["params"]["q_topo"][0] - Q_TOPO[0]          # a V3 pode estar rodada na base (bola_em)
    if abs(rot) > 0.1:
        demo = from_joints(demo["t"], demo["qs"] + np.array([rot, 0, 0, 0, 0, 0]))
    t = tr["t"]
    # tempo da demo alinhado pelo impacto (demo: 2.80 s)
    td = demo["t"] - 2.80 + t[tr["k_imp"]]

    fig, ax = plt.subplots(3, 2, figsize=(12, 8), sharex=True)
    for j in range(6):
        a = ax.flat[j]
        a.plot(t, tr["qs"][:, j], label="V3")
        a.plot(td, demo["qs"][:, j], "o", ms=4, label="demonstração")
        a.axvline(t[tr["k_imp"]], color="gray", ls=":")
        a.set_ylabel(f"J{j + 1} [deg]")
        a.grid(alpha=.3)
    ax[0, 0].legend(fontsize=8)
    ax[2, 0].set_xlabel("t [s]"); ax[2, 1].set_xlabel("t [s]")
    fig.tight_layout()
    fig.savefig(os.path.join(REPORT, f"v3_juntas{sufixo}.png"), dpi=150)

    fig = plt.figure(figsize=(12, 5.5))
    a3 = fig.add_subplot(1, 2, 1, projection="3d")
    a3.plot(*tr["pos"].T, label="V3: ponta da pinça")
    a3.plot(*demo["pos"].T, "o--", ms=3, label="demonstração")
    for k in np.linspace(0, len(t) - 1, 12).astype(int):
        a3.plot(*np.array([tr["pos_flange"][k], tr["pos"][k]]).T, color="tab:orange", lw=3)
        P = np.vstack([[0, 0, 0], tr["juntas"][k]])
        a3.plot(*P.T, color="k", lw=1, alpha=.3)
    a3.scatter(*tr["bola"], s=60, color="w", edgecolor="k", label="bola")
    a3.set_xlabel("x"); a3.set_ylabel("y"); a3.set_zlabel("z"); a3.legend(fontsize=8)
    a2 = fig.add_subplot(1, 2, 2)
    a2.plot(t, ex["v_tcp"], label="pinça (TCP)")
    a2.plot(t, ex["v_flange"], label="flange")
    a2.axhline(C.SAFETY_FACTOR * C.TCP_SPEED_MAX, color="r", ls="--", label="limite c/ margem")
    a2.axvline(t[tr["k_imp"]], color="gray", ls=":")
    a2.set_xlabel("t [s]"); a2.set_ylabel("mm/s"); a2.legend(fontsize=8); a2.grid(alpha=.3)
    fig.tight_layout()
    fig.savefig(os.path.join(REPORT, f"v3_caminho{sufixo}.png"), dpi=150)
    print(f"  gráficos em {os.path.normpath(REPORT)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bola-demo", action="store_true",
                    help="bola onde a demonstração passava (ensaios de 6/11 às 12:5x); por omissão: a bola do lab")
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()
    p = ajustar() if args.bola_demo else ajustar(bola_em(ponta(Q_BOLA_LAB)))
    print(f"  bola: {'demonstração' if args.bola_demo else 'marcada no lab, q = ' + str(Q_BOLA_LAB)}")
    print(f"  tempos: backswing {p['t_back']:.2f} s, downswing {p['t_down']:.2f} s, follow-through {p['t_follow']:.2f} s")
    tr = swing_v3(p)
    print(f"  {len(tr['t'])} pontos, {tr['t'][-1]:.2f} s (impacto em t = {tr['t'][tr['k_imp']]:.2f} s)")
    ok, ex = check_v3(tr)
    print(f"  => swing V3 {'VIÁVEL' if ok else 'NÃO VIÁVEL'}")
    if args.plot:
        plot(tr, ex, sufixo="" if args.bola_demo else "_bola_lab")


if __name__ == "__main__":
    main()
