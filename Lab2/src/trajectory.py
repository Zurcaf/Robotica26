"""
Tarefa 2 — Trajetória de swing para o Lite 6.

Uso (a partir de Lab2/):
    python src/trajectory.py            # gera, verifica viabilidade e imprime o resumo
    python src/trajectory.py --plot     # + gráficos em report/traj_*.png
    python src/trajectory.py --plot --golfista   # swing GOLFISTA (juntas) -> report/traj_golfista_*.png
    python src/trajectory.py --t-down 1.0 --raio 180

ANALOGIA COM O LAB 1
--------------------
No Lab 1 o swing era um pêndulo duplo: braço (ombro) + pulso. Aqui a garra faz
de cabeça do taco:

  * "braço"  -> a garra percorre um ARCO de raio R num plano vertical, centrado
                num "ombro" virtual por cima da bola; phi = 0 na bola.
  * "pulso"  -> a garra INCLINA phi + psi no plano do swing, como um taco;
                psi é o "lag" do pulso, armado no topo e solto tarde
                (f_release), como no Lab 1.

ORIENTAÇÃO SEM PIÃO
-------------------
A rotação é feita em torno de n2 = z0 x d (perpendicular ao eixo da pinça z0 e
à direção do swing d). Como n2 é perpendicular ao eixo da pinça, a pinça
inclina para o lado sem girar sobre si própria. A 1.ª versão rodava em torno da
normal ao plano (eixo x); com a pinça inclinada 45 deg, cos 45 = 71 % dessa
rotação era a pinça a girar como um pião (J6 a fazer 124 deg). Agora J6 ~39 deg,
o antebraço (J4) faz a inclinação lateral, e o flange anda mais devagar.

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
from kinematics import R_from_rpy, fk, ik_numeric, ik_pos3, pose_from_T, rpy_from_R, tool_transform
from robot import pose_sdk

HERE = os.path.dirname(os.path.abspath(__file__))
REPORT = os.path.join(HERE, "..", "report")

DEFAULTS = dict(
    # bola a 190 mm (e não 250): com a pinça o flange fica ~60 mm mais longe da
    # base e a 250 mm o topo do swing saía do alcance.
    bola=(190.0, 0.0, 80.0),     # mm, posição fixa da bola (referencial da base)
    direcao=(0.0, 1.0, 0.0),     # direção do alvo (horizontal)
    raio=150.0,                  # mm, raio do arco ("comprimento do braço+taco"); ignorado com ombro
    # Centro do arco ("ombro" do golfista). None = por cima da bola (bola + raio·ẑ,
    # plano vertical). Com um ponto, o plano do swing é o que contém o ombro, a
    # bola e a direção do alvo (inclinado, como num swing real), o raio é a
    # distância ombro-bola e o golfista "é" o robô (o ombro J2 está em (0,0,243)).
    ombro=None,
    # topo/fim limitados por J5 (limite 124 deg): com -100/80 e pinça, J5 chegava a 124
    phi_top=-90.0,               # graus, ângulo no topo do backswing
    # fim do follow-through: com 70 deg o robô parava SEMPRE com C31 (deteção de
    # colisão) em phi ~69 deg, braço esticado para o lado (lab 5/11); com 50 deg
    # correu completo a lento 1, 1.5 e 2 (data/LAB_2026-11-05.md)
    phi_end=50.0,                # graus, ângulo final do follow-through
    psi_top=30.0,                # graus, "pulso armado" no topo
    f_release=0.6,               # fração do downswing em que o pulso solta
    # Quanto da rotação do arco (phi) entra na orientação da pinça. A pinça é a
    # CABEÇA do taco, sem cabo: não tem de rodar com o arco. k_phi = 1 roda
    # phi + psi (até 120 deg => J6 enrola 124 deg e J5 abana 56 deg); k_phi = 0
    # só roda com o "pulso" psi e percorre o arco quase em translação.
    k_phi=1.0,
    # --- modo "juntas": o golfista É o robô. O swing desenha-se nas juntas e a
    # trajetória da pinça (posição + orientação em função do tempo, o que a
    # Tarefa 2 pede) sai da FK. Sem IK numérica, sem pulso a enrolar.
    # `keyframes` são poses de junta (J1..J6, graus) gravadas ao mexer o robô à
    # mão no UFactory Studio (vídeo de 6 nov). O tronco J1 faz de coordenada do
    # arco: phi = -J1 (backswing para +y, J1 > 0). Cada outra junta é um
    # polinómio de grau `grau` em phi ajustado aos keyframes; J1 = -phi exato.
    #   J5 = polinómio(phi) + a_pulso·psi   (lag do pulso do Lab 1, opcional)
    # A bola fica onde a ponta da pinça está em phi = 0. None = modo cartesiano.
    juntas=None,
    # --- modo "híbrido": a ponta segue o arco cartesiano (bola, direcao, ombro)
    # e a orientação vem do pulso, como no vídeo de 6 nov: J4 e J6 fixos,
    # J5 = j5 + a_pulso·psi. J1-J3 por IK de posição. None = desligado.
    pulso=None,                  # ex.: dict(j4=-228, j5=62, j6=150, a_pulso=0.5)
    t_back=2.0,                  # s, duração do backswing
    t_down=2.2,                  # s, duração do downswing (limitado pela vel. de J6)
    # orientação da garra na bola: para baixo, inclinada 45 deg. A pique
    # (180, 0, 0) põe J5 = 0 => singularidade do pulso, J4/J6 a ~1900 deg/s.
    # None = "cabo radial": o eixo da pinça aponta da bola para o ombro.
    rpy_enderecio=(180.0, 45.0, 0.0),
    garra=C.GRIPPER_TCP_Z,       # mm, flange -> ponta da pinça (0 = sem pinça)
    dt=0.01,                     # s, período de amostragem (100 Hz)
)


# Preset "golfista": modo juntas com o endereço e os ganhos escolhidos com o
# mujoco_preview (sem colisão, J5 longe de 0, braço a levantar nos dois lados).
# Preset "golfista": modo juntas, como o V17 do Lab 1 (tronco = J1, ombro = J2,
# pulso = J5), com o robô DIREITO: braço quase esticado (J3 = 110 deg = 91 % da
# extensão; a 150 fica todo esticado = singularidade do cotovelo) e a pinça a pique
# no endereço. A trajetória da pinça (Tarefa 2) é a FK destes perfis.
GOLFISTA = dict(
    juntas=dict(
        q_enderecio=(0.0, 65.5, 110.0, 0.0, 50.0, 0.0),   # ponta em (390, 0, 80), pinça vertical
        a_tronco=-0.8,     # J1 = -0.8·phi: backswing para +y (alvo -y)
        b_ombro=35.0,      # J2 levanta o braço 35·(1-cos phi): topo a z ~ 400
        b_cotovelo=0.0,
        a_pulso=-1.0,      # J5 arma 30 deg no backswing, solta aos 60 % do downswing
    ),
    phi_top=-90.0, phi_end=50.0,
    # ponta a ~390 mm da base: t_down 4 s dá ~290 mm/s (limite 0.8 x 500); o
    # robô só fez ~220 a 5 nov => começar com --lento 2
    t_back=4.5, t_down=4.0,            # backswing mais lento que o downswing: pico de |v| na bola
)

# Variante híbrida (ponta num arco cartesiano inclinado, pulso fixo como no
# vídeo de 6 nov: J4 ~ -228 = taco para fora). Fica para comparação.
GOLFISTA_HIBRIDO = dict(
    bola=(350.0, 0.0, 80.0), direcao=(0.0, -1.0, 0.0), ombro=(150.0, 0.0, 350.0),
    pulso=dict(j4=-228.0, j5=62.0, j6=150.0, a_pulso=0.5),
    phi_top=-90.0, phi_end=50.0, t_back=4.5, t_down=4.0,
)

# O mesmo swing "copiado" do vídeo (modo juntas, keyframes do Studio), para
# comparação; o J4 ~ -228 e o J5/J6 do GOLFISTA vêm daqui.
GOLFISTA_VIDEO = dict(
    juntas=dict(
        # (J1, J2, J3, J4, J5, J6) lidos do Studio, do topo do backswing ao fim
        # do follow-through. J4 ~ -228 deg é o que põe o "taco para fora".
        keyframes=[
            (74.1, 14.5, 65.3, -227.9, 95.0, 176.5),
            (70.6, 16.5, 65.3, -228.3, 85.8, 171.1),
            (64.6, 22.7, 64.9, -227.0, 85.9, 170.6),
            (56.7, 30.3, 64.4, -228.7, 84.6, 169.1),
            (48.9, 36.7, 64.4, -228.0, 82.5, 167.2),
            (31.9, 49.0, 64.4, -224.3, 75.2, 164.5),
            (23.6, 54.5, 64.4, -223.2, 72.0, 161.7),
            (13.2, 59.0, 64.4, -224.5, 69.1, 157.6),
            (0.6, 62.5, 64.4, -230.5, 62.3, 151.6),
            (-22.8, 63.7, 67.7, -229.2, 49.7, 135.0),
            (-37.9, 63.6, 85.1, -222.9, 40.1, 127.7),
            (-43.5, 62.5, 95.3, -225.2, 39.0, 126.8),
            (-47.4, 59.7, 103.9, -225.8, 40.6, 126.9),
            (-49.1, 56.8, 110.0, -225.0, 41.7, 126.9),
            (-50.4, 54.6, 112.3, -225.3, 41.8, 126.9),
        ],
        grau=3,
        dq=(0.0, -10.0, 0.0, 0.0, 0.0, 0.0),   # ombro 10 deg mais alto: ponta a >= 75 mm da mesa
        j1_impacto=None,                       # fundo do arco (sai J1 ~ -20)
        a_pulso=0.0,
    ),
    # phi = -(J1 - j1_impacto): do topo (J1 = 74) ao fim (J1 = -50)
    phi_top=-94.0, phi_end=30.0,
    # arco grande (~400 mm da base): t_down 6.5 s dá ~370 mm/s na ponta, dentro
    # dos 0.8 x 500 mm/s; o robô mostrou a 5 nov que só faz ~220 => usar --lento 2
    t_back=5.0, t_down=6.5,
)

# Por omissão todos os scripts usam o arco cartesiano (V1, a que tem dados do
# robô). O GOLFISTA ativa-se com --golfista (trajectory, run_robot,
# mujoco_preview) ou com swing_trajectory(GOLFISTA).


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
    if p["juntas"] is not None:
        return _swing_juntas(p, t, phi, psi)
    B = np.asarray(p["bola"], float)
    d = np.asarray(p["direcao"], float)
    d /= np.linalg.norm(d)
    ez = np.array([0.0, 0.0, 1.0])
    if p["ombro"] is None:
        centro, raio = B + p["raio"] * ez, p["raio"]
    else:
        centro = np.asarray(p["ombro"], float)
        raio = np.linalg.norm(B - centro)
    u = (B - centro) / raio                      # do ombro para a bola ("cabo" no endereço)
    u -= d * (u @ d)                             # garante plano (d, u) ortogonal
    u /= np.linalg.norm(u)
    n = np.cross(d, -u)                          # normal ao plano do swing (= d x ẑ no caso vertical)

    pos = centro + raio * (np.sin(phi)[:, None] * d + np.cos(phi)[:, None] * u)
    if p["pulso"] is not None:
        return _swing_hibrido(p, t, phi, psi, pos, centro, raio)
    if p["rpy_enderecio"] is None:
        # eixo z da pinça (flange -> ponta) ao longo de u: a pinça "pendurada" do ombro
        ex = np.cross(d, u)                      # = -n: eixo x da pinça perpendicular ao plano
        R0 = np.column_stack([ex, np.cross(u, ex), u])
    else:
        R0 = R_from_rpy(*p["rpy_enderecio"])
    # eixo perpendicular ao eixo da pinça e à direção do swing: a pinça inclina
    # para o lado sem girar sobre si própria (ver ORIENTAÇÃO SEM PIÃO no topo).
    # Com phi < 0 (backswing) a ponta inclina para -d, como a cabeça de um taco.
    # (No caso "pendurada" n2 = u x d = normal ao plano: a pinça fica alinhada
    # com o "cabo" ombro -> ponta.)
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


def _swing_hibrido(p, t, phi, psi, pos, centro, raio):
    """
    Modo "híbrido": a PONTA segue o arco (pos, dado), a orientação vem do pulso:
    J4 e J6 fixos, J5 = j5 + a_pulso·psi (lag do Lab 1); J1-J3 por IK de posição
    (kinematics.ik_pos3, semente = ponto anterior). Sem torção do pulso, porque
    o pulso não tem de compensar a rotação do tronco.
    """
    w = p["pulso"]
    q = np.asarray(w.get("q0", (0.0, 45.0, 60.0, w["j4"], w["j5"], w["j6"])), float)
    qs, ok = [], True
    for pt, ps in zip(pos, psi):
        q_pulso = (w["j4"], w["j5"] + w.get("a_pulso", 0.0) * np.degrees(ps), w["j6"])
        q, conv = ik_pos3(pt, q_pulso, q, tool_z=p["garra"])
        ok &= conv
        qs.append(q.copy())
    qs = np.array(qs)
    Tf = np.array([fk(qq) for qq in qs])
    R = Tf[:, :3, :3]
    rpy = np.array([rpy_from_R(Ri) for Ri in R])
    return dict(t=t, pos=pos, pos_flange=Tf[:, :3, 3], R=R, centro=centro, raio=raio, qs=qs, ik_ok=ok,
                rpy=np.unwrap(rpy, period=360.0, axis=0), phi=phi, psi=psi, params=p)


def joint_fits(j):
    """
    Polinómios q_j(phi) [graus] ajustados aos keyframes, com phi = -(J1 - j1_impacto):
    J1 = j1_impacto - phi exato, as outras juntas por mínimos quadrados de grau `grau`.
    `dq` (6 valores) soma-se a todos os keyframes (ex.: levantar o ombro uns graus).
    j1_impacto = None => o J1 em que a ponta da pinça passa mais baixo (fundo do arco).
    """
    K = np.asarray(j["keyframes"], float) + np.asarray(j.get("dq", np.zeros(6)), float)
    j1_imp = j.get("j1_impacto")
    if j1_imp is None:
        fits0 = [np.poly1d(np.polyfit(-K[:, 0], K[:, i], j.get("grau", 3))) for i in range(1, 6)]
        j1 = np.linspace(K[:, 0].min(), K[:, 0].max(), 400)
        Tg = tool_transform(j.get("garra", C.GRIPPER_TCP_Z))
        z = [(fk([a, *[f(-a) for f in fits0]]) @ Tg)[2, 3] for a in j1]
        j1_imp = float(j1[int(np.argmin(z))])
    ph = -(K[:, 0] - j1_imp)
    return [np.poly1d([-1.0, j1_imp])] + [np.poly1d(np.polyfit(ph, K[:, i], j.get("grau", 3))) for i in range(1, 6)]


def _swing_juntas(p, t, phi, psi):
    """Modo "juntas": perfis nas juntas -> FK -> trajetória da pinça (ver DEFAULTS)."""
    j = p["juntas"]
    phd = np.degrees(phi)
    if "keyframes" in j:
        fits = joint_fits(j)
        qs = np.column_stack([f(phd) for f in fits])
    else:
        # ganhos, como o V17 do Lab 1: tronco = J1, ombro = J2 (+ cotovelo J3), pulso = J5
        #   J1 = q1 + a_tronco·phi          J2 = q2 - b_ombro·(1 - cos phi)
        #   J3 = q3 - b_cotovelo·(1 - cos phi)
        qs = np.tile(np.asarray(j["q_enderecio"], float), (len(t), 1))
        qs[:, 0] += j["a_tronco"] * phd
        qs[:, 1] -= j["b_ombro"] * (1.0 - np.cos(phi))
        qs[:, 2] -= j.get("b_cotovelo", 0.0) * (1.0 - np.cos(phi))
    qs[:, 4] += j.get("a_pulso", 0.0) * np.degrees(psi)
    Tg = tool_transform(p["garra"])
    Tf = np.array([fk(q) for q in qs])
    Tp = Tf @ Tg
    pos, pos_flange, R = Tp[:, :3, 3], Tf[:, :3, 3], Tp[:, :3, :3]
    rpy = np.array([rpy_from_R(Ri) for Ri in R])
    k0 = int(np.argmin(np.abs(phi)))               # phi = 0 => bola
    p = dict(p, bola=tuple(pos[k0]))
    return dict(t=t, pos=pos, pos_flange=pos_flange, R=R, centro=None, raio=None, qs=qs,
                rpy=np.unwrap(rpy, period=360.0, axis=0), phi=phi, psi=psi, params=p)


def joint_trajectory(traj, q0=None):
    """IK numérica do FLANGE ponto a ponto (semente = ponto anterior => continuidade).
    No modo "juntas" os q já são conhecidos: devolve-os sem IK."""
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
    Verifica (i) velocidades, (ii) alcance, (iii) limites das juntas + colisões.
    Mesa e base: verifica a ponta da pinça, o flange e todas as juntas de J2 em diante.
    """
    t, pos, flange = traj["t"], traj["pos"], traj["pos_flange"]
    sf = C.SAFETY_FACTOR
    v_tcp = np.linalg.norm(np.gradient(pos, t, axis=0), axis=1)
    # com tcp_offset = 0 (o robô do lab) o controlador comanda o FLANGE, que roda
    # com a garra e anda mais depressa do que a ponta: no lab deu C24 (velocidade)
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
        if v_flange.max() > C.CART_SPEED_OBSERVED:
            print(f"  AVISO: o flange chega a {v_flange.max():.0f} mm/s; no lab o controlador não passou de "
                  f"~{C.CART_SPEED_OBSERVED:.0f} mm/s em modo servo cartesiano -> o robô vai ficar para trás "
                  f"(usar --lento >= {v_flange.max() / C.CART_SPEED_OBSERVED:.1f} para seguir à risca)")
    return all(res.values()), dict(v_tcp=v_tcp, v_flange=v_flange, qd=qd, qdd=qdd, alcance=alcance)


def plot(traj, qs, extra, sufixo=""):
    """Gráficos em report/traj{sufixo}_*.png (sufixo="_golfista" com --golfista)."""
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
    ap.add_argument("--golfista", action="store_true", help="modo juntas com o preset GOLFISTA")
    ap.add_argument("--plot", action="store_true")
    args = ap.parse_args()

    p = dict(t_down=args.t_down, raio=args.raio, garra=args.garra, phi_end=args.phi_end, k_phi=args.k_phi)
    traj = swing_trajectory({**p, **GOLFISTA} if args.golfista else p)
    qs, conv = joint_trajectory(traj)
    print(f"  {len(traj['t'])} pontos, {traj['t'][-1]:.2f} s, IK numérica {'convergiu' if conv else 'NÃO convergiu em todos os pontos'}")
    ok, extra = check_feasibility(traj, qs)
    print(f"  => trajetória {'VIÁVEL' if ok and conv else 'NÃO VIÁVEL'}")
    if args.plot:
        plot(traj, qs, extra, "_golfista" if args.golfista else "")


if __name__ == "__main__":
    main()
