"""Robô com a IMU na pinça: modelo MuJoCo, cinemática e referencial da IMU.

- A cinemática direta é a do Lab 2 (Lab2/src/kinematics.py), validada contra
  o controlador. É ela que dá a verdade (orientação e posição da IMU) a
  partir das juntas lidas do robô.
- O modelo MuJoCo é o do Lab 2, com o dedo do lado +y escondido e, no lugar
  dele, o suporte impresso (STL de modelos/suporte_imu/) e uma caixa do
  tamanho do Arduino Uno com as torres de espaçadores. Serve para procurar colisões.

Montagem (medida no laboratório a 16/11): a pinça fica ABERTA quando parada
(16 mm entre o carro e o outro dedo), por isso o suporte está 8,1 mm mais para
fora do que com a pinça fechada. O Uno aparafusa-se à parede sem o acrílico,
em torres de 16 mm; tem 14 mm de altura com as fichas.

Referencial da IMU no flange (R_F_IMU, P_F_IMU): a placa Grove fica aparafusada
à parede pelos seus furos, por isso a POSIÇÃO do chip vem do desenho (suporte_imu.scad e
ficheiro Eagle da Seeed). A ORIENTAÇÃO dos eixos do chip assume a convenção da
InvenSense (ponto do pino 1 em cima à esquerda: x para a direita, y para cima,
z para fora), com o chip rodado 180° na placa, como no ficheiro Eagle. Fica por
confirmar no ensaio de poses (robo/ensaios.py, "poses").
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

LAB3 = Path(__file__).resolve().parents[2]
LAB2_SRC = LAB3.parent / "Lab2" / "src"
sys.path.insert(0, str(LAB2_SRC))
import config as C2                      # noqa: E402  (Lab 2: limites do robô)
from kinematics import fk, ik_numeric    # noqa: E402

CENA_LAB2 = LAB3.parent / "Lab2" / "models" / "lite6" / "swing_scene.xml"
STL_SUPORTE = LAB3 / "modelos" / "suporte_imu" / "suporte_imu.stl"

# ------------------------------------------------------------------ suporte
CORPO_Z = 57.6          # mm, face inferior do corpo da pinça (STEP do fabricante)
ABERTURA = 8.1          # mm, quanto o dedo sai com a pinça aberta (estado parado, medido)
PAREDE_Y = ABERTURA + 11.1 + 3.0   # mm, face de fora da parede do suporte
# Arduino Uno nas torres de espaçadores (medido): 16 mm de torre + 14 mm de Uno
# com fichas; 68,6 x 53,3 mm, mais ~6 mm de ficha USB a sair de um dos lados.
KIT_ESP = 16.0 + 14.0   # mm, da parede ao ponto mais alto do Uno
KIT_LARG, KIT_ALT = 68.6 + 2 * 6.0, 53.3
KIT_Z = CORPO_Z + 3.0 + KIT_ALT / 2   # mm, centro do Uno ao longo do eixo da ferramenta

# ------------------------------------------------------------------ IMU
# Placa da IMU aparafusada à parede, debaixo do Uno, com a ficha Grove do lado +x
# (suporte_imu.scad): aresta do lado do chip em x = 4,2 mm, centro a 24,6 mm
# abaixo da face do corpo; chip a 6,16 mm dessa aresta (ficheiro Eagle da Seeed).
IMU_X0, IMU_CHIP_DX, IMU_Z, IMU_ESP = 4.2, 6.16, 24.6, 1.75
# Colunas = eixos do chip escritos no referencial do flange (orientação: hipótese).
R_F_IMU = np.array([[1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0],
                    [0.0, -1.0, 0.0]])
# chip em cima da placa (0,9 mm de altura): centro 0,45 mm acima da placa
P_F_IMU = np.array([IMU_X0 + IMU_CHIP_DX, PAREDE_Y + IMU_ESP + 0.45, CORPO_Z + IMU_Z])


def T_F_IMU():
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R_F_IMU, P_F_IMU
    return T


def fk_imu(q_deg):
    """Juntas (graus) -> (R mundo<-IMU, posição da IMU em mm)."""
    T = fk(q_deg) @ T_F_IMU()
    return T[:3, :3], T[:3, 3]


def flange_para_imu(R_imu, p_imu):
    """Pose pretendida da IMU -> pose do flange (4x4, mm)."""
    T = np.eye(4)
    T[:3, :3] = R_imu @ R_F_IMU.T
    T[:3, 3] = p_imu - T[:3, :3] @ P_F_IMU
    return T


def ik_imu(R_imu, p_imu, q0):
    q, ok = ik_numeric(flange_para_imu(R_imu, p_imu), q0)
    return q, ok


# ------------------------------------------------------------------ MuJoCo
def modelo(com_suporte=True):
    """Compila a cena do Lab 2 com o suporte e o kit no lugar do dedo +y."""
    import mujoco
    spec = mujoco.MjSpec.from_file(str(CENA_LAB2))
    spec.geom("bola").rgba = [0, 0, 0, 0]          # a bola de golfe do Lab 2 não entra aqui
    if com_suporte:
        dedo = spec.geom("gripper_right_finger")
        dedo.contype, dedo.conaffinity, dedo.rgba = 0, 0, [0, 0, 0, 0]
        spec.add_mesh(name="suporte_imu", file=str(STL_SUPORTE), scale=[1e-3] * 3)
        pinca = spec.body("gripper_body")
        sup = pinca.add_body(name="suporte_imu", pos=[0, ABERTURA / 1000, CORPO_Z / 1000])
        # o .scad roda a peça 180° em torno de x no fim (para a ver como no robô)
        sup.add_geom(name="suporte_imu", type=mujoco.mjtGeom.mjGEOM_MESH, meshname="suporte_imu",
                     quat=[0, 1, 0, 0], rgba=[0.95, 0.55, 0.15, 1], contype=1, conaffinity=1, group=0)
        sup.add_geom(name="kit_arduino", type=mujoco.mjtGeom.mjGEOM_BOX,
                     size=[KIT_LARG / 2000, KIT_ESP / 2000, KIT_ALT / 2000],
                     pos=[0, (PAREDE_Y - ABERTURA + KIT_ESP / 2) / 1000, (KIT_Z - CORPO_Z) / 1000],
                     rgba=[0.15, 0.45, 0.8, 0.85], contype=1, conaffinity=1, group=0)
        sup.add_site(name="imu", pos=(P_F_IMU - [0, ABERTURA, CORPO_Z]) / 1000, size=[0.004] * 3,
                     rgba=[1, 0.1, 0.1, 1])
        for outro in ("gripper_body", "gripper_left_finger", "link6", "link5"):
            spec.add_exclude(bodyname1="suporte_imu", bodyname2=outro)
    m = spec.compile()
    return m, mujoco.MjData(m)


def pose_mj(m, d, q_deg):
    import mujoco
    d.qpos[:6] = np.radians(q_deg)
    d.qvel[:] = 0.0
    mujoco.mj_forward(m, d)


def folgas(m, d, qs, alcance_mm=150.0, passo=5):
    """Menor distância robô-mesa e robô-robô (incluindo suporte e kit) ao longo
    de qs. Devolve (mesa, próprio, pares mais próximos)."""
    import mujoco
    nome = lambda g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or f"geom{g}"
    col = m.geom_contype > 0
    m.geom_margin[col] = alcance_mm / 1000
    mesa = np.full(len(qs), np.nan)
    proprio = np.full(len(qs), np.nan)
    pares = {}
    for k in range(0, len(qs), passo):
        pose_mj(m, d, qs[k])
        for g1, g2 in {(c.geom1, c.geom2) for c in d.contact[:d.ncon]}:
            dist = mujoco.mj_geomDistance(m, d, g1, g2, alcance_mm / 1000, None) * 1000
            par = tuple(sorted((nome(g1), nome(g2))))
            alvo = mesa if "mesa" in par else proprio
            alvo[k] = np.nanmin([alvo[k], dist])
            if par not in pares or dist < pares[par][0]:
                pares[par] = (dist, k)
    m.geom_margin[col] = 0.0
    return mesa, proprio, pares
