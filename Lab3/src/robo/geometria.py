"""Robô com a IMU na pinça: modelo MuJoCo, cinemática e referencial da IMU.

- A cinemática direta é a do Lab 2 (Lab2/src/kinematics.py), validada contra
  o controlador. É ela que dá a verdade (orientação e posição da IMU) a
  partir das juntas lidas do robô.
- O modelo MuJoCo é o do Lab 2, com o dedo do lado +y escondido e, no lugar
  dele, o suporte impresso (STL de modelos/suporte_imu/) e uma caixa do
  tamanho do kit (Arduino + acrílico). Serve para procurar colisões.

Referencial da IMU no flange (R_F_IMU, P_F_IMU): é uma HIPÓTESE, até ser
medido no robô com o ensaio de poses (robo/ensaios.py, "poses"). Assume-se o
chip no centro da placa do Arduino, com z para fora da placa (= y da
ferramenta) e x ao longo da largura da placa (= x da ferramenta).
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
PAREDE_Y = 11.1 + 5.0   # mm, face de fora da parede do suporte
# Kit e IMU como no contexto de suporte_imu.scad (estimados; medir no laboratório):
# acrílico 76 x 59,3 mm encostado à parede, IMU logo a seguir ao acrílico,
# Arduino 15 mm mais fora, com as fichas a sair mais ~12 mm.
KIT_ESP = 28.0          # mm, da parede à ponta das fichas do Arduino
KIT_LARG, KIT_ALT = 76.0, 59.3
KIT_Z = CORPO_Z + 7.0 + KIT_ALT / 2   # mm, centro do kit ao longo do eixo da ferramenta

# ------------------------------------------------------------------ IMU
# Colunas = eixos da IMU escritos no referencial do flange (hipótese).
R_F_IMU = np.array([[1.0, 0.0, 0.0],
                    [0.0, 0.0, 1.0],
                    [0.0, -1.0, 0.0]])
P_F_IMU = np.array([0.0, PAREDE_Y + 4.5, CORPO_Z + 41.0])   # mm: chip entre acrílico e Arduino


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
        sup = pinca.add_body(name="suporte_imu", pos=[0, 0, CORPO_Z / 1000])
        # o .scad roda a peça 180° em torno de x no fim (para a ver como no robô)
        sup.add_geom(name="suporte_imu", type=mujoco.mjtGeom.mjGEOM_MESH, meshname="suporte_imu",
                     quat=[0, 1, 0, 0], rgba=[0.95, 0.55, 0.15, 1], contype=1, conaffinity=1, group=0)
        sup.add_geom(name="kit_arduino", type=mujoco.mjtGeom.mjGEOM_BOX,
                     size=[KIT_LARG / 2000, KIT_ESP / 2000, KIT_ALT / 2000],
                     pos=[0, (PAREDE_Y + KIT_ESP / 2) / 1000, (KIT_Z - CORPO_Z) / 1000],
                     rgba=[0.15, 0.45, 0.8, 0.85], contype=1, conaffinity=1, group=0)
        sup.add_site(name="imu", pos=(P_F_IMU - [0, 0, CORPO_Z]) / 1000, size=[0.004] * 3,
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
