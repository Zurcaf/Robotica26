"""Cena MuJoCo: a sala, a placa da estrutura no teto e o KR 240-2 C pendurado, com a extrusora.

Os elos são formas simples (cilindros, cápsulas, caixas) até chegarem as malhas
do CAD. As juntas estão em radianos no MuJoCo mas com o zero e o sentido da KUKA:
qpos = radians(ângulo da consola). Os limites das juntas são os do controlador.

    .venv/bin/python src/cena.py      # escreve modelos/kr240_cela.xml

Para abrir só o modelo, com as barras de cada junta (separador Control):
    .venv/bin/python -m mujoco.viewer --mjcf=modelos/kr240_cela.xml
"""
from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation as Rot

import kuka as K

PAPER = Path(__file__).resolve().parents[1]
XML = PAPER / "modelos" / "kr240_cela.xml"
mm = 1e-3

LARANJA = [0.95, 0.45, 0.05, 1]
CINZA = [0.25, 0.25, 0.27, 1]
PAU = [0.75, 0.75, 0.78, 1]


def _quat(R):
    x, y, z, w = Rot.from_matrix(R).as_quat()
    return [w, x, y, z]


def construir(peca=None):
    """peca = (x0, y0, x1, y1, z) em mm: bloco da peça impressa até à camada z (obstáculo)."""
    s = mujoco.MjSpec()
    s.modelname = "kr240_cela"
    s.compiler.degree = False          # os limites vão em radianos (o MjSpec assume graus)
    s.option.gravity = [0, 0, 0]       # só cinemática por agora
    s.option.timestep = 0.002
    s.visual.global_.offwidth, s.visual.global_.offheight = 1600, 1200
    s.stat.center = [K.SALA[0] / 2 * mm, K.SALA[1] / 2 * mm, 1.5]
    s.stat.extent = 5.0
    w = s.worldbody

    # -------------------------------------------------------------- sala
    s.add_texture(name="grelha", type=mujoco.mjtTexture.mjTEXTURE_2D,
                  builtin=mujoco.mjtBuiltin.mjBUILTIN_CHECKER, rgb1=[0.82, 0.82, 0.80],
                  rgb2=[0.74, 0.74, 0.72], width=300, height=300)
    s.add_material(name="chao", textures=["", "grelha"], texrepeat=[8, 14])
    X, Y, Z = (v * mm for v in K.SALA)
    # Chão e paredes com 1 m de espessura, para fora da sala: com placas finas o
    # MuJoCo não apanha um elo que as atravesse de um lado ao outro.
    E = 0.5
    w.add_geom(name="chao", type=mujoco.mjtGeom.mjGEOM_BOX, size=[X / 2 + 2 * E, Y / 2 + 2 * E, E],
               pos=[X / 2, Y / 2, -E], material="chao")
    parede = [0.6, 0.65, 0.75, 0.12]
    for nome, pos, size in (("parede_esq", [-E, Y / 2, Z / 2], [E, Y / 2 + 2 * E, Z / 2]),
                            ("parede_dir", [X + E, Y / 2, Z / 2], [E, Y / 2 + 2 * E, Z / 2]),
                            ("parede_cima", [X / 2, Y + E, Z / 2], [X / 2, E, Z / 2]),
                            ("parede_baixo", [X / 2, -E, Z / 2], [X / 2, E, Z / 2])):
        w.add_geom(name=nome, type=mujoco.mjtGeom.mjGEOM_BOX, pos=pos, size=size, rgba=parede,
                   contype=0, conaffinity=0)
    # teto (quase transparente, para se ver o robô de cima), também com 1 m de espessura
    w.add_geom(name="teto", type=mujoco.mjtGeom.mjGEOM_BOX, size=[X / 2 + 2 * E, Y / 2 + 2 * E, E],
               pos=[X / 2, Y / 2, Z + E], rgba=[0.6, 0.65, 0.75, 0.04])
    w.add_light(pos=[X / 2, Y / 2, Z + 2], dir=[0, 0, -1], diffuse=[0.7, 0.7, 0.7])
    w.add_light(pos=[X + 3, -3, 4], dir=[-1, 1, -0.8], diffuse=[0.4, 0.4, 0.4])

    # placa da estrutura de suspensão, por cima do prato do robô
    T_wr = K.robo_no_mundo()
    w.add_geom(name="placa", type=mujoco.mjtGeom.mjGEOM_BOX,
               size=[K.PLACA / 2 * mm, K.PLACA / 2 * mm, 0.02],
               pos=T_wr[:3, 3] * mm + [0, 0, 0.02], rgba=[0.45, 0.45, 0.5, 1])

    # -------------------------------------------------------------- robô
    # Tudo abaixo está no referencial da KUKA (z a sair da base), em metros.
    base = w.add_body(name="base", pos=T_wr[:3, 3] * mm, quat=_quat(T_wr[:3, :3]))
    base.add_geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.45, 0.12], pos=[0, 0, 0.12], rgba=CINZA)
    base.add_site(name="robo", size=[0.02] * 3)

    def junta(corpo, nome, eixo, i):
        corpo.add_joint(name=nome, axis=eixo, range=np.radians(K.LIM[i]), limited=True,
                        armature=1.0, damping=50.0)

    l1 = base.add_body(name="A1", pos=[0, 0, 0])
    junta(l1, "A1", [0, 0, -1], 0)
    l1.add_geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.38, 0.17], pos=[0, 0, 0.41], rgba=LARANJA)
    l1.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.25, 0.30, 0.18], pos=[0.25, 0, 0.70], rgba=LARANJA)

    l2 = l1.add_body(name="A2", pos=[K.A1_OFF * mm, 0, K.D1 * mm])
    junta(l2, "A2", [0, 1, 0], 1)
    l2.add_geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.22, 0.12], pos=[0, -0.32, 0],
                quat=_quat(K.rx(np.pi / 2)), rgba=CINZA)
    # o braço fica ao lado do antebraço (como no KR 240-2: é o que deixa o A3 rodar 274°)
    l2.add_geom(type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[0.14], fromto=[0, -0.32, 0, K.L2 * mm, -0.32, 0],
                rgba=LARANJA)

    l3 = l2.add_body(name="A3", pos=[K.L2 * mm, 0, 0])
    junta(l3, "A3", [0, 1, 0], 2)
    z4 = K.SINAL_55 * K.A3_OFF * mm
    l3.add_geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.2, 0.2], quat=_quat(K.rx(np.pi / 2)),
                rgba=LARANJA)
    l3.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.22, 0.17, 0.17], pos=[-0.05, 0, z4], rgba=LARANJA)

    # A4: eixo ao longo do antebraço, desviado de z4 em relação ao A3
    l4 = l3.add_body(name="A4", pos=[0.2, 0, z4])
    junta(l4, "A4", [-1, 0, 0], 3)
    # o antebraço acaba antes do punho, para o flange poder dobrar ±125° (A5) sem lhe tocar
    l4.add_geom(type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[0.12], fromto=[0, 0, 0, K.L3 * mm - 0.55, 0, 0],
                rgba=LARANJA)
    l4.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[0.12, 0.17, 0.06],
                pos=[K.L3 * mm - 0.32, 0, 0], rgba=LARANJA)          # garfo do punho, dos lados do A5

    l5 = l4.add_body(name="A5", pos=[K.L3 * mm - 0.2, 0, 0])   # centro do punho
    junta(l5, "A5", [0, 1, 0], 4)
    l5.add_geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.11, 0.12], quat=_quat(K.rx(np.pi / 2)),
                rgba=LARANJA)
    l5.add_site(name="punho", size=[0.015] * 3, rgba=[0, 0.6, 1, 1])

    l6 = l5.add_body(name="A6", pos=[0.12, 0, 0])
    junta(l6, "A6", [-1, 0, 0], 5)
    l6.add_geom(type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.1, (K.D6 * mm - 0.12) / 2],
                pos=[(K.D6 * mm - 0.12) / 2, 0, 0], quat=_quat(K.ry(np.pi / 2)), rgba=CINZA)

    # flange, com a orientação da KUKA (x do flange para baixo em casa)
    fl = l6.add_body(name="flange", pos=[K.D6 * mm - 0.12, 0, 0], quat=_quat(K.R_F0))
    fl.add_site(name="flange", size=[0.02] * 3, rgba=[1, 1, 0, 1])

    # -------------------------------------------------------------- extrusora (ao longo do x do flange)
    dz = K.EXT_DZ * mm
    fl.add_geom(name="extrusora", type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[K.EXT_RAIO * mm],
                fromto=[-K.EXT_TRAS * mm, 0, dz, (K.EXT_BICO - 60) * mm, 0, dz], rgba=PAU)
    fl.add_geom(name="ponta_bico", type=mujoco.mjtGeom.mjGEOM_CYLINDER, size=[0.02],
                fromto=[(K.EXT_BICO - 60) * mm, 0, dz, K.EXT_BICO * mm, 0, dz], rgba=CINZA)
    T_bico = K.frame(*K.TOOL_BICO)
    fl.add_site(name="bico", pos=T_bico[:3, 3] * mm, quat=_quat(T_bico[:3, :3]), size=[0.015] * 3,
                rgba=[1, 0, 0, 1])

    # Colisões (bits contype/conaffinity): chão 1, robô e extrusora 2, paredes 4,
    # ponta do bico 8. O robô e a extrusora batem no chão, nas paredes e em si
    # próprios; a ponta do bico só bate nas paredes (no chão está de propósito).
    # Teto e placa contam como as paredes. Corpos pai-filho não colidem (regra do MuJoCo); fica
    # também de fora extrusora-punho (A5), porque o pau ainda atravessa o punho
    # (EXT_DZ por medir). Formas provisórias: colisões aproximadas até ao CAD.
    for g in s.geoms:
        g.contype, g.conaffinity = 0, 0
    s.geom("chao").contype = 1
    for nome in ("parede_esq", "parede_dir", "parede_cima", "parede_baixo", "teto", "placa"):
        s.geom(nome).contype = 4
    for b in s.bodies:
        if b.name in ("base", "A1", "A2", "A3", "A4", "A5", "A6", "flange"):
            for g in b.geoms:
                g.contype, g.conaffinity = 2, 1 | 2 | 4 | 16
    s.geom("ponta_bico").contype, s.geom("ponta_bico").conaffinity = 8, 4
    # peça impressa (bit 16): o robô e a extrusora não lhe podem tocar; a ponta do
    # bico sim (está em cima da camada de propósito). Bloco cheio, pelo lado seguro.
    if peca is not None and peca[4] > 0:
        x0, y0, x1, y1, zp = (v * mm for v in peca)
        s.worldbody.add_geom(name="peca", type=mujoco.mjtGeom.mjGEOM_BOX,
                             size=[(x1 - x0) / 2, (y1 - y0) / 2, zp / 2],
                             pos=[(x0 + x1) / 2, (y0 + y1) / 2, zp / 2], rgba=[0.9, 0.8, 0.3, 0.5],
                             contype=16, conaffinity=0)
    s.add_exclude(bodyname1="flange", bodyname2="A5")
    s.add_exclude(bodyname1="base", bodyname2="world")      # a base está aparafusada à placa e ao teto

    # barras das juntas (no viewer, separador Control), em graus com os limites
    for i in range(6):
        a = s.add_actuator(name=f"A{i + 1}", target=f"A{i + 1}", trntype=mujoco.mjtTrn.mjTRN_JOINT)
        a.set_to_position(kp=0.0)
        a.ctrlrange = np.radians(K.LIM[i])
        a.ctrllimited = True

    s.add_key(name="casa", qpos=np.radians(K.CASA), ctrl=np.radians(K.CASA))
    return s


def colisoes(m, d, q_graus):
    """Pares em colisão na pose q (graus): [(a, b), ...], com o nome do corpo do
    robô ou, para o que é fixo, o nome da peça (chao, parede_...)."""
    d.qpos[:] = np.radians(q_graus)
    mujoco.mj_kinematics(m, d)
    mujoco.mj_collision(m, d)
    nome = lambda g: m.geom(g).name if m.geom_bodyid[g] == 0 else m.body(m.geom_bodyid[g]).name
    return sorted({tuple(sorted((nome(c.geom1), nome(c.geom2)))) for c in d.contact[:d.ncon] if c.dist < 0})


def modelo(peca=None):
    s = construir(peca)
    m = s.compile()
    return s, m, mujoco.MjData(m)


def main():
    s, m, d = modelo()
    XML.parent.mkdir(parents=True, exist_ok=True)
    XML.write_text(s.to_xml())

    # o MuJoCo dá o mesmo que kuka.fk (flange e bico) em poses ao acaso?
    rng = np.random.default_rng(1)
    T_wr = K.robo_no_mundo()
    pior_f = pior_b = pior_r = 0.0
    for _ in range(500):
        q = rng.uniform(K.LIM[:, 0], K.LIM[:, 1])
        d.qpos[:] = np.radians(q)
        mujoco.mj_kinematics(m, d)
        Tf = T_wr @ K.fk(q)
        Tb = T_wr @ K.fk_bico(q)
        pior_f = max(pior_f, np.linalg.norm(d.site("flange").xpos / mm - Tf[:3, 3]))
        pior_b = max(pior_b, np.linalg.norm(d.site("bico").xpos / mm - Tb[:3, 3]))
        pior_r = max(pior_r, np.abs(d.site("bico").xmat.reshape(3, 3) - Tb[:3, :3]).max())
    print(f"Escrito {XML.relative_to(PAPER)}")
    print(f"MuJoCo contra kuka.fk em 500 poses: flange {pior_f:.1e} mm, bico {pior_b:.1e} mm, "
          f"orientação {pior_r:.1e}")


if __name__ == "__main__":
    main()
