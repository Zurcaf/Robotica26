"""Suporte da IMU em STEP (sólido B-rep), para abrir em CAD.

O OpenSCAD não exporta STEP, por isso a peça é refeita aqui em CadQuery com os
mesmos parâmetros de suporte_imu.scad (o .scad continua a ser a referência; se
mudar, mudar aqui também). Mesma orientação do STL: como no robô, base em cima.

    pip install cadquery          (num ambiente à parte; não é dependência do Lab 3)
    python suporte_imu_step.py    -> suporte_imu.step, e compara o volume com o STL
"""
from pathlib import Path

import cadquery as cq
import numpy as np

AQUI = Path(__file__).resolve().parent

# --- parâmetros (iguais aos de suporte_imu.scad)
dedo_larg, dedo_prof, furo_y, furo_dx, lamina_y = 20, 15.3, 7.7, 12, 11.1
carro_larg, rebordo, folga_carro = 17, (1.5, 1.5), 0.2
m25_passagem, m25_cabeca, sob_cabeca, base_alt = 3.0, 5.4, 1.2, 7
parede_esp, parede_larg, placa_topo, uno_margem, folga_corpo, m3_passagem = 3, 74, 3, 3, 0.6, 3.4
imu_comp, imu_larg, imu_saida, imu_z, m3_aperto = 40, 20, 7.2, 24.6, 1.6
nervura_esp, nervura_alt, rasgo, numero = 3, 38, (2.2, 5), "1"
uno = [(14.0, 2.5), (15.2, 50.8), (66.0, 7.6), (66.0, 35.6)]
uno_larg, uno_alt = 68.6, 53.3
parede_alt = placa_topo + uno_alt + uno_margem
imu_x1 = parede_larg / 2 + imu_saida
imu_x0 = imu_x1 - imu_comp
imu_furos = [(14.2, imu_larg / 2 + m3_aperto), (14.2, -imu_larg / 2 - m3_aperto), (-m3_aperto, 0)]


def caixa(x0, x1, y0, y1, z0, z1):
    return cq.Workplane("XY").box(x1 - x0, y1 - y0, z1 - z0, centered=False).translate((x0, y0, z0))


def cil_y(x, z, d, y0, y1):
    """Cilindro ao longo de Y, de y0 a y1, centrado em (x, z)."""
    return cq.Workplane("XZ", origin=(0, y0, 0)).center(x, z).circle(d / 2).extrude(-(y1 - y0))


def cil_z(x, y, d, z0, z1):
    return cq.Workplane("XY", origin=(0, 0, z0)).center(x, y).circle(d / 2).extrude(z1 - z0)


peca = caixa(-dedo_larg / 2, dedo_larg / 2, 0, dedo_prof, 0, base_alt)
lb = dedo_larg / 2 - carro_larg / 2 - folga_carro
for x0 in (carro_larg / 2 + folga_carro, -dedo_larg / 2):
    peca = peca.union(caixa(x0, x0 + lb, 0, dedo_prof, -rebordo[1], 0.01))
peca = peca.union(caixa(-parede_larg / 2, parede_larg / 2, lamina_y, lamina_y + parede_esp, folga_corpo, parede_alt))
for s in (1, -1):        # nervuras: triângulo no plano YZ, 3 mm de espessura em X
    xc = s * (dedo_larg / 2 + nervura_esp / 2)
    tri = (cq.Workplane("YZ", origin=(xc - nervura_esp / 2, 0, 0))
           .polyline([(0, folga_corpo), (lamina_y + 0.1, folga_corpo), (lamina_y + 0.1, nervura_alt)]).close()
           .extrude(nervura_esp))
    peca = peca.union(tri)

for s in (1, -1):        # parafusos M2,5 da pinça
    x = s * furo_dx / 2
    peca = peca.cut(cil_z(x, furo_y, m25_passagem, -rebordo[1] - 1, base_alt + 1))
    peca = peca.cut(cil_z(x, furo_y, m25_cabeca, sob_cabeca, base_alt + 1))
for hx, hz in uno:       # parafusos M3 das torres do Uno (padrão e espelho)
    for s in (1, -1):
        peca = peca.cut(cil_y(s * (hx - uno_larg / 2), placa_topo + hz, m3_passagem, lamina_y - 1, lamina_y + parede_esp + 1))
for fx, fz in imu_furos:  # parafusos M3 que apertam a placa da IMU
    peca = peca.cut(cil_y(imu_x0 + fx, imu_z + fz, m3_passagem, lamina_y - 1, lamina_y + parede_esp + 1))
for s in (1, -1):        # rasgos para a abraçadeira
    for dz in (0, 9):
        x = s * (parede_larg / 2 - 3) - rasgo[0] / 2
        z = parede_alt - 18 + dz
        peca = peca.cut(caixa(x, x + rasgo[0], lamina_y - 1, lamina_y + parede_esp + 1, z, z + rasgo[1]))
# número gravado 0,6 mm na face de baixo da base (lê-se de baixo, como no .scad)
txt = (cq.Workplane("XY", origin=(0, dedo_prof / 2 - 2, base_alt - 0.6))
       .text(numero, 6, 1.0, kind="bold", halign="center", valign="center"))
peca = peca.cut(txt)

# a mesma rotação final do .scad: 180° em torno de x
peca = peca.rotate((0, 0, 0), (1, 0, 0), 180)
cq.exporters.export(peca, str(AQUI / "suporte_imu.step"))


def volume_stl(caminho):
    d = np.fromfile(caminho, dtype=np.uint8)
    n = int(np.frombuffer(d[80:84].tobytes(), np.uint32)[0])
    rec = np.frombuffer(d[84:84 + 50 * n].tobytes(), dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")]))
    v = rec["v"].astype(float)
    return abs(np.einsum("ij,ij->i", v[:, 0], np.cross(v[:, 1], v[:, 2])).sum()) / 6, v.reshape(-1, 3)


vs, pts = volume_stl(AQUI / "suporte_imu.stl")
bb = peca.val().BoundingBox()
print(f"STEP: volume {peca.val().Volume():.0f} mm³, caixa x[{bb.xmin:.1f}, {bb.xmax:.1f}] "
      f"y[{bb.ymin:.1f}, {bb.ymax:.1f}] z[{bb.zmin:.1f}, {bb.zmax:.1f}]")
print(f"STL:  volume {vs:.0f} mm³, caixa x[{pts[:, 0].min():.1f}, {pts[:, 0].max():.1f}] "
      f"y[{pts[:, 1].min():.1f}, {pts[:, 1].max():.1f}] z[{pts[:, 2].min():.1f}, {pts[:, 2].max():.1f}]")
