"""
Clip da simulação no MuJoCo do swing GOLFISTA_PERTO, para o vídeo dos ensaios.

Uso (a partir de Lab2/):
    python video/video_mujoco.py SAIDA.mp4

Duas câmaras lado a lado (face_on e cima), o swing à velocidade planeada, com a
folga mínima à mesa e entre elos em cada instante (mesmas contas de
mujoco_preview.py --folgas).
"""
import subprocess
import sys

import mujoco
import sys as _sys
import os as _os
_sys.path.insert(0, _os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "..", "src"))   # módulos do Lab 2

import numpy as np
from PIL import Image, ImageDraw, ImageFont

import mujoco_preview as P
from trajectory import GOLFISTA_PERTO, joint_trajectory, swing_trajectory

W, H, FPS, LADO, TOPO = 1920, 1080, 30, 960, 120
F = "/System/Library/Fonts/Helvetica.ttc"
VERDE, AZUL = (27, 127, 59), (31, 58, 95)


def main():
    out = sys.argv[1]
    traj = swing_trajectory(GOLFISTA_PERTO)
    qs = joint_trajectory(traj)[0]
    t = traj["t"]
    m, d = P.load()
    m.geom("bola").pos[:] = np.asarray(traj["params"]["bola"]) / 1000.0
    col = m.geom_contype > 0
    r = mujoco.Renderer(m, LADO, LADO)
    ff = subprocess.Popen(["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}",
                           "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", out],
                          stdin=subprocess.PIPE)
    f1, f2, f3 = ImageFont.truetype(F, 46), ImageFont.truetype(F, 28), ImageFont.truetype(F, 34)
    n = int((t[-1] + 2.0) * FPS)                       # 1 s parado no início e no fim
    for k in range(n):
        ts = float(np.clip(k / FPS - 1.0, 0.0, t[-1]))
        i = min(int(np.searchsorted(t, ts)), len(t) - 1)
        m.geom_margin[col] = 0.25                      # pares próximos -> distância exata com mj_geomDistance
        P.pose(m, d, qs[i])
        mesa = elos = np.inf
        for g1, g2 in {(c.geom1, c.geom2) for c in d.contact[:d.ncon]}:
            dist = mujoco.mj_geomDistance(m, d, g1, g2, 1.0, None) * 1000
            if "mesa" in (P.nome_geom(m, g1), P.nome_geom(m, g2)):
                mesa = min(mesa, dist)
            else:
                elos = min(elos, dist)
        m.geom_margin[col] = 0.0
        img = Image.new("RGB", (W, H), (255, 255, 255))
        for j, cam in enumerate(("face_on", "cima")):
            r.update_scene(d, camera=m.camera(cam).id)
            P.desenha_trajetoria(r.scene, traj)
            img.paste(Image.fromarray(r.render()), (j * LADO, TOPO))
        dr = ImageDraw.Draw(img)
        dr.rectangle([0, 0, 28, TOPO], fill=VERDE)
        dr.text((60, 14), "Simulated first: the planned swing in MuJoCo", font=f1, fill=VERDE)
        dr.text((60, 72), "Official 3D model of the robot · every link and the gripper checked against the table and each other",
                font=f2, fill=AZUL)
        dr.rectangle([1290, 950, W - 30, H - 26], fill=(255, 255, 255))
        dr.text((1310, 958), f"clearance to table:  {mesa:5.0f} mm", font=f3, fill=(11, 11, 11))
        dr.text((1310, 1002), f"between links:       {elos:5.0f} mm", font=f3, fill=(11, 11, 11))
        dr.rectangle([0, 1012, 1250, H], fill=AZUL)
        dr.text((40, 1026), "Minimum over the whole swing: 82 mm to the table, 25 mm between links — no contact",
                font=f2, fill=(255, 255, 255))
        ff.stdin.write(np.asarray(img).tobytes())
    ff.stdin.close()
    ff.wait()
    print(f"  {n} fotogramas -> {out}")


if __name__ == "__main__":
    main()
