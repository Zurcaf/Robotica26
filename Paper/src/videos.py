"""Vídeos para o grupo (MP4, gravados sem janela com o MuJoCo e o ffmpeg).

    .venv/bin/python src/videos.py          # escreve videos/1_limites.mp4, 2_chao.mp4, 3_mesa.mp4

1. limites: cada junta de um limite ao outro, à velocidade da ficha (mexer.py).
2. chão: o bico varre a zona útil a 0 m, para baixo, cotovelo em baixo (desenhar.py), a 2x.
3. mesa: a mesa de 2,9 x 1,1 m, camada no chão e a 1,6 m, com o volume imprimível.
"""
from __future__ import annotations

import subprocess
from pathlib import Path

import mujoco
import numpy as np

import cena
import kuka as K
import mexer
import plano as PL

PAPER = Path(__file__).resolve().parents[1]
SAIDA = PAPER / "videos"
W, H, FPS = 1280, 720, 30
FONTE = "/System/Library/Fonts/Helvetica.ttc"


def tempos(qs):
    dt = np.max(np.abs(np.diff(qs, axis=0)) / K.VEL, axis=1)
    return np.r_[0.0, np.cumsum(dt)]


def com_caminho(pts, mesa=None, zt=0.0, peca=None):
    """Cena com o caminho desenhado (um cilindro por troço reto), o volume da mesa e a peça."""
    s = cena.construir(peca)
    w = s.worldbody
    i0 = 0
    for i in range(1, len(pts)):
        fim = i == len(pts) - 1
        if fim or np.linalg.norm(np.cross(pts[i] - pts[i0], pts[i + 1] - pts[i0])) > 1e-3:
            if np.linalg.norm(pts[i] - pts[i0]) > 1:
                w.add_geom(type=mujoco.mjtGeom.mjGEOM_CAPSULE, size=[0.008],
                           fromto=np.r_[pts[i0], pts[i]] * 1e-3 + [0, 0, 0.002, 0, 0, 0.002],
                           rgba=[0.1, 0.5, 0.9, 1], contype=0, conaffinity=0)
            i0 = i
    if mesa is not None:
        x0, y0, x1, y1 = (v * 1e-3 for v in mesa)
        zt = max(zt * 1e-3, 0.01)
        w.add_geom(type=mujoco.mjtGeom.mjGEOM_BOX, size=[(x1 - x0) / 2, (y1 - y0) / 2, zt / 2],
                   pos=[(x0 + x1) / 2, (y0 + y1) / 2, zt / 2], rgba=[0.2, 0.8, 0.3, 0.15],
                   contype=0, conaffinity=0)
    m = s.compile()
    return m, mujoco.MjData(m)


def _legenda(texto):
    """Legenda em baixo à esquerda: imagem RGB e transparência (H, W, 1), feitas uma vez."""
    from PIL import Image, ImageDraw, ImageFont
    try:
        f = ImageFont.truetype(FONTE, 26)
    except OSError:
        f = ImageFont.load_default()
    im = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    dr = ImageDraw.Draw(im)
    x0, y0, x1, y1 = dr.textbbox((0, 0), texto, font=f)
    x, y = 20, H - (y1 - y0) - 35
    dr.rectangle([x - 10, y - 8, x + (x1 - x0) + 10, y + (y1 - y0) + 14], fill=(0, 0, 0, 140))
    dr.text((x - x0, y - y0), texto, font=f, fill=(255, 255, 255, 255))
    a = np.asarray(im, np.float32)
    return a[..., :3], a[..., 3:] / 255.0


def gravar(nome, m, d, t, q, texto, rapidez=1.0, az0=-60.0, rodar=40.0):
    """Grava a trajetória de juntas q (graus) nos instantes t (s), com uma legenda."""
    SAIDA.mkdir(exist_ok=True)
    out = SAIDA / f"{nome}.mp4"
    legenda, alfa = _legenda(texto)
    ff = subprocess.Popen(["ffmpeg", "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}",
                           "-r", str(FPS), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p",
                           "-crf", "20", str(out)], stdin=subprocess.PIPE)
    r = mujoco.Renderer(m, H, W)
    cam = mujoco.MjvCamera()
    cam.lookat[:] = [K.ROBO_XY[0] / 1000, K.ROBO_XY[1] / 1000 - 1.2, 1.2]
    cam.distance, cam.elevation = 7.5, -22
    dur = t[-1] / rapidez
    n = int(dur * FPS) + 1
    qr = np.radians(q)
    for k in range(n):
        tt = min(k / FPS * rapidez, t[-1])
        d.qpos[:] = [np.interp(tt, t, qr[:, j]) for j in range(6)]
        mujoco.mj_forward(m, d)
        cam.azimuth = az0 + rodar * k / max(n - 1, 1)
        r.update_scene(d, camera=cam)
        img = r.render().astype(np.float32)
        img = img * (1 - alfa) + legenda * alfa             # legenda por cima
        ff.stdin.write(img.astype(np.uint8).tobytes())
    ff.stdin.close()
    ff.wait()
    print(f"  {out.relative_to(PAPER)}: {dur:.0f} s")


def main():
    # 1. limites
    _, m, d = cena.modelo()
    alvos = np.degrees(np.array(mexer.plano_demo()))
    q = [alvos[0]]
    for a in alvos[1:]:                                   # passo de 0,5° na junta que mais anda
        n = max(1, int(np.abs(a - q[-1]).max() / 0.5))
        q += list(np.linspace(q[-1], a, n + 1)[1:])
    q = np.array(q)
    gravar("1_limites", m, d, tempos(q), q,
           "KR 240-2 C de teto: cada junta de um limite ao outro (limites do controlador, velocidades da ficha)")

    # 2. chão
    _, m, d = cena.modelo()
    xs, ys, M = PL.zona(m, d)
    pts = PL.linhas(xs, ys, M, 200.0)
    qs, partes, _ = PL.planear(pts, m, d)
    q = np.vstack([np.linspace(K.CASA, qs[0], 60), qs, np.linspace(qs[-1], K.CASA, 60)])
    m, d = com_caminho(pts)
    area = M.sum() * ((xs[1] - xs[0]) / 1000) ** 2
    gravar("2_chao", m, d, tempos(q), q,
           f"Bico sempre para baixo a 0 m: {area:.1f} m2 sem saltos nem colisoes (velocidade 2x)", rapidez=2.0)

    # 3. mesa do maior volume (volume.py): peça de 0,8 m; o robô faz a última camada
    #    por cima da peça já impressa, que conta como obstáculo
    mesa, z = [400.0, 3500.0, 3500.0, 4700.0], 800.0
    for _ in range(4):
        peca = (*mesa, z)
        _, m, d = cena.modelo(peca=peca)
        xs, ys, _ = PL.zona(m, d, z=z)
        GX, GY = np.meshgrid(xs, ys)
        R = (GX >= mesa[0]) & (GX <= mesa[2]) & (GY >= mesa[1]) & (GY <= mesa[3])
        pts = PL.linhas(xs, ys, R, 200.0, z=z)
        qs, partes, _ = PL.planear(pts, m, d)
        v = PL.verificar(qs, pts, m, d)
        print(f"  mesa {mesa}: partes {len(partes)}, sem pose {v['sem_pose']}, colisões {len(v['colisoes'])}, "
              f"salto {v['salto']:.1f}°")
        if not partes and not v["sem_pose"] and not v["colisoes"]:
            break
        mesa = [mesa[0] + 50, mesa[1] + 50, mesa[2] - 50, mesa[3] - 50]     # encolhe 5 cm a toda a volta
    else:
        raise SystemExit("a mesa não passou na verificação")
    q = np.vstack([np.linspace(K.CASA, qs[0], 60), qs, np.linspace(qs[-1], K.CASA, 60)])
    m, d = com_caminho(pts, peca=peca)
    w, h = (mesa[2] - mesa[0]) / 1000, (mesa[3] - mesa[1]) / 1000
    gravar("3_mesa", m, d, tempos(q), q,
           f"Maior volume: peca de 0,8 m numa mesa de {w:.1f} x {h:.1f} m".replace(".", ",") + f"; ultima camada sem tocar na peca (amarelo)",
           az0=-30.0, rodar=30.0)

if __name__ == "__main__":
    main()
