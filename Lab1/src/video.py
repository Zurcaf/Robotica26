"""
Vídeo do swing em câmara lenta.

Ao contrário do viewer interativo, aqui a velocidade de reprodução é EXACTA e
escolhida por nós: basta dizer quantas vezes mais lento se quer ver.

Uso (a partir de Lab1/):
    python src/video.py                        # 8x mais lento, câmara face_on
    python src/video.py --lento 20             # 20x mais lento
    python src/video.py --camera down_the_line
    python src/video.py --gif                  # GIF em vez de MP4 (sem ffmpeg)

Grava em report/swing_<camera>.mp4 (ou .gif).
"""

import argparse
import os
import shutil
import subprocess
import numpy as np
import mujoco

from swing import load, reference, _indices, DEFAULTS

OUTDIR = "report"
FPS = 30          # fotogramas por segundo do ficheiro de vídeo


def simula_frames(p, camera, largura, altura, lento, t_fim):
    """Corre o swing e devolve a lista de fotogramas em câmara lenta."""
    m, d = load()
    qadr, vadr, uadr = _indices(m)
    p["q_range"] = m.jnt_range[[m.joint("shoulder").id, m.joint("wrist").id]]
    lo, hi = m.actuator_ctrlrange[uadr, 0], m.actuator_ctrlrange[uadr, 1]
    M_full = np.zeros((m.nv, m.nv))
    renderer = mujoco.Renderer(m, altura, largura)
    cam_id = m.camera(camera).id

    # Um fotograma a cada (1/FPS) segundos de tempo REPRODUZIDO. Como queremos
    # ver `lento` vezes mais devagar, isso corresponde a 1/(FPS*lento) segundos
    # de tempo SIMULADO por fotograma.
    dt_frame = 1.0 / (FPS * lento)
    frames = []
    proximo = 0.0
    mujoco.mj_resetData(m, d)
    while d.time < t_fim:
        if d.time >= proximo:
            renderer.update_scene(d, camera=cam_id)
            frames.append(renderer.render().copy())
            proximo += dt_frame

        q_ref, v_ref, a_ref = reference(d.time, p)
        mujoco.mj_fullM(m, d, M_full)
        M2 = M_full[np.ix_(vadr, vadr)]
        a_cmd = (a_ref + p["kd"] * (v_ref - d.qvel[vadr])
                 + p["kp"] * (q_ref - d.qpos[qadr]))
        d.ctrl[uadr] = np.clip(M2 @ a_cmd + d.qfrc_bias[vadr], lo, hi)
        mujoco.mj_step(m, d)
    return frames


def grava_mp4(frames, caminho, largura, altura):
    """Escreve os fotogramas em bruto para o ffmpeg."""
    cmd = [
        "ffmpeg", "-y", "-loglevel", "error",
        "-f", "rawvideo", "-pix_fmt", "rgb24",
        "-s", f"{largura}x{altura}", "-r", str(FPS), "-i", "-",
        "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", caminho,
    ]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    for f in frames:
        proc.stdin.write(f.tobytes())
    proc.stdin.close()
    proc.wait()


def grava_gif(frames, caminho):
    from PIL import Image
    imgs = [Image.fromarray(f) for f in frames]
    imgs[0].save(caminho, save_all=True, append_images=imgs[1:],
                 duration=int(1000 / FPS), loop=0)


def main():
    ap = argparse.ArgumentParser(description="Vídeo do swing em câmara lenta")
    ap.add_argument("--lento", type=float, default=8.0,
                    help="quantas vezes mais lento que o tempo real (def. 8)")
    ap.add_argument("--camera", default="face_on",
                    choices=["face_on", "down_the_line"])
    ap.add_argument("--largura", type=int, default=960)
    ap.add_argument("--altura", type=int, default=720)
    ap.add_argument("--gif", action="store_true", help="GIF em vez de MP4")
    args = ap.parse_args()

    p = dict(DEFAULTS)
    # até um pouco depois do follow-through (não vale a pena filmar o voo todo)
    t_fim = p["t_back"] + p["t_pause"] + 1.5 * p["t_down"] + 0.35

    print(f"a simular e a renderizar a {args.lento:g}x mais lento...")
    frames = simula_frames(p, args.camera, args.largura, args.altura,
                           args.lento, t_fim)
    os.makedirs(OUTDIR, exist_ok=True)

    if args.gif or not shutil.which("ffmpeg"):
        caminho = os.path.join(OUTDIR, f"swing_{args.camera}.gif")
        grava_gif(frames, caminho)
    else:
        caminho = os.path.join(OUTDIR, f"swing_{args.camera}.mp4")
        grava_mp4(frames, caminho, args.largura, args.altura)

    dur = len(frames) / FPS
    print(f"  {len(frames)} fotogramas, {dur:.1f} s de vídeo para "
          f"{t_fim:.2f} s de swing  ->  {caminho}")


if __name__ == "__main__":
    main()
