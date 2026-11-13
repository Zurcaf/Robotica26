"""
Junta o vídeo de um ensaio com os dados gravados nesse ensaio (painel ao lado).

Uso (a partir de Lab2/):
    python src/video_dados.py VIDEO.mov data/run_servo_lento2_golfista_perto_rep2.csv \
        --lento 2 --titulo "Metade da velocidade" --out videos/lento2.mp4

Sincronização: o instante em que o swing começa no vídeo é o que maximiza a
correlação entre o movimento na imagem (diferença entre fotogramas) e a
velocidade do flange medida pelo robô. Pode ser imposto com --inicio [s].

Painel: juntas planeadas (tronco J1, ombro J2, pulso J5), velocidade do flange
medida e erro de posição (medido - pedido), com um cursor no instante atual.
Precisa de ffmpeg no PATH.
"""
import argparse
import os
import subprocess

import numpy as np

from trajectory import GOLFISTA_PERTO, joint_trajectory, swing_trajectory

W, H, FPS = 1920, 1080, 30
VW = 608                      # largura do vídeo do telemóvel (vertical) depois de escalado
PW = W - VW                   # largura do painel
COR = {"J1": "#2a78d6", "J2": "#eb6834", "J5": "#1baf7a", "v": "#2a78d6", "e": "#eb6834"}
MARGEM = 1.5                  # s de vídeo antes e depois do swing


def duracao(video):
    return float(subprocess.check_output(["ffprobe", "-v", "error", "-show_entries", "format=duration",
                                          "-of", "csv=p=0", video]).decode())


def movimento(video, fps=15, w=64, h=114):
    """Energia de movimento por fotograma (diferença média entre fotogramas cinzentos pequenos)."""
    raw = subprocess.check_output(["ffmpeg", "-v", "error", "-i", video, "-vf", f"fps={fps},scale={w}:{h},format=gray",
                                   "-f", "rawvideo", "-"])
    fr = np.frombuffer(raw, np.uint8).reshape(-1, h, w).astype(float)
    return np.abs(np.diff(fr, axis=0)).mean(axis=(1, 2)), fps


def sincronizar(video, t, v):
    """Instante [s] do vídeo em que o swing começa: máximo da correlação movimento <-> velocidade medida."""
    m, fps = movimento(video)
    m = (m - np.median(m)) / (m.std() + 1e-9)
    n = int(t[-1] * fps)
    ref = np.interp(np.arange(n) / fps, t, v)
    ref = (ref - ref.mean()) / ref.std()
    if len(m) <= n:
        return 0.0, 0.0
    c = np.array([np.dot(m[k:k + n], ref) / n for k in range(len(m) - n)])
    return c.argmax() / fps, c.max()


def painel_base(titulo, t, q, v, e, cortes):
    """Imagem do painel (sem cursor) e a função t -> x em píxeis de cada gráfico."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(3, 1, figsize=(PW / 100, H / 100), dpi=100, sharex=True,
                           gridspec_kw=dict(left=0.09, right=0.74, top=0.86, bottom=0.07, hspace=0.16))
    fig.text(0.045, 0.945, titulo, fontsize=25, fontweight="bold", va="center")
    for nome, i in (("J1", 0), ("J2", 1), ("J5", 4)):
        ax[0].plot(t, q[:, i], color=COR[nome], lw=2.5)
    ax[0].set_ylabel("juntas planeadas [°]", fontsize=13)
    ax[1].plot(t, v, color=COR["v"], lw=2.5)
    ax[1].set_ylabel("velocidade do flange\nmedida [mm/s]", fontsize=13)
    ax[2].plot(t, e, color=COR["e"], lw=2.5)
    ax[2].set_ylabel("erro de posição\n(medido − pedido) [mm]", fontsize=13)
    ax[2].set_xlabel("t [s]", fontsize=13)
    ax[2].set_ylim(0, max(1.2, e.max() * 1.15))
    for a in ax:
        a.set_xlim(t[0], t[-1])
        a.grid(alpha=0.25, lw=0.6)
        a.spines[["top", "right"]].set_visible(False)
        a.tick_params(labelsize=11)
        for c in cortes:
            a.axvline(c, color="#52514e", lw=1, ls=":")
    topo = ax[0].get_ylim()[1]
    for x, s in zip([0.5 * cortes[0], np.mean(cortes), 0.5 * (cortes[1] + t[-1])], ["backswing", "downswing", "follow-through"]):
        ax[0].text(x, topo, s, ha="center", va="bottom", fontsize=11, color="#52514e")
    fig.canvas.draw()
    img = np.asarray(fig.canvas.buffer_rgba())[:, :, :3].copy()
    caixas = [a.get_window_extent() for a in ax]          # píxeis, origem em baixo
    plt.close(fig)
    return img, caixas


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("video")
    ap.add_argument("csv")
    ap.add_argument("--lento", type=float, required=True)
    ap.add_argument("--titulo", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--inicio", type=float, default=None, help="instante do vídeo em que o swing começa [s]")
    ap.add_argument("--enderecio", type=float, nargs=6, default=None, help="pose de endereço usada no ensaio (como em run_robot.py)")
    args = ap.parse_args()
    from PIL import Image, ImageDraw, ImageFont

    d = np.loadtxt(args.csv, delimiter=",")
    t, ref, med = d[:, 0], d[:, 1:4], d[:, 7:10]
    e = np.linalg.norm(med - ref, axis=1)
    e[2:-2] = np.median(np.column_stack([e[i:len(e) - 4 + i] for i in range(5)]), axis=1)   # tira amostras isoladas
    w = 5
    v = np.zeros_like(t)
    v[w:-w] = np.linalg.norm(med[2 * w:] - med[:-2 * w], axis=1) / (t[2 * w:] - t[:-2 * w])
    G = GOLFISTA_PERTO
    p = dict(G, t_back=G["t_back"] * args.lento, t_down=G["t_down"] * args.lento)
    if args.enderecio is not None:
        q0 = (args.enderecio[0], args.enderecio[1], args.enderecio[2], 0.0, args.enderecio[4], 0.0)
        p["juntas"] = dict(G["juntas"], q_enderecio=q0)
    traj = swing_trajectory(p)
    q = joint_trajectory(traj)[0]
    tq = traj["t"]
    cortes = (p["t_back"], p["t_back"] + p["t_down"])

    t0, corr = (args.inicio, None) if args.inicio is not None else sincronizar(args.video, t, v)
    dur = duracao(args.video)
    a, b = max(0.0, t0 - MARGEM), min(dur, t0 + t[-1] + MARGEM)
    print(f"  swing começa aos {t0:.2f} s do vídeo" + ("" if corr is None else f" (correlação {corr:.2f})") +
          f"; corte {a:.2f}-{b:.2f} s de {dur:.1f} s")

    base, caixas = painel_base(args.titulo, t, np.column_stack([np.interp(t, tq, q[:, i]) for i in range(6)]), v, e, cortes)
    fonte = lambda s: ImageFont.truetype("/System/Library/Fonts/Helvetica.ttc", s)
    f_g, f_p = fonte(30), fonte(19)
    n = int(round((b - a) * FPS))
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    ff = subprocess.Popen(
        ["ffmpeg", "-v", "error", "-y",
         "-ss", f"{a:.3f}", "-t", f"{b - a:.3f}", "-i", args.video,
         "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{PW}x{H}", "-r", str(FPS), "-i", "-",
         "-filter_complex", f"[0:v]fps={FPS},scale={VW}:{H}:force_original_aspect_ratio=increase,crop={VW}:{H}[v];[v][1:v]hstack=inputs=2[o]",
         "-map", "[o]", "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20", "-r", str(FPS), args.out],
        stdin=subprocess.PIPE)
    x_txt = int(PW * 0.765)
    for k in range(n):
        ts = np.clip(a + k / FPS - t0, t[0], t[-1])          # tempo do swing neste fotograma
        img = Image.fromarray(base.copy())
        dr = ImageDraw.Draw(img)
        i = int(np.searchsorted(t, ts))
        i = min(i, len(t) - 1)
        for c in caixas:                                      # cursor nos três gráficos
            x = c.x0 + (ts - t[0]) / (t[-1] - t[0]) * (c.x1 - c.x0)
            dr.line([(x, H - c.y1), (x, H - c.y0)], fill=(11, 11, 11), width=2)
        fase = "backswing" if ts <= cortes[0] else "downswing" if ts <= cortes[1] else "follow-through"
        dr.text((x_txt, 60), f"t = {ts:5.2f} s", font=f_g, fill=(11, 11, 11))
        dr.text((x_txt, 100), fase, font=f_p, fill=(82, 81, 78))
        y = H - caixas[0].y1 + 20
        for nome, j, papel in (("J1", 0, "tronco"), ("J2", 1, "ombro"), ("J5", 4, "pulso")):
            dr.rectangle([x_txt, y + 8, x_txt + 22, y + 26], fill=COR[nome])
            dr.text((x_txt + 32, y), f"{nome} {np.interp(ts, tq, q[:, j]):6.1f}°", font=f_g, fill=(11, 11, 11))
            dr.text((x_txt + 32, y + 36), papel, font=f_p, fill=(82, 81, 78))
            y += 78
        y = H - caixas[1].y1 + 40
        dr.text((x_txt, y), f"{v[i]:5.0f} mm/s", font=f_g, fill=(11, 11, 11))
        y = H - caixas[2].y1 + 40
        dr.text((x_txt, y), f"{e[i]:5.2f} mm", font=f_g, fill=(11, 11, 11))
        ff.stdin.write(np.asarray(img).tobytes())
    ff.stdin.close()
    ff.wait()
    print(f"  {n} fotogramas -> {os.path.normpath(args.out)}")


if __name__ == "__main__":
    main()
