"""
Imagens do swing (renderização offscreen) — Tarefa 1/2 do enunciado.

Guarda uma imagem por fase do swing, das duas câmaras definidas no XML
(`face_on`, de frente para o jogador, e `down_the_line`, atrás na linha do
alvo — as duas vistas com que se filma um swing), e opcionalmente as sete
fases lado a lado numa tira, que é a figura usada no relatório.

Uso (a partir de Lab1/):
    python src/renders.py                       # 7 fases x 2 câmaras -> report/pose_*.png
    python src/renders.py --strip               # + as duas tiras em report/ROB_LAB1_Report/Photos/
    python src/renders.py --modelo models/golfer_v13.xml --prefixo v13_
"""

import argparse
import os
import numpy as np
import mujoco

from swing import load, _prep, controlo, t_impacto, config, MODEL

OUTDIR = "report"
PHOTOS = os.path.join("report", "ROB_LAB1_Report", "Photos")

FASES = [
    ("1_enderecio",      "Endereço (q = 0)"),
    ("2_meio_backswing", "Meio do backswing"),
    ("3_topo",           "Topo do backswing"),
    ("4_meio_downswing", "Meio do downswing"),
    ("5_impacto",        "Impacto"),
    ("6_follow_through", "Follow-through"),
    ("7_final",          "Pose final"),
]


def instantes(p):
    """Instante de simulação de cada fase [s]."""
    t_imp = t_impacto(p)
    return [
        0.0,                              # endereço
        0.5 * p["t_back"],                # meio do backswing
        p["t_back"],                      # topo
        t_imp - 0.5 * p["t_down"],        # meio do downswing
        t_imp,                            # impacto
        t_imp + 0.15,                     # follow-through
        t_imp + 0.45,                     # pose final (já parado)
    ]


def fotografa(m, d, p, alvos, largura, altura):
    """Corre o swing e devolve {fase: {câmara: imagem}} nos instantes pedidos."""
    _, cfg, qadr, vadr, uadr = _prep(m, p)
    M_full = np.zeros((m.nv, m.nv))
    renderer = mujoco.Renderer(m, altura, largura)
    imagens = {}
    mujoco.mj_resetData(m, d)
    mujoco.mj_forward(m, d)        # sem isto o 1.º fotograma (t = 0) sai vazio
    proxima = 0
    while proxima < len(alvos):
        if d.time >= alvos[proxima]:
            nome, _ = FASES[proxima]
            imagens[nome] = {}
            for cam in ("face_on", "down_the_line"):
                # NOTA: passar o ID da câmara, não o nome. Com o nome em string
                # o update_scene não dá erro mas renderiza uma vista inútil
                # (só o skybox), o que é difícil de diagnosticar.
                renderer.update_scene(d, camera=m.camera(cam).id)
                imagens[nome][cam] = renderer.render().copy()
            proxima += 1
        d.ctrl[uadr] = controlo(m, d, p, qadr, vadr, uadr, M_full)
        mujoco.mj_step(m, d)
    return imagens


def tira(imagens, cam, caminho):
    """Sete fases lado a lado, com o nome da fase por cima (1960 x 322 px)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, len(FASES), figsize=(19.6, 3.22), dpi=100)
    for ax, (nome, _) in zip(axes, FASES):
        ax.imshow(imagens[nome][cam])
        ax.set_title(nome, fontsize=9)
        ax.axis("off")
    fig.tight_layout(pad=0.3)
    fig.savefig(caminho, dpi=100)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description="Imagens do swing")
    ap.add_argument("--modelo", default=MODEL)
    ap.add_argument("--largura", type=int, default=1280)
    ap.add_argument("--altura", type=int, default=960)
    ap.add_argument("--outdir", default=OUTDIR, help="pasta das poses soltas")
    ap.add_argument("--prefixo", default="pose_", help="prefixo das poses soltas")
    ap.add_argument("--strip", action="store_true",
                    help="grava também as tiras <v>_sequencia_<câmara>.png em Photos/")
    args = ap.parse_args()

    m, d = load(args.modelo)
    p, cfg, _, _, _ = _prep(m)
    versao = "v17" if "torso" in cfg["joints"] else "v13"
    alvos = instantes(p)

    imagens = fotografa(m, d, p, alvos, args.largura, args.altura)

    os.makedirs(args.outdir, exist_ok=True)
    for nome, titulo in FASES:
        for cam, img in imagens[nome].items():
            caminho = os.path.join(args.outdir, f"{args.prefixo}{nome}_{cam}.png")
            _guardar(img, caminho)
            print(f"  {titulo:<22} {cam:<15} -> {caminho}")

    if args.strip:
        os.makedirs(PHOTOS, exist_ok=True)
        for cam in ("face_on", "down_the_line"):
            caminho = os.path.join(PHOTOS, f"{versao}_sequencia_{cam}.png")
            tira(imagens, cam, caminho)
            print(f"  tira {cam:<15} -> {caminho}")


def _guardar(img, caminho):
    """Guarda um array RGB como PNG (usa matplotlib, já é dependência)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.image as mpimg
    mpimg.imsave(caminho, img)


if __name__ == "__main__":
    main()
