"""
Imagens do swing (renderização offscreen) — Tarefa 1/2 do enunciado.

Guarda em report/ uma imagem por fase do swing, das duas câmaras definidas no
XML (`face_on`, de frente para o jogador, e `down_the_line`, atrás na linha do
alvo — as duas vistas com que se filma um swing).

Uso (a partir de Lab1/):
    python src/renders.py                 # todas as fases, as duas câmaras
    python src/renders.py --largura 1280
"""

import argparse
import os
import numpy as np
import mujoco

from swing import load, reference, _indices, DEFAULTS, MODEL

OUTDIR = "report"

# Fases do swing, como fração do tempo até ao impacto. O instante do impacto é
# t_back + t_pause + t_down; ver reference() em swing.py.
FASES = [
    ("1_enderecio",      "Endereço (q = 0)"),
    ("2_topo",           "Topo do backswing"),
    ("3_meio_downswing", "Meio do downswing"),
    ("4_impacto",        "Impacto"),
    ("5_follow_through", "Follow-through"),
]


def instantes(p):
    """Instante de simulação de cada fase [s]."""
    t_top = p["t_back"]
    t_imp = p["t_back"] + p["t_pause"] + p["t_down"]
    return [
        0.0,                                  # endereço
        t_top,                                # topo
        t_top + p["t_pause"] + p["t_down"] / 2,   # meio do downswing
        t_imp,                                # impacto
        t_imp + 0.55 * p["t_down"],           # follow-through
    ]


def main():
    ap = argparse.ArgumentParser(description="Imagens do swing")
    ap.add_argument("--largura", type=int, default=1280)
    ap.add_argument("--altura", type=int, default=960)
    args = ap.parse_args()

    p = dict(DEFAULTS)
    m, d = load()
    qadr, vadr, uadr = _indices(m)
    p["q_range"] = m.jnt_range[[m.joint("shoulder").id, m.joint("wrist").id]]
    lo, hi = m.actuator_ctrlrange[uadr, 0], m.actuator_ctrlrange[uadr, 1]
    M_full = np.zeros((m.nv, m.nv))

    os.makedirs(OUTDIR, exist_ok=True)
    alvos = instantes(p)
    renderer = mujoco.Renderer(m, args.altura, args.largura)

    # Corre o swing uma vez e fotografa quando passa por cada instante-alvo.
    mujoco.mj_resetData(m, d)
    proxima = 0
    nsteps = int((alvos[-1] + 0.05) / m.opt.timestep)
    for _ in range(nsteps):
        if proxima < len(alvos) and d.time >= alvos[proxima]:
            nome, titulo = FASES[proxima]
            for cam in ("face_on", "down_the_line"):
                # NOTA: passar o ID da câmara, não o nome. Com o nome em string
                # o update_scene não dá erro mas renderiza uma vista inútil
                # (só o skybox), o que é difícil de diagnosticar.
                renderer.update_scene(d, camera=m.camera(cam).id)
                img = renderer.render()
                caminho = os.path.join(OUTDIR, f"pose_{nome}_{cam}.png")
                _guardar(img, caminho)
                print(f"  {titulo:<22} {cam:<15} -> {caminho}")
            proxima += 1

        q_ref, v_ref, a_ref = reference(d.time, p)
        mujoco.mj_fullM(m, d, M_full)
        M2 = M_full[np.ix_(vadr, vadr)]
        a_cmd = (a_ref + p["kd"] * (v_ref - d.qvel[vadr])
                 + p["kp"] * (q_ref - d.qpos[qadr]))
        d.ctrl[uadr] = np.clip(M2 @ a_cmd + d.qfrc_bias[vadr], lo, hi)
        mujoco.mj_step(m, d)


def _guardar(img, caminho):
    """Guarda um array RGB como PNG (usa matplotlib, já é dependência)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.image as mpimg
    mpimg.imsave(caminho, img)


if __name__ == "__main__":
    main()
