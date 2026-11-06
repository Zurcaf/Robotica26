"""
Pré-visualização do swing da Tarefa 2 em MuJoCo, com o modelo oficial do Lite 6
(models/lite6/, derivado do URDF da UFactory) e a pinça Gripper Lite.

NÃO é uma simulação física: as juntas são postas diretamente nos ângulos que a
IK numérica de trajectory.py dá para cada instante (replay cinemático). Serve
para:
  1. ver o braço inteiro, com volume, a fazer o swing (viewer ou imagens);
  2. verificar colisões com os elos a sério (mesa, base, elo contra elo), o que
     check_feasibility() só faz com pontos — responde ao "no dangerous physical
     contact" do enunciado;
  3. conferir a nossa FK (tabela DH do PDF) contra a geometria do URDF.

Uso (a partir de Lab2/):
    python src/mujoco_preview.py                 # verifica colisões e FK, não abre janela
    python src/mujoco_preview.py --view          # viewer interativo, swing em loop
    python src/mujoco_preview.py --fotos         # 7 fases x 3 câmaras -> report/mj_*.png
    python src/mujoco_preview.py --fotos --strip # + tiras report/mj_sequencia_<câmara>.png
    python src/mujoco_preview.py --lento 2 --phi-end 50 --view

Os parâmetros da trajetória são os de trajectory.DEFAULTS (ver --help).
No macOS usar o python normal (não mjpython): o viewer corre com launch() e um
callback, como no Lab 1.
"""
import argparse
import os

import mujoco
import numpy as np

import config as C
from kinematics import fk
from trajectory import DEFAULTS, joint_trajectory, swing_trajectory

HERE = os.path.dirname(os.path.abspath(__file__))
MODEL = os.path.join(HERE, "..", "models", "lite6", "swing_scene.xml")
REPORT = os.path.join(HERE, "..", "report")
CAMARAS = ("face_on", "down_the_line", "cima")
FASES = [
    ("1_enderecio", "Endereço"),
    ("2_meio_backswing", "Meio do backswing"),
    ("3_topo", "Topo do backswing"),
    ("4_meio_downswing", "Meio do downswing"),
    ("5_impacto", "Impacto"),
    ("6_follow_through", "Follow-through"),
    ("7_final", "Pose final"),
]


def instantes(traj):
    """Instante de cada fase [s], a partir dos parâmetros da trajetória."""
    p = traj["params"]
    tb, td, tf = p["t_back"], p["t_down"], traj["t"][-1]
    t_imp = tb + td
    return [0.0, 0.5 * tb, tb, tb + 0.5 * td, t_imp, t_imp + 0.5 * (tf - t_imp), tf]


def load(caminho=MODEL):
    m = mujoco.MjModel.from_xml_path(os.path.abspath(caminho))
    d = mujoco.MjData(m)
    return m, d


def pose(m, d, q_deg):
    """Põe as 6 juntas em q (graus) e atualiza a cinemática (sem física)."""
    d.qpos[:6] = np.radians(q_deg)
    d.qvel[:] = 0.0
    mujoco.mj_forward(m, d)


def nome_geom(m, gid):
    return mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, gid) or f"geom{gid}"


def verificar(m, d, traj, qs, margem_mm):
    """
    Percorre a trajetória e devolve (ok, folga_min_mm, pares) com os contactos
    encontrados. Com margem > 0 o MuJoCo também lista pares a menos de `margem`
    (dist > 0 = só perto; dist <= 0 = penetração = colisão).
    """
    col = m.geom_group == 3                                 # geoms de colisão do modelo
    m.geom_margin[col] = margem_mm / 1000.0
    pares, folga = {}, np.inf
    e_flange = e_ponta = 0.0
    for k, q in enumerate(qs):
        pose(m, d, q)
        e_flange = max(e_flange, np.linalg.norm(d.site("attachment_site").xpos * 1000 - traj["pos_flange"][k]))
        e_ponta = max(e_ponta, np.linalg.norm(d.site("end_effector").xpos * 1000 - traj["pos"][k]))
        for c in d.contact[:d.ncon]:
            par = tuple(sorted((nome_geom(m, c.geom1), nome_geom(m, c.geom2))))
            dist = c.dist * 1000
            folga = min(folga, dist)
            if par not in pares or dist < pares[par][0]:
                pares[par] = (dist, traj["t"][k])
    m.geom_margin[col] = 0.0
    ok = folga > 0
    print(f"  FK: |MuJoCo - nossa| máx. flange {e_flange:.2f} mm, ponta {e_ponta:.2f} mm "
          f"(URDF: 243.5/200.2/227.61/62.5 mm e pinça 81.1 mm; PDF: 243.3/200/227.6/61.5 e {traj['params']['garra']} mm)")
    if not pares:
        print(f"  colisões: nenhum par a menos de {margem_mm:.0f} mm em {len(qs)} poses  OK")
    else:
        print(f"  pares a menos de {margem_mm:.0f} mm (dist <= 0 é colisão; as malhas de colisão são "
              "invólucros convexos, logo as distâncias são conservadoras):")
        for (a, b), (dist, t) in sorted(pares.items(), key=lambda kv: kv[1][0]):
            print(f"    {a:<24} {b:<24} dist {dist:7.1f} mm  (t = {t:.2f} s)  {'COLISÃO' if dist <= 0 else 'perto'}")
        print(f"  => {'sem colisão' if ok else 'COLISÃO'} (menor distância {folga:.1f} mm)")
    return ok, folga, pares


def desenha_trajetoria(scene, traj, passo=4):
    """Esferas pequenas ao longo do caminho da ponta (geoms só visuais da cena)."""
    for p in traj["pos"][::passo]:
        if scene.ngeom >= scene.maxgeom:
            break
        g = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.003, 0, 0]),
                            p / 1000.0, np.eye(3).flatten(), np.array([0.9, 0.2, 0.1, 0.8], dtype=np.float32))
        scene.ngeom += 1


def fotos(m, d, traj, qs, largura, altura, strip):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.image as mpimg
    import matplotlib.pyplot as plt

    os.makedirs(REPORT, exist_ok=True)
    renderer = mujoco.Renderer(m, altura, largura)
    imagens = {}
    for (nome, titulo), t_alvo in zip(FASES, instantes(traj)):
        k = int(np.argmin(np.abs(traj["t"] - t_alvo)))
        pose(m, d, qs[k])
        imagens[nome] = {}
        for cam in CAMARAS:
            renderer.update_scene(d, camera=m.camera(cam).id)   # ID, não nome (ver Lab 1)
            desenha_trajetoria(renderer.scene, traj)
            img = renderer.render().copy()
            imagens[nome][cam] = img
            caminho = os.path.join(REPORT, f"mj_{nome}_{cam}.png")
            mpimg.imsave(caminho, img)
        print(f"  {titulo:<20} t = {traj['t'][k]:5.2f} s  -> report/mj_{nome}_<câmara>.png")
    if strip:
        for cam in CAMARAS:
            fig, axes = plt.subplots(1, len(FASES), figsize=(19.6, 3.4), dpi=100)
            for ax, (nome, titulo) in zip(axes, FASES):
                ax.imshow(imagens[nome][cam])
                ax.set_title(titulo, fontsize=9)
                ax.axis("off")
            fig.tight_layout(pad=0.3)
            caminho = os.path.join(REPORT, f"mj_sequencia_{cam}.png")
            fig.savefig(caminho, dpi=100)
            plt.close(fig)
            print(f"  tira {cam:<14} -> {os.path.relpath(caminho)}")


def view(m, d, traj, qs):
    """Viewer interativo: o swing repete-se em loop, em tempo real."""
    import mujoco.viewer

    t, T = traj["t"], traj["t"][-1] + 1.0          # 1 s parado no fim de cada volta
    m.opt.gravity[:] = 0.0                          # replay cinemático: nada deve cair

    def replay(model, data):
        k = int(np.searchsorted(t, data.time % T, side="right")) - 1
        data.qpos[:6] = np.radians(qs[min(max(k, 0), len(qs) - 1)])
        data.qvel[:] = 0.0

    mujoco.set_mjcb_control(replay)
    try:
        pose(m, d, qs[0])
        mujoco.viewer.launch(m, d)
    finally:
        mujoco.set_mjcb_control(None)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--modelo", default=MODEL)
    ap.add_argument("--lento", type=float, default=1.0, help="fator de abrandamento (como run_robot.py)")
    ap.add_argument("--phi-end", type=float, default=DEFAULTS["phi_end"])
    ap.add_argument("--phi-top", type=float, default=DEFAULTS["phi_top"])
    ap.add_argument("--t-down", type=float, default=DEFAULTS["t_down"])
    ap.add_argument("--raio", type=float, default=DEFAULTS["raio"])
    ap.add_argument("--garra", type=float, default=DEFAULTS["garra"])
    ap.add_argument("--margem", type=float, default=C.TABLE_CLEARANCE,
                    help="mm: avisa de pares de geoms a menos desta distância")
    ap.add_argument("--view", action="store_true")
    ap.add_argument("--fotos", action="store_true")
    ap.add_argument("--strip", action="store_true")
    ap.add_argument("--largura", type=int, default=1280)
    ap.add_argument("--altura", type=int, default=960)
    args = ap.parse_args()

    p = dict(phi_end=args.phi_end, phi_top=args.phi_top, raio=args.raio, garra=args.garra,
             t_back=DEFAULTS["t_back"] * args.lento, t_down=args.t_down * args.lento)
    traj = swing_trajectory(p)
    qs, conv = joint_trajectory(traj)
    print(f"  {len(qs)} poses, {traj['t'][-1]:.2f} s, IK numérica {'convergiu' if conv else 'NÃO convergiu em todos os pontos'}")

    m, d = load(args.modelo)
    verificar(m, d, traj, qs, args.margem)
    if args.fotos or args.strip:
        fotos(m, d, traj, qs, args.largura, args.altura, args.strip)
    if args.view:
        view(m, d, traj, qs)


if __name__ == "__main__":
    main()
