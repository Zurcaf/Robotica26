"""
Tarefa 2 — O swing no modelo 3D do Lite 6 (MuJoCo), para verificar que não há
contacto perigoso.

O modelo (models/lite6/) é o oficial da UFACTORY, com a pinça. Não é uma
simulação física: em cada instante as juntas são postas nos ângulos da
trajetória e mede-se a distância entre as malhas de colisão de todos os pares
de elos (incluindo o corpo e os dedos da pinça) e entre o robô e a mesa. Isto
completa a verificação de trajectory.py, que só olha para pontos. Serve também
para comparar a nossa cinemática direta com a geometria do modelo.

Uso (a partir da pasta acima de src/):
    python src/mujoco_preview.py --golfista --perto                 # colisões e FK, sem janela
    python src/mujoco_preview.py --golfista --perto --folgas        # folga mínima vs tempo -> report/
    python src/mujoco_preview.py --golfista --perto --fotos --strip --prefixo mjgp_   # imagens das fases
    python src/mujoco_preview.py --golfista --perto --view          # janela interativa, em loop
    python src/mujoco_preview.py                                    # primeira versão (arco cartesiano)

No macOS usar o python normal (não o mjpython).
"""
import argparse
import os

import mujoco
import numpy as np

import config as C
from kinematics import fk
from trajectory import DEFAULTS, GOLFISTA, GOLFISTA_PERTO, joint_trajectory, swing_trajectory

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
    col = m.geom_contype > 0                                # tudo o que colide: elos, corpo e dedos da pinça, mesa
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


def folgas(m, d, traj, qs, prefixo="mj_", alcance_mm=250.0):
    """
    Folga mínima em cada instante, robô-mesa e entre elos não adjacentes, com as
    malhas de colisão do modelo. Gráfico em report/<prefixo>folgas.png.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    col = m.geom_contype > 0                                # elos, corpo e dedos da pinça, mesa
    m.geom_margin[col] = alcance_mm / 1000.0                # lista todos os pares a menos de `alcance`
    mesa, proprio = np.full(len(qs), np.nan), np.full(len(qs), np.nan)
    for k, q in enumerate(qs):
        pose(m, d, q)
        # Com a margem grande, os contactos só dizem que pares estão perto (e podem
        # colidir); a distância exata de cada par vem de mj_geomDistance.
        for g1, g2 in {(c.geom1, c.geom2) for c in d.contact[:d.ncon]}:
            dist = mujoco.mj_geomDistance(m, d, g1, g2, 1.0, None) * 1000
            alvo = mesa if "mesa" in (nome_geom(m, g1), nome_geom(m, g2)) else proprio
            alvo[k] = np.nanmin([alvo[k], dist])
    m.geom_margin[col] = 0.0
    t = traj["t"]
    for nome, f in (("robô-mesa", mesa), ("robô-robô", proprio)):
        k = int(np.nanargmin(f))
        print(f"  folga {nome}: mínimo {f[k]:.1f} mm em t = {t[k]:.2f} s")

    fig, ax = plt.subplots(figsize=(7.5, 3.6))
    ax.plot(t, mesa, color="#2a78d6", lw=2, label="robô (com pinça) — mesa")
    ax.plot(t, proprio, color="#eb6834", lw=2, label="entre elos não adjacentes")
    ax.axhline(0, color="k", lw=1)
    ax.text(t[-1], 2, "contacto", ha="right", va="bottom", fontsize=8, color="#52514e")
    for c in instantes(traj)[2:5:2]:                        # topo do backswing e impacto
        ax.axvline(c, color="#52514e", lw=0.8, ls=":")
    ax.set_ylim(bottom=-5)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("folga mínima [mm]")
    ax.legend(fontsize=8, frameon=False)
    ax.grid(alpha=0.25, lw=0.5)
    ax.spines[["top", "right"]].set_visible(False)
    fig.tight_layout()
    caminho = os.path.join(REPORT, f"{prefixo}folgas.png")
    fig.savefig(caminho, dpi=170)
    print(f"  gráfico -> report/{prefixo}folgas.png")


def desenha_trajetoria(scene, traj, passo=4):
    """Esferas pequenas ao longo do caminho da ponta (geoms só visuais da cena)."""
    for p in traj["pos"][::passo]:
        if scene.ngeom >= scene.maxgeom:
            break
        g = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_SPHERE, np.array([0.003, 0, 0]),
                            p / 1000.0, np.eye(3).flatten(), np.array([0.9, 0.2, 0.1, 0.8], dtype=np.float32))
        scene.ngeom += 1


def fotos(m, d, traj, qs, largura, altura, strip, prefixo="mj_"):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.image as mpimg
    import matplotlib.pyplot as plt

    frames = os.path.join(REPORT, "frames")
    os.makedirs(frames, exist_ok=True)
    renderer = mujoco.Renderer(m, altura, largura)
    imagens = {}
    for (nome, titulo), t_alvo in zip(FASES, instantes(traj)):
        k = int(np.argmin(np.abs(traj["t"] - t_alvo)))
        pose(m, d, qs[k])
        imagens[nome] = {}
        for cam in CAMARAS:
            renderer.update_scene(d, camera=m.camera(cam).id)   # a câmara passa-se pelo id
            desenha_trajetoria(renderer.scene, traj)
            img = renderer.render().copy()
            imagens[nome][cam] = img
            caminho = os.path.join(frames, f"{prefixo}{nome}_{cam}.png")
            mpimg.imsave(caminho, img)
        print(f"  {titulo:<20} t = {traj['t'][k]:5.2f} s  -> report/frames/{prefixo}{nome}_<câmara>.png")
    if strip:
        for cam in CAMARAS:
            fig, axes = plt.subplots(1, len(FASES), figsize=(19.6, 3.4), dpi=100)
            for ax, (nome, titulo) in zip(axes, FASES):
                ax.imshow(imagens[nome][cam])
                ax.set_title(titulo, fontsize=9)
                ax.axis("off")
            fig.tight_layout(pad=0.3)
            caminho = os.path.join(REPORT, f"{prefixo}sequencia_{cam}.png")
            fig.savefig(caminho, dpi=100)
            plt.close(fig)
            print(f"  tira {cam:<14} -> {os.path.relpath(caminho)}")


def view(m, d, traj, qs):
    """Viewer interativo: o swing repete-se em loop, em tempo real."""
    import mujoco.viewer

    t, T = traj["t"], traj["t"][-1] + 1.0          # 1 s parado no fim de cada volta
    m.opt.gravity[:] = 0.0                          # só se impõem ângulos: nada deve cair

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
    ap.add_argument("--k-phi", type=float, default=DEFAULTS["k_phi"], help="0..1, quanto do arco roda a pinça")
    ap.add_argument("--golfista", action="store_true", help="swing desenhado nas juntas (preset GOLFISTA)")
    ap.add_argument("--perto", action="store_true", help="com --golfista: preset GOLFISTA_PERTO, o do relatório")
    ap.add_argument("--enderecio", type=float, nargs=6, metavar="J", default=None,
                    help="com --golfista: pose de endereço [graus] lida no robô com a ponta na bola (J4 e J6 passam a 0)")
    ap.add_argument("--prefixo", default="mj_", help="prefixo dos ficheiros em report/")
    ap.add_argument("--margem", type=float, default=C.TABLE_CLEARANCE,
                    help="mm: avisa de pares de geoms a menos desta distância")
    ap.add_argument("--view", action="store_true")
    ap.add_argument("--fotos", action="store_true")
    ap.add_argument("--strip", action="store_true")
    ap.add_argument("--folgas", action="store_true", help="gráfico da folga mínima (mesa e entre elos) vs t")
    ap.add_argument("--largura", type=int, default=1280)
    ap.add_argument("--altura", type=int, default=960)
    args = ap.parse_args()

    p = dict(phi_end=args.phi_end, phi_top=args.phi_top, raio=args.raio, garra=args.garra, k_phi=args.k_phi,
             t_back=DEFAULTS["t_back"] * args.lento, t_down=args.t_down * args.lento)
    if args.golfista:
        g = GOLFISTA_PERTO if args.perto else GOLFISTA
        if args.enderecio is not None:
            q0 = [args.enderecio[0], args.enderecio[1], args.enderecio[2], 0.0, args.enderecio[4], 0.0]
            g = dict(g, juntas=dict(g["juntas"], q_enderecio=tuple(q0)))
        p = {**p, **g, "t_back": g["t_back"] * args.lento, "t_down": g["t_down"] * args.lento}
    traj = swing_trajectory(p)
    qs, conv = joint_trajectory(traj)
    print(f"  {len(qs)} poses, {traj['t'][-1]:.2f} s, juntas {'obtidas' if conv else 'NÃO obtidas em todos os pontos'}")
    amp = qs.max(axis=0) - qs.min(axis=0)
    print("  amplitude das juntas [deg]: " + "  ".join(f"J{i + 1} {a:.0f}" for i, a in enumerate(amp)))

    m, d = load(args.modelo)
    m.geom("bola").pos[:] = np.asarray(traj["params"]["bola"]) / 1000.0   # a bola da cena segue a trajetória
    verificar(m, d, traj, qs, args.margem)
    if args.folgas:
        folgas(m, d, traj, qs, args.prefixo)
    if args.fotos or args.strip:
        fotos(m, d, traj, qs, args.largura, args.altura, args.strip, args.prefixo)
    if args.view:
        view(m, d, traj, qs)


if __name__ == "__main__":
    main()
