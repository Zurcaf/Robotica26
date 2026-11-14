"""Verifica os ensaios do robô antes de os correr (como no Lab 2: simular primeiro).

    .venv/bin/python src/robo/verificar.py                  # todos: tabela de verificação
    .venv/bin/python src/robo/verificar.py inclinacoes --ver   # janela MuJoCo, em loop
    .venv/bin/python src/robo/verificar.py --fotos          # imagens em resultados/robo/
    .venv/bin/python src/robo/verificar.py --sim            # dados de IMU simulados

Para cada ensaio: limites das juntas (com 5° de folga), velocidade e
aceleração das juntas (até 80 % do máximo), velocidade do flange (até
400 mm/s), folgas no MuJoCo com o suporte e o kit (robô-mesa e robô-robô) e
altura mínima da IMU. O plano fica em data/planos/<ensaio>.npz, que é o que
correr.py envia.

--sim gera data/sim/robo_<ensaio>.csv (+ _verdade.npz) com o simulador da
IMU, para testar a análise com a verdade do robô antes de ir ao laboratório.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI))
sys.path.insert(0, str(AQUI.parent))
import ensaios as E          # noqa: E402
import geometria as G        # noqa: E402
from imu_io import DADOS, LAB3   # noqa: E402

PLANOS = DADOS / "planos"
FIGS = LAB3 / "resultados" / "robo"
MARGEM_JUNTA = 5.0
FOLGA_MIN = 20.0           # mm, abaixo disto o ensaio não passa
V_JUNTA = 0.8 * G.C2.JOINT_SPEED_MAX
A_JUNTA = 0.8 * G.C2.JOINT_ACC_MAX
V_FLANGE = 0.8 * G.C2.TCP_SPEED_MAX


def verificar(plano, m, d, verbose=True):
    t, q = plano["t"], plano["q"]
    lim = np.array(G.C2.JOINT_LIMITS)
    folga_lim = np.minimum(q - lim[:, 0], lim[:, 1] - q).min()
    v = np.gradient(q, t, axis=0)
    a = np.gradient(v, t, axis=0)
    p_f = np.array([G.fk(qk)[:3, 3] for qk in q])
    v_f = np.linalg.norm(np.gradient(p_f, t, axis=0), axis=1)
    p_imu = np.array([G.fk_imu(qk)[1] for qk in q])
    mesa, proprio, pares = G.folgas(m, d, q)
    r = {
        "dur": t[-1], "folga_lim": folga_lim, "v_max": np.abs(v).max(), "a_max": np.abs(a).max(),
        "v_flange": v_f.max(), "mesa": np.nanmin(mesa), "proprio": np.nanmin(proprio),
        "imu_z": p_imu[:, 2].min(),
    }
    r["ok"] = (folga_lim >= MARGEM_JUNTA and r["v_max"] <= V_JUNTA and r["a_max"] <= A_JUNTA
               and r["v_flange"] <= V_FLANGE and r["mesa"] >= FOLGA_MIN and r["proprio"] >= FOLGA_MIN)
    r["pares"] = sorted(pares.items(), key=lambda kv: kv[1][0])[:3]
    return r


def imu_simulada(plano, nome):
    """Leituras simuladas da IMU ao longo do plano (verdade = cinemática)."""
    from scipy.interpolate import CubicSpline
    from scipy.spatial.transform import Rotation as Rot
    import simular as S
    from imu_io import escrever
    t, q = plano["t"], plano["q"]
    ti = np.arange(0, t[-1], 1 / S.FS_INT)
    qi = CubicSpline(t, q, axis=0)(ti)
    T = [G.fk(qk) @ G.T_F_IMU() for qk in qi]
    R = Rot.from_matrix(np.array([Tk[:3, :3] for Tk in T]))
    p = np.array([Tk[:3, 3] for Tk in T]) / 1000.0
    w = np.vstack([(R[:-1].inv() * R[1:]).as_rotvec() * S.FS_INT, np.zeros((1, 3))])
    par = S.ParametrosIMU()
    t_o, acc, gyro, Tc, R_o, p_o, w_o = S.simular(ti, R, w, p, par)
    csv = escrever(DADOS / "sim" / f"robo_{nome}.csv", t_o, acc, gyro, Tc, S.META,
                   comentarios=["simulado a partir do plano do robô (src/robo/verificar.py)", f"cenario=robo_{nome}"])
    import json
    from dataclasses import asdict
    np.savez(DADOS / "sim" / f"robo_{nome}_verdade.npz", t=t_o, q=R_o.as_quat(), p=p_o, w_dps=w_o,
             parametros=json.dumps(asdict(par)))
    return csv


def fotos(plano, m, d, n=6):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import mujoco
    FIGS.mkdir(parents=True, exist_ok=True)
    rend = mujoco.Renderer(m, 360, 480)
    et = [e for e in plano["etiquetas"] if not e[2].startswith("→") or len(plano["etiquetas"]) < 12]
    escolha = np.linspace(0, len(et) - 1, min(n, len(et))).round().astype(int)
    fig, axs = plt.subplots(1, len(escolha), figsize=(3.2 * len(escolha), 2.9))
    for ax, i in zip(np.atleast_1d(axs), escolha):
        t0, t1, txt = et[i]
        k = int(round((t0 + t1) / 2 / E.DT))
        G.pose_mj(m, d, plano["q"][k])
        cam = mujoco.MjvCamera()
        cam.lookat[:] = d.site("imu").xpos
        cam.lookat[2] *= 0.6
        cam.distance, cam.azimuth, cam.elevation = 0.95, 135, -18
        rend.update_scene(d, camera=cam)
        ax.imshow(rend.render())
        ax.set_title(f"{txt}\nt = {(t0 + t1) / 2:.0f} s", fontsize=8)
        ax.axis("off")
    fig.suptitle(plano["nome"], fontsize=10)
    fig.tight_layout()
    out = FIGS / f"{plano['nome']}.png"
    fig.savefig(out, dpi=110)
    plt.close(fig)
    return out


def ver(plano, m, d):
    import mujoco
    import mujoco.viewer
    t, q = plano["t"], plano["q"]
    T = t[-1] + 1.0
    m.opt.gravity[:] = 0.0

    def replay(model, data):
        k = int(np.searchsorted(t, data.time % T, side="right")) - 1
        data.qpos[:6] = np.radians(q[min(max(k, 0), len(q) - 1)])
        data.qvel[:] = 0.0

    mujoco.set_mjcb_control(replay)
    try:
        G.pose_mj(m, d, q[0])
        mujoco.viewer.launch(m, d)
    finally:
        mujoco.set_mjcb_control(None)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ensaios", nargs="*", help=f"um ou mais de: {', '.join(E.ENSAIOS)}")
    ap.add_argument("--ver", action="store_true", help="abre o MuJoCo (só o primeiro ensaio)")
    ap.add_argument("--fotos", action="store_true")
    ap.add_argument("--sim", action="store_true")
    a = ap.parse_args()
    nomes = a.ensaios or list(E.ENSAIOS)
    m, d = G.modelo()
    PLANOS.mkdir(parents=True, exist_ok=True)

    print(f"{'ensaio':12s} {'dur':>6s} {'f.lim':>6s} {'v jt':>6s} {'a jt':>6s} {'v fl':>6s} "
          f"{'mesa':>6s} {'robô':>6s} {'IMU z':>6s}")
    print(f"{'':12s} {'s':>6s} {'°':>6s} {'°/s':>6s} {'°/s²':>6s} {'mm/s':>6s} {'mm':>6s} {'mm':>6s} {'mm':>6s}")
    todos_ok = True
    for nome in nomes:
        plano = E.ENSAIOS[nome]()
        r = verificar(plano, m, d)
        todos_ok &= r["ok"]
        np.savez(PLANOS / f"{nome}.npz", t=plano["t"], q=plano["q"],
                 etiquetas=np.array([f"{a:.2f}|{b:.2f}|{c}" for a, b, c in plano["etiquetas"]]),
                 R_F_IMU=G.R_F_IMU, P_F_IMU=G.P_F_IMU)
        print(f"{nome:12s} {r['dur']:6.0f} {r['folga_lim']:6.1f} {r['v_max']:6.0f} {r['a_max']:6.0f} "
              f"{r['v_flange']:6.0f} {r['mesa']:6.0f} {r['proprio']:6.0f} {r['imu_z']:6.0f}  "
              f"{'OK' if r['ok'] else 'FALHA'}")
        if not r["ok"] or nome == nomes[0]:
            for (g1, g2), (dist, k) in r["pares"]:
                print(f"{'':14s}par mais perto: {g1} – {g2}: {dist:.0f} mm (t = {k * E.DT:.1f} s)")
        if plano.get("falhas"):
            print(f"{'':14s}orientações sem solução (ficam de fora): {'; '.join(plano['falhas'])}")
        if a.fotos:
            print(f"{'':14s}imagens: {fotos(plano, m, d)}")
        if a.sim:
            print(f"{'':14s}IMU simulada: {imu_simulada(plano, nome)}")
        if a.ver:
            ver(plano, m, d)
            break
    print(f"\nLimites usados: folga às juntas ≥ {MARGEM_JUNTA:.0f}°, juntas ≤ {V_JUNTA:.0f} °/s e "
          f"{A_JUNTA:.0f} °/s², flange ≤ {V_FLANGE:.0f} mm/s, folgas MuJoCo ≥ {FOLGA_MIN:.0f} mm.")
    print("Tudo OK." if todos_ok else "HÁ ENSAIOS QUE NÃO PASSAM: não os correr no robô.")


if __name__ == "__main__":
    main()
