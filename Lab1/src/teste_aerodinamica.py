"""
Teste da aerodinâmica — valida o modelo de ar do golfer_v13.xml.

Responde a três perguntas, cada uma com números que se podem pôr no relatório:

  1. A BOLA: que Cd, CL e decaimento da rotação é que o MuJoCo aplica de facto?
     (voo livre, sem gravidade, nas condições de lançamento do swing)
     O MuJoCo não usa o 1/2 de F = 1/2 rho Cd A v^2, por isso os valores do
     fluidcoef NÃO são o Cd — é preciso medi-los.

  2. O BRAÇO E O TACO: o arrasto que o modelo lhes aplica é plausível?
     Compara três modelos do ar no corpo:
       vácuo          — sem ar nenhum (a versão anterior do modelo)
       caixa inércia  — o que o MuJoCo faz por omissão quando há density mas a
                        geom não declara fluidshape
       elipsoide      — fluidshape="ellipsoid" em cada geom (o que o XML usa)
     e confronta com uma estimativa à mão do arrasto da cabeça + haste.

  3. O CONTROLADOR: se ele souber o arrasto e o compensar, recupera a velocidade?

Uso (a partir de Lab1/):
    python src/teste_aerodinamica.py

Produz report/aerodinamica.png e imprime as tabelas.
"""

import os
import numpy as np
import mujoco

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from swing import load, run_swing

OUT = os.path.join("report", "aerodinamica.png")
RHO = 1.204          # densidade do ar [kg/m3], a mesma do XML


# ----------------------------------------------------------------------------
# 1. Bola em voo livre
# ----------------------------------------------------------------------------
BOLA_XML = """
<mujoco>
  <option gravity="0 0 {g}" timestep="0.0005" density="{rho}" viscosity="{mu}"/>
  <worldbody>
    <body><freejoint/>
      <geom type="sphere" size="{r}" mass="{mass}" fluidshape="ellipsoid"
            fluidcoef="{coef}"/>
    </body>
  </worldbody>
</mujoco>"""


def bola_modelo(m_golf, coef=None, g=0.0, ar=True):
    """Modelo só com a bola, com os mesmos parâmetros do golfer_v13.xml."""
    gb = m_golf.geom("ball_geom").id
    if coef is None:
        coef = m_golf.geom_fluid[gb][1:6]
    xml = BOLA_XML.format(
        g=g, rho=m_golf.opt.density if ar else 0.0,
        mu=m_golf.opt.viscosity if ar else 0.0,
        r=m_golf.geom_size[gb][0], mass=m_golf.body_mass[m_golf.geom_bodyid[gb]],
        coef=" ".join(f"{c:g}" for c in coef))
    m = mujoco.MjModel.from_xml_string(xml)
    return m, mujoco.MjData(m)


def coeficientes_efetivos(m_golf, v0, spin_rpm):
    """Cd, CL e decaimento da rotação que o MuJoCo aplica à bola."""
    m, d = bola_modelo(m_golf)
    R = m.geom_size[0][0]
    A = np.pi * R**2
    w0 = spin_rpm * 2 * np.pi / 60
    # bola a voar em +x com backspin (rotação em torno de -y => sustentação +z)
    d.qvel[:3] = [v0, 0, 0]
    d.qvel[3:] = [0, -w0, 0]
    mujoco.mj_forward(m, d)
    f = d.qfrc_fluid[:3]
    q = 0.5 * RHO * A * v0**2
    cd, cl = -f[0] / q, f[2] / q
    for _ in range(int(1.0 / m.opt.timestep)):
        mujoco.mj_step(m, d)
    decai = 1.0 - np.linalg.norm(d.qvel[3:]) / w0
    return cd, cl, decai, R * w0 / v0


def trajetoria(m_golf, v0, launch_deg, spin_rpm, coef=None, ar=True):
    """Trajetória (x, z) da bola até aterrar, com gravidade."""
    m, d = bola_modelo(m_golf, coef=coef, g=-9.81, ar=ar)
    a = np.radians(launch_deg)
    w0 = spin_rpm * 2 * np.pi / 60
    d.qpos[2] = 0.0
    d.qvel[:3] = [v0 * np.cos(a), 0, v0 * np.sin(a)]
    d.qvel[3:] = [0, -w0, 0]
    xs, zs = [0.0], [0.0]
    while True:
        mujoco.mj_step(m, d)
        xs.append(d.qpos[0]); zs.append(d.qpos[2])
        if d.qpos[2] < 0 or d.time > 20:
            break
    return np.array(xs), np.array(zs)


# ----------------------------------------------------------------------------
# 2. Ar no braço e no taco: três modelos
# ----------------------------------------------------------------------------
def configura(m, modelo, ref_fluid):
    """Põe o modelo de ar pedido em todas as geoms MENOS a bola."""
    gb = m.geom("ball_geom").id
    m.opt.density = RHO if modelo != "vácuo" else 0.0
    m.opt.viscosity = 1.81e-5 if modelo != "vácuo" else 0.0
    for g in range(m.ngeom):
        if g == gb:
            continue
        m.geom_fluid[g] = ref_fluid[g]
        if modelo == "caixa inércia":
            # geom_fluid[0] = 0 => o MuJoCo usa a caixa de inércia do corpo
            m.geom_fluid[g, 0] = 0.0


def estimativa_mao(m, v_head):
    """
    Arrasto da cabeça + haste estimado à mão, binário em torno do punho.
    Corpos rombos (placa/cilindro perpendicular ao escoamento): Cd ~ 1.0-1.2
    (Hoerner, Fluid-Dynamic Drag, 1965). A velocidade de cada ponto da haste
    cresce linearmente com a distância ao punho (rotação pura).
    """
    gh, gs = m.geom("clubhead").id, m.geom("shaft").id
    sx, sy, sz = 2 * m.geom_size[gh]          # dimensões totais da caixa
    A_head = sorted([sx * sy, sx * sz, sy * sz])[1]   # face "média" de frente
    L = np.linalg.norm(m.geom_pos[gh])        # punho -> cabeça
    d_shaft = 2 * m.geom_size[gs][0]
    Cd = 1.1
    F_head = 0.5 * RHO * Cd * A_head * v_head**2
    # haste: dF = 1/2 rho Cd d (v r/L)^2 dr ;  binário = integral r dF
    T_shaft = 0.5 * RHO * Cd * d_shaft * v_head**2 * L**2 / 4
    return F_head * L + T_shaft, F_head, A_head, L


# ----------------------------------------------------------------------------
def main():
    m, d = load()
    ref_fluid = m.geom_fluid.copy()

    # ---- 2+3: swing completo nos três modelos, com e sem compensação
    print("\n2) AR NO BRAÇO E NO TACO — swing completo (no vácuo a bola também não tem ar;"
          "\n   nos outros dois a bola tem sempre os coeficientes do XML)\n")
    print(f"  {'modelo':<15}{'controlador':<14}{'v_taco':>7}{'v_bola':>7}"
          f"{'q impacto [deg]':>18}{'carry':>8}"
          f"{'binário ar máx':>22}{'saturação':>14}")
    print(f"  {'':<15}{'':<14}{'[m/s]':>7}{'[m/s]':>7}{'(braço, pulso)':>18}{'[m]':>8}"
          f"{'braço / pulso [N.m]':>22}{'braço / pulso':>14}")
    res = {}
    for modelo in ["vácuo", "caixa inércia", "elipsoide"]:
        for comp in [False, True]:
            if modelo == "vácuo" and comp:
                continue
            configura(m, modelo, ref_fluid)
            r = run_swing(m, d, {"comp_fluid": comp})
            res[(modelo, comp)] = r
            fm, sat = r["fluid_max"], r["sat"] * 100
            print(f"  {modelo:<15}{'compensa ar' if comp else 'normal':<14}"
                  f"{r['v_head']:7.1f}{r['v_ball']:7.1f}"
                  f"{'(%+.1f, %+.1f)' % tuple(r['q_impact']):>18}{r['carry']:8.1f}"
                  f"{'%5.1f / %4.1f' % tuple(fm):>22}"
                  f"{'%3.0f %% / %3.0f %%' % tuple(sat):>14}")
    configura(m, "elipsoide", ref_fluid)            # repõe o modelo do XML

    nom = res[("elipsoide", False)]
    T_mao, F_head, A_head, L = estimativa_mao(m, res[("vácuo", False)]["v_head"])
    print(f"\n  Estimativa à mão (Cd = 1.1, cabeça {A_head*1e4:.0f} cm2 a {L:.2f} m do punho,"
          f" v = {res[('vácuo', False)]['v_head']:.1f} m/s):")
    print(f"    força na cabeça {F_head:.1f} N; binário cabeça + haste no punho "
          f"~ {T_mao:.1f} N.m")
    print(f"    elipsoide: {res[('elipsoide', False)]['fluid_max'][1]:.1f} N.m   "
          f"caixa de inércia: {res[('caixa inércia', False)]['fluid_max'][1]:.1f} N.m")

    # ---- 1: a bola, nas condições de lançamento do swing nominal
    cd, cl, decai, S = coeficientes_efetivos(m, nom["v_ball"], nom["spin_rpm"])
    gb = m.geom("ball_geom").id
    print(f"\n1) BOLA EM VOO LIVRE  (fluidcoef = {' '.join(f'{c:g}' for c in m.geom_fluid[gb][1:6])})\n")
    print(f"  lançamento: v = {nom['v_ball']:.1f} m/s, spin = {nom['spin_rpm']:.0f} rpm, "
          f"S = r.w/v = {S:.3f}")
    print(f"  Cd efetivo        {cd:6.3f}     literatura ~0.22-0.30")
    print(f"  CL efetivo        {cl:6.3f}     literatura ~0.15-0.25 (S ~ 0.1-0.2)")
    print(f"  rotação perdida   {decai*100:5.1f} %/s   literatura: poucos %/s")

    # trajetórias com as condições de lançamento do swing nominal
    coef = m.geom_fluid[gb][1:6].copy()
    sem_magnus = coef.copy(); sem_magnus[4] = 0.5   # 0.5 = sustentação nula (ver XML)
    traj = {
        "vácuo": trajetoria(m, nom["v_ball"], nom["launch"], nom["spin_rpm"], ar=False),
        "só arrasto": trajetoria(m, nom["v_ball"], nom["launch"], nom["spin_rpm"], coef=sem_magnus),
        "arrasto + Magnus": trajetoria(m, nom["v_ball"], nom["launch"], nom["spin_rpm"]),
    }
    print("\n  carry da bola sozinha, mesmas condições de lançamento:")
    for k, (x, z) in traj.items():
        print(f"    {k:<18} {x[-1]:6.1f} m   (altura máx {z.max():.1f} m)")

    # ---- gráfico
    cores = {"vácuo": "#2a78d6", "caixa inércia": "#eb6834", "elipsoide": "#1baf7a"}
    tinta, tinta2 = "#0b0b0b", "#52514e"
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 4.6),
                                   gridspec_kw={"width_ratios": [1.6, 1]})
    for (k, (x, z)), c in zip(traj.items(), ["#2a78d6", "#eda100", "#1baf7a"]):
        ax1.plot(x, z, lw=2, color=c, label=k)
        ax1.annotate(f"{x[-1]:.0f} m", (x[-1], 0), xytext=(0, 6),
                     textcoords="offset points", ha="center", color=tinta, fontsize=9)
    ax1.set_xlabel("distância [m]", color=tinta2)
    ax1.set_ylabel("altura [m]", color=tinta2)
    ax1.set_title(f"Voo da bola  (v = {nom['v_ball']:.1f} m/s, "
                  f"{nom['launch']:.1f}°, {nom['spin_rpm']:.0f} rpm)", color=tinta)
    ax1.set_ylim(bottom=0)
    ax1.legend(frameon=False)

    nomes = ["vácuo", "caixa inércia", "caixa inércia\n+ compensa", "elipsoide",
             "elipsoide\n+ compensa"]
    chaves = [("vácuo", False), ("caixa inércia", False), ("caixa inércia", True),
              ("elipsoide", False), ("elipsoide", True)]
    vals = [res[k]["v_head"] for k in chaves]
    cs = [cores[k[0]] for k in chaves]
    bars = ax2.bar(range(len(vals)), vals, color=cs, width=0.62,
                   edgecolor="#fcfcfb", linewidth=2)
    for i, (b, v) in enumerate(zip(bars, vals)):
        ax2.text(b.get_x() + b.get_width() / 2, v + 0.2, f"{v:.1f}",
                 ha="center", va="bottom", color=tinta, fontsize=9)
    ax2.set_xticks(range(len(vals)), nomes, fontsize=8, color=tinta2)
    ax2.set_ylim(0, 41)
    ax2.set_ylabel("velocidade da cabeça do taco [m/s]", color=tinta2)
    ax2.set_title("Ar no braço e no taco", color=tinta)
    for ax in (ax1, ax2):
        ax.grid(alpha=0.25, axis="y")
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
    fig.tight_layout()
    os.makedirs("report", exist_ok=True)
    fig.savefig(OUT, dpi=130, facecolor="#fcfcfb")
    print(f"\n  gráfico guardado em {OUT}")


if __name__ == "__main__":
    main()
