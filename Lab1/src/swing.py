"""
Swing do golfista V13 — 2 dof atuados (tronco + braços/taco).

Uso (a partir de Lab1/):
    python src/swing.py                 # corre um swing e imprime os resultados
    python src/swing.py --view          # abre o viewer com o swing a correr
    python src/swing.py --t-down 0.28   # muda a duração do downswing

IDEIA DO CONTROLO
-----------------
O problema central com 2 dof é que a cabeça do taco só passa na bola quando
AS DUAS juntas estão em q = 0 ao mesmo tempo (é assim que a pose de endereço
está definida no XML). Se uma junta chega antes da outra, o arco passa ao lado
ou por cima da bola, e a face chega com a orientação errada — foi o que fez o
primeiro teste falhar (impacto a torso=+17°, shoulder=-12° => a bola era
enterrada no chão em vez de levantar voo).

Resolve-se em duas partes:

1) TRAJETÓRIA que cruza zero em simultâneo, por construção.
   No downswing usa-se, para cada junta i, aceleração angular constante:

       q_i(u) = q_topo_i · (1 - (u/T)^2),    u = tempo desde o topo

   Com o MESMO T nas duas juntas, ambas passam por q = 0 exatamente em
   u = T (o instante do impacto), e nesse ponto a velocidade é máxima
   (-2·q_topo_i/T) — que é o que se quer num swing.

   Aceleração constante é também a forma mais eficiente de gastar um motor
   com binário limitado: o binário fica no máximo durante todo o downswing.
   Para o mesmo pico de binário, esta parábola dá +27 % de velocidade no
   impacto face a um perfil em cosseno.

2) CONTROLO POR BINÁRIO CALCULADO (computed torque), para que a trajetória
   real siga a de referência sem atraso:

       tau = M(q)·[ a_ref + Kd·(v_ref - v) + Kp·(q_ref - q) ] + h(q, v)

   onde M é a matriz de massa e h = qfrc_bias reúne Coriolis/centrífugas e
   gravidade. É o controlador clássico de manipuladores: a parte M·a_ref + h
   é o feedforward que "sabe" a dinâmica, e o PD só corrige o erro residual.
   Os binários são sempre saturados no ctrlrange do XML (±150 / ±100 N·m),
   por isso o modelo nunca usa força que um humano não conseguisse fazer.
"""

import argparse
import numpy as np
import mujoco
import mujoco.viewer

MODEL = "models/golfer_v13.xml"

# ----------------------------------------------------------------------------
# Parâmetros do swing (graus e segundos). Os valores por omissão são os que
# saíram do varrimento de viabilidade — ver report/simplificacoes.md.
# ----------------------------------------------------------------------------
DEFAULTS = dict(
    q_top_shoulder=-125.0,  # ângulo dos braços no topo do backswing [deg]
    q_top_wrist=-95.0,      # armação do pulso no topo [deg] (real: ~90)
    f_release=0.65,         # fração do downswing em que o pulso começa a soltar
    t_back=0.75,            # duração do backswing [s]
    t_pause=0.05,           # pausa no topo [s]
    t_down=0.30,            # topo -> impacto [s]  (real: 0.25-0.30 s)
    t_sim=9.0,              # duração total simulada [s] (tem de dar para a bola aterrar)
    kp=900.0,               # ganhos do PD (rad/s^2 por rad, e por rad/s)
    kd=60.0,
)
# Com o modelo de pêndulo duplo (braço + pulso) o downswing já cabe nos 0.30 s
# de um swing real sem saturar os motores. No modelo anterior (tronco + braços,
# dof 90 % redundantes) era preciso 0.38-0.46 s. Ver report/simplificacoes.md.


# ----------------------------------------------------------------------------
# Trajetória de referência
# ----------------------------------------------------------------------------
def _min_jerk(tau):
    """Perfil de jerk mínimo s(tau) em [0,1] e as suas derivadas (tau = t/T)."""
    tau = np.clip(tau, 0.0, 1.0)
    s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    ds = 30 * tau**2 - 60 * tau**3 + 30 * tau**4
    dds = 60 * tau - 180 * tau**2 + 120 * tau**3
    return s, ds, dds


def reference(t, p):
    """
    Posição, velocidade e aceleração de referência das 2 juntas no instante t.

    Fases:
      [0, t_back)                   backswing, jerk mínimo de 0 até q_topo
      [t_back, t_back+t_pause)      pausa no topo
      [.., +t_down)                 downswing em cosseno -> q = 0 no impacto
      [.., +t_down)                 follow-through (o cosseno continua sozinho)
      depois                        mantém a pose final

    Devolve (q_ref, v_ref, a_ref), cada um com 2 elementos [torso, shoulder].
    """
    q_top = np.radians([p["q_top_shoulder"], p["q_top_wrist"]])
    t_back, t_pause, t_down = p["t_back"], p["t_pause"], p["t_down"]
    t_start_down = t_back + t_pause

    if t < t_back:                                   # --- backswing
        s, ds, dds = _min_jerk(t / t_back)
        return q_top * s, q_top * ds / t_back, q_top * dds / t_back**2

    if t < t_start_down:                             # --- pausa no topo
        return q_top.copy(), np.zeros(2), np.zeros(2)

    # --- downswing + follow-through: aceleração angular CONSTANTE
    #     q_i(u) = q_topo_i * (1 - (u/T)^2)
    # As duas juntas usam o MESMO T, logo ambas cruzam q = 0 exatamente em
    # u = T (impacto), com velocidade máxima -2*q_topo_i/T.
    #
    # Porquê parábola e não cosseno: com aceleração constante o binário está no
    # máximo durante TODO o downswing, que é o uso mais eficiente de um motor
    # com limite de binário. Para o mesmo pico de aceleração, a parábola dá
    # 2.0*|q_topo|/T de velocidade no impacto contra 1.57*|q_topo|/T do cosseno
    # (+27 %). Com o cosseno, o pico de binário cai todo no início do
    # downswing, onde a velocidade ainda é zero — desperdício.
    # O fator 1.35 limita o follow-through: a parábola, se a deixassem correr,
    # levava as juntas muito para lá do limite e os braços davam a volta completa
    # ao peito. Com 1.35 a pose final fica ~0.82*|q_topo| depois do impacto, que
    # é a ordem de grandeza de um "finish" real.
    u = t - t_start_down

    # --- braço (dof 1): parábola desde o topo, cruza zero em u = t_down
    tau = min(u / t_down, 1.35)                      # 1.35 = fim do follow-through
    q = q_top * (1.0 - tau**2)
    v = -q_top * 2.0 * tau / t_down
    a = -q_top * 2.0 * np.ones(2) / t_down**2

    # --- pulso (dof 2): LIBERTAÇÃO TARDIA ("lag").
    # O pulso fica armado durante a primeira parte do downswing e só começa a
    # soltar em f_release*t_down; depois solta com aceleração constante e chega
    # também a zero em u = t_down. Isto é o "lag" de que falam os treinadores, e
    # é o que distingue um swing eficiente de um "casting" (soltar cedo): com o
    # taco dobrado a inércia é baixa, logo o braço acelera barato, e a energia
    # é entregue à cabeça do taco no fim, quando o braço de alavanca é longo.
    #
    # O efeito é grande e mede-se: mantendo tudo o resto igual,
    #   f_release = 0.10 -> 26.8 m/s     0.30 -> 29.4 m/s     0.65 -> 37.2 m/s
    # Soltar tarde vale +39 % de velocidade da cabeça do taco. Acima de ~0.70 o
    # pulso já não tem tempo de chegar a zero e a batida sai má.
    t_rel = p.get("f_release", 0.55) * t_down
    if u < t_rel:
        q[1], v[1], a[1] = q_top[1], 0.0, 0.0
    else:
        Tw = t_down - t_rel
        tw = min((u - t_rel) / Tw, 1.35)
        q[1] = q_top[1] * (1.0 - tw**2)
        v[1] = -q_top[1] * 2.0 * tw / Tw
        a[1] = -q_top[1] * 2.0 / Tw**2
    if u / t_down >= 1.35:                           # pose final estática
        v[:] = 0.0
        a[:] = 0.0
    # A referência nunca pode sair dos limites das juntas declarados no XML,
    # senão o follow-through manda o motor contra o batente e o controlador
    # passa o resto do swing a lutar contra a restrição.
    rng = p.get("q_range")
    if rng is not None:
        lo, hi = rng[:, 0] + np.radians(3.0), rng[:, 1] - np.radians(3.0)
        q_c = np.clip(q, lo, hi)
        v = np.where(q_c == q, v, 0.0)
        a = np.where(q_c == q, a, 0.0)
        return q_c, v, a
    return q, v, a


# ----------------------------------------------------------------------------
# Simulação
# ----------------------------------------------------------------------------
def load(path=MODEL):
    """Carrega o modelo e devolve (model, data)."""
    model = mujoco.MjModel.from_xml_path(path)
    return model, mujoco.MjData(model)


def _indices(m):
    """Índices das 2 juntas atuadas em qpos, qvel e ctrl."""
    jt, js = m.joint("shoulder"), m.joint("wrist")
    qadr = np.array([jt.qposadr[0], js.qposadr[0]])
    vadr = np.array([jt.dofadr[0], js.dofadr[0]])
    uadr = np.array([m.actuator("shoulder_motor").id, m.actuator("wrist_motor").id])
    return qadr, vadr, uadr


def run_swing(model, data, params=None, noise=None, seed=None):
    """
    Corre um swing completo e devolve as métricas do impacto.

    params : dict  — sobrepõe DEFAULTS (ver acima).
    noise  : dict  — perturbações, todas opcionais:
                     {"wrist_torque_std": X}  ruído gaussiano [N·m] somado
                                                 ao binário do ombro
                                                 ("pulso trémulo" do enunciado)
                     {"shoulder_torque_std": X}     idem para o tronco
                     {"freq": f}                 largura de banda do ruído [Hz],
                                                 por omissão 10 Hz (tremor
                                                 fisiológico humano: 8-12 Hz)

                 NOTA IMPORTANTE: o ruído é mantido constante durante 1/freq
                 segundos (amostragem com retenção de ordem zero) e NÃO
                 re-amostrado a cada passo de integração. Ruído branco por passo
                 seria fisicamente errado aqui: com dt = 5e-5 s há ~9200 passos
                 num downswing, as amostras independentes cancelavam-se e o
                 efeito media-se a quase zero (o desvio do carry dava 0.03 m).
                 Pior: o resultado dependia do passo de integração escolhido,
                 o que tira sentido ao teste.
    seed   : int   — semente do gerador, para repetibilidade.

    Devolve dict com:
      v_head    velocidade da cabeça do taco no instante do impacto [m/s]
      v_ball    velocidade de lançamento da bola [m/s]
      launch    ângulo de lançamento acima da horizontal [deg]
      side      desvio lateral do lançamento [deg] (+ = para a direita do alvo)
      q_impact  ângulos das juntas no impacto [deg] — devem ser ~(0, 0)
      ball_pos  posição final da bola [m] (x = direção do alvo)
      carry     distância de voo até à 1.ª aterragem [m] — métrica principal
      land_y    desvio lateral no ponto de aterragem [m]
      dist      distância total em x (voo + rolamento) [m]; ver nota no código
      rolling   True se a bola ainda se movia no fim da simulação
      hit       True se houve contacto taco-bola
      tau_max   binário máximo usado em cada junta [N·m]
      sat       fração do downswing em que cada motor esteve saturado
    """
    p = dict(DEFAULTS)
    if params:
        p.update(params)
    noise = noise or {}
    rng = np.random.default_rng(seed)

    m, d = model, data
    mujoco.mj_resetData(m, d)
    qadr, vadr, uadr = _indices(m)
    p["q_range"] = m.jnt_range[[m.joint("shoulder").id, m.joint("wrist").id]]
    n = m.nv

    g_clubhead = m.geom("clubhead").id
    g_ball = m.geom("ball_geom").id
    g_floor = m.geom("floor").id
    b_ball = m.body("ball").id

    ctrl_lo = m.actuator_ctrlrange[uadr, 0]
    ctrl_hi = m.actuator_ctrlrange[uadr, 1]

    kp, kd = p["kp"], p["kd"]
    t_impact_ref = p["t_back"] + p["t_pause"] + p["t_down"]

    M_full = np.zeros((n, n))
    J = np.zeros((3, n))
    tau_noise = np.zeros(2)
    t_next_noise = 0.0

    hit = False
    launch_done = False
    carry = np.nan
    land_y = np.nan
    v_head = v_ball = launch = side = np.nan
    q_impact = np.array([np.nan, np.nan])
    tau_max = np.zeros(2)
    n_sat = np.zeros(2)
    n_down = 0

    nsteps = int(p["t_sim"] / m.opt.timestep)
    for _ in range(nsteps):
        t = d.time
        q_ref, v_ref, a_ref = reference(t, p)
        q = d.qpos[qadr]
        v = d.qvel[vadr]

        # --- binário calculado: tau = M*(a_ref + Kd*ev + Kp*eq) + h
        mujoco.mj_fullM(m, d, M_full)
        M2 = M_full[np.ix_(vadr, vadr)]
        h2 = d.qfrc_bias[vadr]
        a_cmd = a_ref + kd * (v_ref - v) + kp * (q_ref - q)
        tau = M2 @ a_cmd + h2

        # --- perturbações (Tarefa 2): ruído com banda limitada
        if noise:
            if t >= t_next_noise:
                t_next_noise += 1.0 / noise.get("freq", 10.0)
                tau_noise[0] = rng.normal(0.0, noise.get("shoulder_torque_std", 0.0))
                tau_noise[1] = rng.normal(0.0, noise.get("wrist_torque_std", 0.0))
            tau = tau + tau_noise

        # --- saturação: nunca sair do ctrlrange declarado no XML
        tau_sat = np.clip(tau, ctrl_lo, ctrl_hi)
        if p["t_back"] <= t <= t_impact_ref:
            n_down += 1
            n_sat += np.abs(tau - tau_sat) > 1e-9
        tau_max = np.maximum(tau_max, np.abs(tau_sat))
        d.ctrl[uadr] = tau_sat

        mujoco.mj_step(m, d)

        # --- contacto taco-bola neste passo?
        touching = False
        for i in range(d.ncon):
            c = d.contact[i]
            if {c.geom1, c.geom2} == {g_clubhead, g_ball}:
                touching = True
                break

        if touching and not hit and abs(t - t_impact_ref) < 0.20:
            # Primeiro contacto, mas só conta se ocorrer perto do instante de
            # impacto previsto. Sem esta janela, um swing que FALHA a bola
            # acabava por ser contado como acerto: no follow-through (ou já
            # parado) o taco encosta na bola em repouso e isso era registado
            # como impacto a ~0 m/s, poluindo as estatísticas.
            mujoco.mj_jacGeom(m, d, J, None, g_clubhead)
            v_head = float(np.linalg.norm(J @ d.qvel))
            q_impact = np.degrees(d.qpos[qadr].copy())
            hit = True

        # O lançamento só pode ser medido ENQUANTO a bola está em contacto com
        # a face (e no passo seguinte). Depois disso a gravidade acelera-a na
        # descida e a velocidade volta a subir — medir o máximo ao longo de
        # todo o voo dava a velocidade de aterragem, com ângulo negativo.
        if hit and not launch_done:
            if touching:
                vb = d.cvel[b_ball][3:].copy()
                s = float(np.linalg.norm(vb))
                if not (v_ball >= s):      # cobre o caso v_ball = nan
                    v_ball = s
                    launch = float(np.degrees(np.arctan2(vb[2], np.hypot(vb[0], vb[1]))))
                    side = float(np.degrees(np.arctan2(-vb[1], vb[0])))
            else:
                launch_done = True         # a bola largou a face: congela

        # --- primeira aterragem: define o CARRY (métrica padrão no golfe).
        # A distância total (voo + rolamento) não é de confiar neste modelo:
        # o relvado é um plano rígido, sem deformação nem efeito do backspin,
        # por isso a bola rola muito mais do que rolaria num fairway real.
        if launch_done and np.isnan(carry):
            for i in range(d.ncon):
                c = d.contact[i]
                if {c.geom1, c.geom2} == {g_ball, g_floor}:
                    carry = float(d.body("ball").xpos[0])
                    land_y = float(d.body("ball").xpos[1])
                    break

    ball_pos = d.body("ball").xpos.copy()
    return dict(
        v_head=v_head, v_ball=v_ball, launch=launch, side=side,
        q_impact=q_impact, ball_pos=ball_pos, dist=float(ball_pos[0]),
        carry=carry, land_y=land_y,
        rolling=bool(np.linalg.norm(d.cvel[b_ball][3:]) > 0.05),
        hit=hit, tau_max=tau_max,
        sat=n_sat / max(n_down, 1),
    )


# ----------------------------------------------------------------------------
# Viewer
# ----------------------------------------------------------------------------
def view(params=None):
    """
    Abre o viewer com o swing a correr em tempo real.

    O controlador é registado como callback do MuJoCo (set_mjcb_control) e o
    viewer trata do ciclo de simulação. Foi feito assim de propósito: o outro
    caminho (launch_passive, em que somos nós a chamar mj_step) exige o
    interpretador `mjpython` em macOS, e esse vem avariado nesta versão. Com o
    callback + viewer "managed" funciona com o python normal nos três sistemas.

    Teclas: Space = play/pausa · Backspace = repõe e faz novo swing ·
            Ctrl+L = recarrega o XML.
    """
    p = dict(DEFAULTS)
    if params:
        p.update(params)
    m, d = load()
    qadr, vadr, uadr = _indices(m)
    p["q_range"] = m.jnt_range[[m.joint("shoulder").id, m.joint("wrist").id]]
    M_full = np.zeros((m.nv, m.nv))
    ctrl_lo = m.actuator_ctrlrange[uadr, 0]
    ctrl_hi = m.actuator_ctrlrange[uadr, 1]

    def controlador(model, data):
        q_ref, v_ref, a_ref = reference(data.time, p)
        mujoco.mj_fullM(model, data, M_full)
        M2 = M_full[np.ix_(vadr, vadr)]
        a_cmd = (a_ref + p["kd"] * (v_ref - data.qvel[vadr])
                 + p["kp"] * (q_ref - data.qpos[qadr]))
        data.ctrl[uadr] = np.clip(M2 @ a_cmd + data.qfrc_bias[vadr],
                                  ctrl_lo, ctrl_hi)

    mujoco.set_mjcb_control(controlador)
    try:
        print("Viewer aberto. Space = play/pausa, Backspace = novo swing.")
        mujoco.viewer.launch(m, d)
    finally:
        mujoco.set_mjcb_control(None)


# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Swing do golfista V13 (2 dof)")
    ap.add_argument("--view", action="store_true", help="abre o viewer")
    ap.add_argument("--t-down", type=float, help="duração do downswing [s]")
    ap.add_argument("--q-top-shoulder", type=float, help="topo dos braços [deg]")
    ap.add_argument("--q-top-wrist", type=float, help="armação do pulso [deg]")
    ap.add_argument("--f-release", type=float, help="fração do downswing p/ soltar o pulso")
    args = ap.parse_args()

    params = {}
    if args.t_down is not None:
        params["t_down"] = args.t_down
    if args.q_top_shoulder is not None:
        params["q_top_shoulder"] = args.q_top_shoulder
    if args.q_top_wrist is not None:
        params["q_top_wrist"] = args.q_top_wrist
    if args.f_release is not None:
        params["f_release"] = args.f_release

    if args.view:
        view(params)
        return

    m, d = load()
    r = run_swing(m, d, params)
    if not r["hit"]:
        print("FALHOU: o taco não tocou na bola.")
        return
    print(f"  cabeça do taco no impacto : {r['v_head']:6.1f} m/s")
    print(f"  bola                      : {r['v_ball']:6.1f} m/s  "
          f"(smash {r['v_ball']/r['v_head']:.2f})")
    print(f"  lançamento                : {r['launch']:6.1f} deg   "
          f"desvio lateral {r['side']:+.1f} deg")
    print(f"  juntas no impacto         : braço {r['q_impact'][0]:+.1f} deg, "
          f"pulso {r['q_impact'][1]:+.1f} deg   (alvo: 0, 0)")
    if np.isnan(r["carry"]):
        print("  carry (até aterrar)       :    -    "
              "(a bola não aterrou dentro de t_sim)")
    else:
        print(f"  carry (até aterrar)       : {r['carry']:6.1f} m   "
              f"(desvio lateral {r['land_y']:+.2f} m)")
    print(f"  binário máx usado         : braço {r['tau_max'][0]:.0f} N.m, "
          f"pulso {r['tau_max'][1]:.0f} N.m")
    print(f"  saturação no downswing    : braço {r['sat'][0]*100:.0f} %, "
          f"pulso {r['sat'][1]*100:.0f} %")
    # o taco só passa na bola se as duas juntas chegarem a zero juntas; se o
    # impacto se der longe de (0,0) a batida é má, por muito boa que a
    # velocidade pareça
    if np.abs(r["q_impact"]).max() > 3.0:
        print("  AVISO: impacto longe de (0,0) — batida má. Provavelmente os "
              "motores saturaram e a trajetória deixou de ser seguida.")


if __name__ == "__main__":
    main()
