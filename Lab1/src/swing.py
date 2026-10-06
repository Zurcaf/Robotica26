"""
Swing do golfista — V17 (3 dof: tronco + ombro esq. + pulso) e V13 (2 dof).

Uso (a partir de Lab1/):
    python src/swing.py                              # V17: um swing, imprime os resultados
    python src/swing.py --modelo models/golfer_v13.xml   # o modelo de 2 dof
    python src/swing.py --view                       # viewer com o swing em tempo real
    python src/swing.py --t-down 0.32                # downswing mais lento

O modelo é detetado pelo XML (tem junta "torso" => V17). Cada modelo tem a sua
lista de juntas atuadas, os seus parâmetros por omissão e o seu perfil de
trajetória; todo o resto (controlador, medições, passo adaptativo, ruído) é
comum e funciona para qualquer número de juntas.

IDEIA DO CONTROLO
-----------------
A cabeça do taco só passa na bola quando TODAS as juntas atuadas estão em q = 0
ao mesmo tempo (é assim que a pose de endereço está definida nos XML). Se uma
junta chega antes da outra, o arco passa ao lado ou por cima da bola e a face
chega com a orientação errada.

Resolve-se em duas partes:

1) TRAJETÓRIA de referência que cruza zero em simultâneo, por construção.

   V17: sequência de polinómios de 5.º grau (quínticos) por junta, com posição,
   velocidade e aceleração contínuas e aceleração nula nas extremidades, por
   isso os binários pedidos não têm degraus. Fases: backswing (cada junta
   arranca na sua fração f_back), pausa no topo, downswing (cada junta arranca
   na sua fração f_down — o pulso fica armado até 55 %, o "lag"), impacto com
   todas as juntas em q = 0 no mesmo instante t_imp, follow-through até à pose
   final. No impacto cada junta chega com a velocidade v_imp que se lhe impõe:
   o ombro chega parado (ponto mais baixo do arco das mãos), o tronco ainda a
   rodar, o pulso no pico. Regra útil: com v_imp = 2·|q_topo|/T o quíntico do
   downswing colapsa em q = Δq·(2τ³ − τ⁴), cuja velocidade é monótona e tem o
   pico EXATAMENTE no impacto — a mesma velocidade de impacto que a parábola de
   aceleração constante do V13, mas de classe C².

   V13: aceleração angular constante no downswing (q = q_topo·(1 − (u/T)²)) e
   libertação tardia do pulso em f_release·T. Mantido tal e qual para os
   números documentados em report/simplificacoes.md continuarem reproduzíveis.

2) CONTROLO POR BINÁRIO CALCULADO (computed torque / inverse dynamics control,
   Siciliano et al. §8.5.2; Sequeira, Introdução à Robótica, cap. 5 e 7):

       tau = M(q)·[ a_ref + Kd·(v_ref - v) + Kp·(q_ref - q) ] + h(q, v)

   M é a submatriz de massa das juntas atuadas (mj_fullM) e h = qfrc_bias reúne
   Coriolis/centrífugas e gravidade. A parte M·a_ref + h é o feedforward que
   "sabe" a dinâmica, e o PD só corrige o erro residual. No V17 o braço direito
   passivo (preso ao punho por uma restrição) e a cabeça NÃO entram em M: o seu
   efeito é uma perturbação que o PD absorve. Os binários são sempre saturados
   no ctrlrange do XML, por isso o modelo nunca usa força que um humano não
   conseguisse fazer. O arrasto do ar não entra em h (o MuJoCo põe-no em
   qfrc_fluid); com comp_fluid=True o controlador também o compensa — ver
   src/teste_aerodinamica.py para o efeito (pequeno, porque os motores saturam).

CUIDADOS NUMÉRICOS (não regredir — ver report/simplificacoes.md §3.1)
  - a velocidade da cabeça lê-se ANTES do passo em que o contacto é detetado;
  - a velocidade da bola amostra-se também no passo em que o contacto acaba;
  - passo de 5e-6 s enquanto a cabeça está a menos de 15 cm da bola;
  - o ruído é mantido 1/freq s (ordem zero), não re-amostrado a cada passo.
"""

import argparse
import numpy as np
import mujoco
import mujoco.viewer

MODEL = "models/golfer_v17.xml"
MODEL_V13 = "models/golfer_v13.xml"

DT_IMPACTO = 5e-6   # passo [s] com a cabeça do taco perto da bola (ver run_swing)
R_IMPACTO = 0.15    # "perto" = centros a menos de 15 cm

# ----------------------------------------------------------------------------
# Parâmetros do swing (graus e segundos), um conjunto por modelo.
# ----------------------------------------------------------------------------
DEFAULTS_V13 = dict(
    perfil="v13",
    q_top_shoulder=-125.0,  # ângulo dos braços no topo do backswing [deg]
    q_top_wrist=-95.0,      # armação do pulso no topo [deg] (real: ~90)
    f_release=0.65,         # fração do downswing em que o pulso começa a soltar
    t_back=0.75,            # duração do backswing [s]
    t_pause=0.05,           # pausa no topo [s]
    t_down=0.30,            # topo -> impacto [s]  (real: 0.25-0.30 s)
    t_sim=9.0,              # duração total simulada [s] (tem de dar para a bola aterrar)
    kp=900.0,               # ganhos do PD (rad/s^2 por rad, e por rad/s)
    kd=60.0,
    comp_fluid=False,       # True: o controlador compensa também o arrasto do ar
    t_noise_on=0.0,         # instante a partir do qual o ruído atua [s]
)

DEFAULTS_V17 = dict(
    perfil="v17",
    # ordem das juntas: tronco, ombro, pulso
    q_top=(-90.0, -85.0, -95.0),    # topo do backswing [deg]
    q_final=(85.0, -70.0, 105.0),   # pose final do follow-through [deg]
    f_back=(0.0, 0.0, 0.30),        # fração do backswing em que cada junta arranca
    f_down=(0.0, 0.10, 0.55),       # fração do downswing em que cada junta arranca
                                    # (o pulso fica armado até 55 %: o "lag")
    v_imp=(425.0, 0.0, None),       # velocidade no impacto [deg/s]; None = 2|q_top|/T
                                    # (pico exatamente no impacto)
    t_ft_rest=0.30,                 # follow-through de uma junta que chega parada [s]
    t_back=0.80,
    t_pause=0.05,
    t_down=0.30,
    t_sim=9.0,
    kp=900.0,
    kd=60.0,
    comp_fluid=False,
    t_noise_on=0.24,                # = 0.30*t_back: quando o pulso começa a armar e
                                    # o taco já saiu do endereço. Antes disso, 10 N.m
                                    # no pulso em repouso dão um erro PD de
                                    # tau/(Kp*M_ww) ~ 3 deg = 35 mm na cabeça, mais
                                    # do que qualquer folga razoável à bola.
)
DEFAULTS = DEFAULTS_V17

CONFIGS = {
    "v13": dict(joints=["shoulder", "wrist"],
                motors=["shoulder_motor", "wrist_motor"],
                labels=["braço", "pulso"], defaults=DEFAULTS_V13),
    "v17": dict(joints=["torso", "shoulder", "wrist"],
                motors=["torso_motor", "shoulder_motor", "wrist_motor"],
                labels=["tronco", "ombro", "pulso"], defaults=DEFAULTS_V17),
}


def config(m):
    """Configuração (juntas, motores, rótulos, DEFAULTS) do modelo carregado."""
    tem_torso = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "torso") >= 0
    return CONFIGS["v17" if tem_torso else "v13"]


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


def _quintico(T, q0, v0, a0, q1, v1, a1):
    """Coeficientes (grau 5 -> 0) do polinómio com estas condições fronteira."""
    A = np.array([[0, 0, 0, 0, 0, 1],
                  [0, 0, 0, 0, 1, 0],
                  [0, 0, 0, 2, 0, 0],
                  [T**5, T**4, T**3, T**2, T, 1],
                  [5*T**4, 4*T**3, 3*T**2, 2*T, 1, 0],
                  [20*T**3, 12*T**2, 6*T, 2, 0, 0]], dtype=float)
    return np.linalg.solve(A, np.array([q0, v0, a0, q1, v1, a1], dtype=float))


def _segmentos_v17(p):
    """
    Constrói, por junta, a lista de segmentos [(t0, t1, coef ou valor)].
    coef = polinómio quíntico em (t - t0); um float = junta parada nesse valor.
    """
    nj = len(p["q_top"])
    q_top = np.radians(p["q_top"])
    q_fin = np.radians(p["q_final"])
    t_back, t_pause, t_down = p["t_back"], p["t_pause"], p["t_down"]
    t_imp = t_back + t_pause + t_down
    segs = []
    for i in range(nj):
        s = []
        tb0 = p["f_back"][i] * t_back                 # arranque do backswing
        td0 = t_back + t_pause + p["f_down"][i] * t_down   # arranque do downswing
        v_imp = p["v_imp"][i]
        if v_imp is None:                             # pico exatamente no impacto
            v_imp = 2.0 * (0.0 - q_top[i]) / (t_imp - td0)
        else:
            v_imp = np.radians(v_imp)
        if abs(v_imp) > 1e-9:
            t_ft = 2.0 * abs(q_fin[i]) / abs(v_imp)    # travagem em espelho
        else:
            t_ft = p["t_ft_rest"]
        s.append((0.0, tb0, 0.0))
        s.append((tb0, t_back, _quintico(t_back - tb0, 0, 0, 0, q_top[i], 0, 0)))
        s.append((t_back, td0, float(q_top[i])))
        s.append((td0, t_imp, _quintico(t_imp - td0, q_top[i], 0, 0, 0, v_imp, 0)))
        s.append((t_imp, t_imp + t_ft, _quintico(t_ft, 0, v_imp, 0, q_fin[i], 0, 0)))
        s.append((t_imp + t_ft, np.inf, float(q_fin[i])))
        segs.append(s)
    return segs


def _reference_v17(t, p):
    if "_segs" not in p:
        p["_segs"] = _segmentos_v17(p)
    nj = len(p["_segs"])
    q, v, a = np.zeros(nj), np.zeros(nj), np.zeros(nj)
    for i, s in enumerate(p["_segs"]):
        for t0, t1, c in s:
            if t0 <= t < t1 or (t1 == np.inf):
                if isinstance(c, float):
                    q[i] = c
                else:
                    u = t - t0
                    q[i] = np.polyval(c, u)
                    v[i] = np.polyval(np.polyder(c), u)
                    a[i] = np.polyval(np.polyder(c, 2), u)
                break
    return q, v, a


def _reference_v13(t, p):
    """
    Posição, velocidade e aceleração de referência das 2 juntas do V13.

    Fases:
      [0, t_back)                   backswing, jerk mínimo de 0 até q_topo
      [t_back, t_back+t_pause)      pausa no topo
      [.., +t_down)                 downswing -> q = 0 no impacto (braço em
                                    parábola; pulso armado e depois a soltar)
      [.., +0.35*t_down)            follow-through
      depois                        mantém a pose final
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
    # u = T (impacto), com velocidade máxima -2*q_topo_i/T. Com aceleração
    # constante o binário está no máximo durante TODO o downswing: para o mesmo
    # pico de aceleração dá +27 % de velocidade no impacto face a um cosseno.
    # O fator 1.35 limita o follow-through (pose final ~0.82*|q_topo|).
    u = t - t_start_down
    tau = min(u / t_down, 1.35)
    q = q_top * (1.0 - tau**2)
    v = -q_top * 2.0 * tau / t_down
    a = -q_top * 2.0 * np.ones(2) / t_down**2

    # --- pulso: LIBERTAÇÃO TARDIA ("lag"). Fica armado até f_release*t_down e
    # depois solta com aceleração constante, chegando também a zero em u = T.
    # Soltar a 10 % dá 29.6 m/s, a 65 % dá 38.8 m/s (+31 %).
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
    return q, v, a


def reference(t, p):
    """
    (q_ref, v_ref, a_ref) das juntas atuadas no instante t, segundo p["perfil"].
    A referência nunca sai dos limites das juntas (p["q_range"], com margem de
    3°), senão o follow-through manda o motor contra o batente e o controlador
    passa o resto do swing a lutar contra a restrição.
    """
    if p.get("perfil", "v17") == "v13":
        q, v, a = _reference_v13(t, p)
    else:
        q, v, a = _reference_v17(t, p)
    rng = p.get("q_range")
    if rng is not None:
        lo, hi = rng[:, 0] + np.radians(3.0), rng[:, 1] - np.radians(3.0)
        q_c = np.clip(q, lo, hi)
        v = np.where(q_c == q, v, 0.0)
        a = np.where(q_c == q, a, 0.0)
        return q_c, v, a
    return q, v, a


def t_impacto(p):
    """Instante de impacto previsto pela referência [s]."""
    return p["t_back"] + p["t_pause"] + p["t_down"]


# ----------------------------------------------------------------------------
# Simulação
# ----------------------------------------------------------------------------
def load(path=MODEL):
    """Carrega o modelo e devolve (model, data)."""
    model = mujoco.MjModel.from_xml_path(path)
    return model, mujoco.MjData(model)


def _indices(m):
    """Índices das juntas atuadas em qpos, qvel e ctrl (pela ordem da config)."""
    cfg = config(m)
    qadr = np.array([m.joint(j).qposadr[0] for j in cfg["joints"]])
    vadr = np.array([m.joint(j).dofadr[0] for j in cfg["joints"]])
    uadr = np.array([m.actuator(a).id for a in cfg["motors"]])
    return qadr, vadr, uadr


def _prep(m, params=None):
    """Parâmetros completos (DEFAULTS do modelo + params + q_range) e índices."""
    cfg = config(m)
    p = dict(cfg["defaults"])
    if params:
        p.update(params)
    p["q_range"] = m.jnt_range[[m.joint(j).id for j in cfg["joints"]]]
    qadr, vadr, uadr = _indices(m)
    return p, cfg, qadr, vadr, uadr


def binario_calculado(m, d, p, qadr, vadr, M_full):
    """tau = M*(a_ref + Kd*ev + Kp*eq) + h, SEM saturação."""
    q_ref, v_ref, a_ref = reference(d.time, p)
    mujoco.mj_fullM(m, d, M_full)
    M2 = M_full[np.ix_(vadr, vadr)]
    a_cmd = a_ref + p["kd"] * (v_ref - d.qvel[vadr]) + p["kp"] * (q_ref - d.qpos[qadr])
    tau = M2 @ a_cmd + d.qfrc_bias[vadr]
    if p.get("comp_fluid"):
        tau = tau - d.qfrc_fluid[vadr]          # arrasto do ar (não está em h)
    return tau


def controlo(m, d, p, qadr, vadr, uadr, M_full):
    """Binário calculado saturado no ctrlrange — o que os motores aplicam."""
    tau = binario_calculado(m, d, p, qadr, vadr, M_full)
    return np.clip(tau, m.actuator_ctrlrange[uadr, 0], m.actuator_ctrlrange[uadr, 1])


def _connect_anchor_points(m, d):
    """Pontos (mundo) dos dois lados da restrição connect mão-punho, se existir."""
    eid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_EQUALITY, "hand_r_grip")
    if eid < 0:
        return None
    b1, b2 = m.eq_obj1id[eid], m.eq_obj2id[eid]
    a1, a2 = m.eq_data[eid][0:3], m.eq_data[eid][3:6]
    p1 = d.xpos[b1] + d.xmat[b1].reshape(3, 3) @ a1
    p2 = d.xpos[b2] + d.xmat[b2].reshape(3, 3) @ a2
    return p1, p2


def run_swing(model, data, params=None, noise=None, seed=None,
              guardar_traj=False, guardar_juntas=False):
    """
    Corre um swing completo e devolve as métricas do impacto.

    params : dict  — sobrepõe os DEFAULTS do modelo (ver acima).
    noise  : dict  — perturbações, todas opcionais:
                     {"<junta>_torque_std": X}  ruído gaussiano [N·m] somado ao
                                                 binário dessa junta (torso,
                                                 shoulder, wrist); "pulso trémulo"
                                                 do enunciado = wrist_torque_std
                     {"freq": f}                 largura de banda do ruído [Hz],
                                                 por omissão 10 Hz (tremor
                                                 fisiológico reforçado: 8-12 Hz)
                     {"t_on": t}                 instante a partir do qual o ruído
                                                 atua [s]; por omissão
                                                 p["t_noise_on"]

                 NOTA IMPORTANTE: o ruído é mantido constante durante 1/freq
                 segundos (amostragem com retenção de ordem zero) e NÃO
                 re-amostrado a cada passo de integração. Ruído branco por passo
                 seria fisicamente errado aqui: com dt = 5e-5 s há ~6000 passos
                 num downswing, as amostras independentes cancelavam-se e o
                 efeito media-se a quase zero. Pior: o resultado dependia do
                 passo de integração escolhido, o que tira sentido ao teste.
    seed   : int   — semente do gerador, para repetibilidade.
    guardar_traj   — guarda a trajetória da bola em r["traj"] (t, x, y, z a 2 ms)
    guardar_juntas — guarda r["juntas"]: linhas [t, q..., q_ref..., v..., tau...]
                     (graus, graus/s, N·m) a cada 1 ms, para gráficos.

    Devolve dict com:
      v_head    velocidade da cabeça do taco no instante do impacto [m/s]
      v_ball    velocidade de lançamento da bola [m/s]
      launch    ângulo de lançamento acima da horizontal [deg]
      side      desvio lateral do lançamento [deg] (+ = para a direita do alvo)
      q_impact  ângulos das juntas no impacto [deg] — devem ser ~0
      w_impact  velocidades das juntas no impacto [deg/s]
      t_impact  instante do impacto [s]
      ball_pos  posição final da bola [m] (x = direção do alvo)
      attack    ângulo de ataque da cabeça no impacto [deg] (<0 = a descer)
      loft_dyn  loft dinâmico: inclinação da face no impacto [deg]
      spin_rpm  rotação da bola ao sair da face [rpm]
      carry     distância de voo até à 1.ª aterragem [m] — métrica principal
      land_y    desvio lateral no ponto de aterragem [m] (+ = lado do jogador
                = esquerda do alvo para um destro; side e land_y têm sinais opostos)
      apogeu    altura máxima da bola durante o voo [m]
      t_voo     tempo entre o lançamento e a 1.ª aterragem [s]
      descida   ângulo de descida na aterragem [deg] (+ = a cair)
      traj      trajetória da bola [t, x, y, z] (se guardar_traj)
      juntas    registo das juntas (se guardar_juntas)
      dist      distância total em x (voo + rolamento) [m]; ver nota no código
      rolling   True se a bola ainda se movia no fim da simulação
      hit       True se houve contacto taco-bola
      tau_max   binário máximo usado em cada junta [N·m]
      sat       fração do downswing em que cada motor esteve saturado
      fluid_max binário máximo do ar em cada junta até ao impacto [N·m]
      conn_err_max  erro máximo da restrição mão direita-punho [m] (V17)
    """
    m, d = model, data
    p, cfg, qadr, vadr, uadr = _prep(m, params)
    noise = noise or {}
    rng = np.random.default_rng(seed)
    nj = len(qadr)

    mujoco.mj_resetData(m, d)
    n = m.nv

    g_clubhead = m.geom("clubhead").id
    g_ball = m.geom("ball_geom").id
    g_floor = m.geom("floor").id
    b_ball = m.body("ball").id

    ctrl_lo = m.actuator_ctrlrange[uadr, 0]
    ctrl_hi = m.actuator_ctrlrange[uadr, 1]
    t_impact_ref = t_impacto(p)

    M_full = np.zeros((n, n))
    J = np.zeros((3, n))
    tau_noise = np.zeros(nj)
    t_on = noise.get("t_on", p.get("t_noise_on", 0.0))
    t_next_noise = t_on

    hit = False
    launch_done = False
    carry = np.nan
    land_y = np.nan
    apogeu = 0.0            # altura máxima atingida durante o voo
    t_launch = np.nan       # instante em que a bola larga a face
    t_voo = np.nan          # duração do voo
    descida = np.nan        # ângulo de descida na aterragem
    v_ant = np.zeros(3)     # velocidade da bola no passo anterior (p/ o ângulo)
    traj = []               # [t, x, y, z] amostrado a cada 2 ms
    juntas = []             # [t, q, q_ref, v, tau] amostrado a cada 1 ms
    t_next_juntas = 0.0
    attack = loft_dyn = spin_rpm = np.nan
    v_head = v_ball = launch = side = np.nan
    t_impact = np.nan
    q_impact = np.full(nj, np.nan)
    w_impact = np.full(nj, np.nan)
    tau_max = np.zeros(nj)
    n_sat = np.zeros(nj)
    n_down = 0.0
    fluid_max = np.zeros(nj)
    conn_err_max = 0.0
    tem_connect = _connect_anchor_points(m, d) is not None

    # Passo adaptativo: o do XML no resto do swing e no voo, DT_IMPACTO quando a
    # cabeça do taco está perto da bola. Com 5e-5 s em todo o lado o contacto não
    # convergia: com mudanças de 1 ms no downswing a velocidade da bola saltava
    # 44.7-49.0 m/s, conforme o ponto do passo em que o contacto começava.
    # Com 5e-6 s perto da bola fica em 46.9-47.3 m/s (~1 %). O passo fino só
    # dura alguns ms, por isso o custo extra é desprezável.
    dt_base = m.opt.timestep
    while d.time < p["t_sim"]:
        t = d.time
        perto = np.linalg.norm(d.geom_xpos[g_clubhead] - d.geom_xpos[g_ball]) < R_IMPACTO
        dt = DT_IMPACTO if perto else dt_base
        m.opt.timestep = dt

        # --- binário calculado: tau = M*(a_ref + Kd*ev + Kp*eq) + h
        tau = binario_calculado(m, d, p, qadr, vadr, M_full)

        # --- perturbações (Tarefa 2): ruído com banda limitada, só depois de t_on
        if noise and t >= t_on:
            if t >= t_next_noise:
                t_next_noise += 1.0 / noise.get("freq", 10.0)
                for i, j in enumerate(cfg["joints"]):
                    tau_noise[i] = rng.normal(0.0, noise.get(f"{j}_torque_std", 0.0))
            tau = tau + tau_noise

        # --- saturação: nunca sair do ctrlrange declarado no XML
        tau_sat = np.clip(tau, ctrl_lo, ctrl_hi)
        if t <= t_impact_ref:
            fluid_max = np.maximum(fluid_max, np.abs(d.qfrc_fluid[vadr]))
            if tem_connect:
                p1, p2 = _connect_anchor_points(m, d)
                conn_err_max = max(conn_err_max, float(np.linalg.norm(p1 - p2)))
        if p["t_back"] <= t <= t_impact_ref:
            n_down += dt
            n_sat += dt * (np.abs(tau - tau_sat) > 1e-9)
        tau_max = np.maximum(tau_max, np.abs(tau_sat))
        d.ctrl[uadr] = tau_sat

        if guardar_juntas and t >= t_next_juntas:
            t_next_juntas += 1e-3
            q_ref, _, _ = reference(t, p)
            juntas.append(np.concatenate(([t], np.degrees(d.qpos[qadr]),
                                          np.degrees(q_ref), np.degrees(d.qvel[vadr]),
                                          tau_sat)))

        # Estado ANTES do passo: é nele que o mj_step deteta os contactos. Depois
        # do passo, d.qvel já inclui parte do impulso do contacto, e ler a
        # velocidade da cabeça aí dava valores que saltavam 34.9-37.2 m/s com
        # mudanças de 1 ms no downswing (smash 1.22-1.47, sem sentido físico).
        if not hit and abs(t - t_impact_ref) < 0.20:
            mujoco.mj_jacGeom(m, d, J, None, g_clubhead)
            vh_pre = J @ d.qvel
            q_pre = d.qpos[qadr].copy()
            w_pre = d.qvel[vadr].copy()

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
            vh = vh_pre
            v_head = float(np.linalg.norm(vh))
            q_impact = np.degrees(q_pre)
            w_impact = np.degrees(w_pre)
            t_impact = t
            # ângulo de ataque: inclinação da trajetória da cabeça (<0 = a descer)
            attack = float(np.degrees(np.arctan2(vh[2], np.hypot(vh[0], vh[1]))))
            # loft dinâmico: inclinação da normal da face (eixo x local do geom)
            n_face = d.geom_xmat[g_clubhead].reshape(3, 3)[:, 0]
            loft_dyn = float(np.degrees(np.arcsin(np.clip(n_face[2], -1, 1))))
            hit = True

        # O lançamento só pode ser medido ENQUANTO a bola está em contacto com
        # a face (e no passo seguinte). Depois disso a gravidade acelera-a na
        # descida e a velocidade volta a subir — medir o máximo ao longo de
        # todo o voo dava a velocidade de aterragem, com ângulo negativo.
        # O "passo seguinte" conta mesmo: d.cvel e d.contact são calculados no
        # início do mj_step, antes da integração, por isso a velocidade do passo
        # em que o contacto acaba ainda é a do último instante de contacto.
        # Sem esta amostra a medição dependia de onde caía o fim do contacto:
        # com ar no taco media 37.6 m/s para uma bola que saía a 46.5 m/s.
        if hit and not launch_done:
            vb = d.cvel[b_ball][3:].copy()
            s = float(np.linalg.norm(vb))
            if not (v_ball >= s):          # cobre o caso v_ball = nan
                v_ball = s
                launch = float(np.degrees(np.arctan2(vb[2], np.hypot(vb[0], vb[1]))))
                side = float(np.degrees(np.arctan2(-vb[1], vb[0])))
            if not touching:
                launch_done = True         # a bola largou a face: congela
                t_launch = d.time
                w = d.cvel[b_ball][:3]     # velocidade angular [rad/s]
                spin_rpm = float(np.linalg.norm(w) * 60.0 / (2.0 * np.pi))

        # --- durante o voo: apogeu e trajetória (para o gráfico e o CSV)
        if launch_done and np.isnan(carry):
            pos = d.body("ball").xpos
            apogeu = max(apogeu, float(pos[2]))
            tv = d.time - t_launch                      # tempo desde o lançamento
            if guardar_traj and (not traj or tv - traj[-1][0] >= 0.002):
                traj.append([tv, float(pos[0]), float(pos[1]), float(pos[2])])
            v_ant = d.cvel[b_ball][3:].copy()

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
                    t_voo = float(d.time - t_launch)
                    # ângulo de descida: inclinação da velocidade mesmo antes de tocar
                    descida = float(np.degrees(np.arctan2(-v_ant[2],
                                                          np.hypot(v_ant[0], v_ant[1]))))
                    break

    m.opt.timestep = dt_base
    ball_pos = d.body("ball").xpos.copy()
    return dict(
        v_head=v_head, v_ball=v_ball, launch=launch, side=side,
        q_impact=q_impact, w_impact=w_impact, t_impact=t_impact,
        ball_pos=ball_pos, dist=float(ball_pos[0]),
        carry=carry, land_y=land_y,
        apogeu=apogeu, t_voo=t_voo, descida=descida,
        traj=np.array(traj) if traj else np.zeros((0, 4)),
        juntas=np.array(juntas) if juntas else np.zeros((0, 1 + 4 * nj)),
        attack=attack, loft_dyn=loft_dyn, spin_rpm=spin_rpm,
        rolling=bool(np.linalg.norm(d.cvel[b_ball][3:]) > 0.05),
        hit=hit, tau_max=tau_max,
        sat=n_sat / max(n_down, 1e-12),
        fluid_max=fluid_max,
        conn_err_max=conn_err_max,
        labels=list(cfg["labels"]), joints=list(cfg["joints"]),
    )


# ----------------------------------------------------------------------------
# Viewer
# ----------------------------------------------------------------------------
def view(params=None, modelo=MODEL):
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
    m, d = load(modelo)
    p, cfg, qadr, vadr, uadr = _prep(m, params)
    M_full = np.zeros((m.nv, m.nv))

    def controlador(model, data):
        data.ctrl[uadr] = controlo(model, data, p, qadr, vadr, uadr, M_full)

    mujoco.set_mjcb_control(controlador)
    try:
        print("Viewer aberto. Space = play/pausa, Backspace = novo swing.")
        mujoco.viewer.launch(m, d)
    finally:
        mujoco.set_mjcb_control(None)


# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description="Swing do golfista (V17: 3 dof; V13: 2 dof)")
    ap.add_argument("--modelo", default=MODEL, help="XML do modelo")
    ap.add_argument("--view", action="store_true", help="abre o viewer")
    ap.add_argument("--t-down", type=float, help="duração do downswing [s]")
    ap.add_argument("--q-top", help="topo do backswing [deg], ex.: -90,-85,-95")
    ap.add_argument("--f-release", type=float,
                    help="(só V13) fração do downswing p/ soltar o pulso")
    ap.add_argument("--comp-fluid", action="store_true",
                    help="o controlador compensa também o arrasto do ar")
    args = ap.parse_args()

    m, d = load(args.modelo)
    cfg = config(m)
    params = {}
    if args.t_down is not None:
        params["t_down"] = args.t_down
    if args.q_top is not None:
        vals = [float(x) for x in args.q_top.split(",")]
        if cfg["defaults"]["perfil"] == "v13":
            params["q_top_shoulder"], params["q_top_wrist"] = vals
        else:
            params["q_top"] = tuple(vals)
    if args.f_release is not None:
        params["f_release"] = args.f_release
    if args.comp_fluid:
        params["comp_fluid"] = True

    if args.view:
        view(params, args.modelo)
        return

    r = run_swing(m, d, params)
    lab = cfg["labels"]
    if not r["hit"]:
        print("FALHOU: o taco não tocou na bola.")
        return
    print(f"  cabeça do taco no impacto : {r['v_head']:6.1f} m/s")
    print(f"  bola                      : {r['v_ball']:6.1f} m/s  "
          f"(smash {r['v_ball']/r['v_head']:.2f})")
    print(f"  lançamento                : {r['launch']:6.1f} deg   "
          f"desvio lateral {r['side']:+.1f} deg")
    print(f"  ataque / loft dinâmico    : {r['attack']:+6.1f} / {r['loft_dyn']:.1f} deg   "
          f"spin {r['spin_rpm']:.0f} rpm")
    print("  juntas no impacto         : "
          + ", ".join(f"{l} {q:+.1f} deg" for l, q in zip(lab, r["q_impact"]))
          + "   (alvo: 0)")
    print("  vel. juntas no impacto    : "
          + ", ".join(f"{l} {w:+.0f} deg/s" for l, w in zip(lab, r["w_impact"])))
    if np.isnan(r["carry"]):
        print("  carry (até aterrar)       :    -    "
              "(a bola não aterrou dentro de t_sim)")
    else:
        print(f"  carry (até aterrar)       : {r['carry']:6.1f} m   "
              f"(desvio lateral {r['land_y']:+.2f} m)")
    print(f"  apogeu / tempo de voo     : {r['apogeu']:6.1f} m / {r['t_voo']:.2f} s")
    print(f"  ângulo de descida         : {r['descida']:6.1f} deg")
    print("  binário máx usado         : "
          + ", ".join(f"{l} {t:.0f} N.m" for l, t in zip(lab, r["tau_max"])))
    print("  saturação no downswing    : "
          + ", ".join(f"{l} {s*100:.0f} %" for l, s in zip(lab, r["sat"])))
    if "torso" in cfg["joints"]:
        print(f"  erro máx mão dir.-punho   : {r['conn_err_max']*1000:5.1f} mm")
    # o taco só passa na bola se as juntas chegarem a zero juntas; se o impacto
    # se der longe de 0 a batida é má, por muito boa que a velocidade pareça
    if np.abs(r["q_impact"]).max() > 3.0:
        print("  AVISO: impacto longe de q = 0 — batida má. Provavelmente os "
              "motores saturaram e a trajetória deixou de ser seguida.")


if __name__ == "__main__":
    main()
