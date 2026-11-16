"""Ensaios no robô com a IMU na pinça: cada um é um plano de juntas a 100 Hz.

Os movimentos definidos pela orientação da IMU (poses, inclinações) rodam a
IMU em torno dela própria, para não lhe somar acelerações. Os de posição
(vaivém, deriva, acenar) mantêm a orientação.

    poses       18 orientações paradas (6 faces + 12 ao acaso): calibração do
                acelerómetro no robô e rotação entre o chip e o flange (B, B2)
    rotacoes    J6 e J5 a velocidade constante, 30/90/140 °/s: escala e
                desalinhamento do giroscópio (C)
    inclinacoes rolamento e arfagem de 0 a 90° em passos de 15° (D)
    abanar      paragens a 30° e 60° de arfagem com abanões laterais de 2 Hz (D2)
    vaivem      vaivém de ±50 mm de lado e na vertical, a 0,1 e 0,2 g (D3)
    deriva      soco de 300 mm e um quadrado, parado no início e no fim (G)
    acenar      rotação de ±30° em torno de um "cotovelo" 150 mm abaixo (F1)
"""
from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as Rot

import geometria as G

DT = 0.01                                   # s, período de envio (como no Lab 2)
P0 = np.array([350.0, 0.0, 300.0])          # mm, posição de partida da IMU (a de mais folga com a IMU de nível)
R_BAIXO = G.fk(np.zeros(6))[:3, :3]         # flange a apontar para baixo (roll 180)
R_IMU_BAIXO = R_BAIXO @ G.R_F_IMU           # IMU com a pinça para baixo
# IMU de nível (z para cima) com a pinça na horizontal a apontar para a frente (+x)
R_IMU_NIVEL = np.array([[0.0, 0, 1], [1, 0, 0], [0, 1, 0]]) @ G.R_F_IMU
_MJ = None


def _mj():
    global _MJ
    if _MJ is None:
        _MJ = G.modelo()
    return _MJ


def folga_pose(q):
    """Menor folga (mm) no MuJoCo numa só pose: robô-mesa e robô-robô."""
    m, d = _mj()
    mesa, proprio, _ = G.folgas(m, d, [q], passo=1)
    return float(np.nanmin(np.r_[mesa, proprio, 1e3]))


def minjerk(s):
    s = np.clip(s, 0, 1)
    return 10 * s**3 - 15 * s**4 + 6 * s**5


class Plano:
    """Juntas ao longo do tempo, construídas por segmentos."""

    def __init__(self, nome, q0):
        self.nome = nome
        self.q = [np.asarray(q0, float)]
        self.etiquetas = []                 # (t0, t1, texto)

    @property
    def t(self):
        return np.arange(len(self.q)) * DT

    @property
    def atual(self):
        return self.q[-1]

    def _etiqueta(self, n, texto):
        if texto:
            t1 = (len(self.q) - 1) * DT
            self.etiquetas.append((t1 - n * DT, t1, texto))

    def parar(self, dur, texto=""):
        n = int(round(dur / DT))
        self.q += [self.atual.copy()] * n
        self._etiqueta(n, texto)
        return self

    def juntas(self, q_alvo, vmax=30.0, texto=""):
        """Movimento suave nas juntas; duração pela junta que mais anda."""
        dq = np.asarray(q_alvo) - self.atual
        dur = max(1.875 * np.abs(dq).max() / vmax, 0.5)
        n = int(round(dur / DT))
        q0 = self.atual
        for k in range(1, n + 1):
            self.q.append(q0 + dq * minjerk(k / n))
        self._etiqueta(n, texto)
        return self

    def junta_constante(self, j, ang, vel, acc=400.0, texto=""):
        """Junta j anda `ang` graus com velocidade constante `vel` (rampas `acc`)."""
        sinal = np.sign(ang)
        ang, t_a = abs(ang), vel / acc
        d_a = 0.5 * acc * t_a**2
        if 2 * d_a > ang:
            raise ValueError("ângulo curto demais para atingir a velocidade")
        T = 2 * t_a + (ang - 2 * d_a) / vel
        tt = np.arange(1, int(np.ceil(T / DT)) + 1) * DT
        tt[-1] = min(tt[-1], T)
        s = np.where(tt < t_a, 0.5 * acc * tt**2,
             np.where(tt < T - t_a, d_a + vel * (tt - t_a), ang - 0.5 * acc * (T - tt)**2))
        q0 = self.atual
        for v in s:
            q = q0.copy()
            q[j] += sinal * v
            self.q.append(q)
        self._etiqueta(len(tt), texto)
        return self

    def imu(self, R_de_t, p_de_t, dur, texto=""):
        """Segue uma pose da IMU dada por funções de s ∈ [0, 1] (IK amostra a amostra)."""
        n = int(round(dur / DT))
        for k in range(1, n + 1):
            s = k / n
            q, ok = G.ik_imu(R_de_t(s), p_de_t(s), self.atual)
            if not ok:
                raise RuntimeError(f"{self.nome}: IK não convergiu em t = {len(self.q) * DT:.2f} s ({texto})")
            self.q.append(q)
        self._etiqueta(n, texto)
        return self

    def rodar_imu_para(self, R_alvo, vmax=30.0, texto=""):
        """Roda a IMU até R_alvo, parada no sítio, pelo caminho mais curto."""
        R0, p0 = G.fk_imu(self.atual)
        rv = (Rot.from_matrix(R_alvo) * Rot.from_matrix(R0).inv()).as_rotvec()
        ang = np.degrees(np.linalg.norm(rv))
        dur = max(1.875 * ang / vmax, 0.5)
        return self.imu(lambda s: Rot.from_rotvec(rv * minjerk(s)).as_matrix() @ R0,
                        lambda s: p0, dur, texto)

    def resultado(self):
        return {"nome": self.nome, "t": self.t, "q": np.array(self.q), "etiquetas": self.etiquetas}


def ik_robusta(R_imu, p_imu, q_perto, margem=8.0):
    """IK com vários pontos de partida; devolve a solução válida mais perto de
    q_perto (ou None). Válida = convergiu e fica a `margem` graus dos limites."""
    lim = np.array(G.C2.JOINT_LIMITS)
    q_perto = np.asarray(q_perto, float)
    sementes = [q_perto, q_perto + [0, 0, 0, 180, 0, -180], q_perto + [0, 0, 0, -180, 0, 180]]
    w = q_perto.copy(); w[4] = -w[4]; w[3] += 180; w[5] -= 180
    sementes += [w, np.array([0, 10, 40, 0, 30, 0.0]), np.array([0, 10, 40, 90, 60, -90.0]),
                 np.array([0, 10, 40, -90, 60, 90.0]), np.array([0, 20, 60, 0, -40, 0.0])]
    melhor = None
    for sem in sementes:
        q, ok = G.ik_imu(R_imu, p_imu, sem)
        if not ok:
            continue
        q[3:] = (q[3:] + 180) % 360 - 180 + 360 * np.round((q_perto[3:] - ((q[3:] + 180) % 360 - 180)) / 360)
        if np.any(q - lim[:, 0] < margem) or np.any(lim[:, 1] - q < margem):
            continue
        if melhor is None or np.abs(q - q_perto).max() < np.abs(melhor - q_perto).max():
            melhor = q
    return melhor


def q_inicial(R_imu, p_imu=None):
    """Juntas para uma pose da IMU: várias sementes, longe dos limites (15°),
    e a de maior folga no MuJoCo."""
    p_imu = P0 if p_imu is None else p_imu
    melhor, f_melhor = None, -np.inf
    for sem in ([0, 10, 40, 0, 30, 0], [0, 30, 90, 0, 60, 0], [0, 30, 90, 0, -60, 0],
                [0, 0, 60, 180, 60, 180], [0, 20, 70, 0, 90, 90], [0, 20, 70, 0, -90, -90]):
        q = ik_robusta(R_imu, p_imu, np.array(sem, float), margem=15.0)
        if q is not None and folga_pose(q) > f_melhor:
            melhor, f_melhor = q, folga_pose(q)
    if melhor is None:
        raise RuntimeError("pose inicial sem solução")
    return melhor


# =================================================================== ensaios
def melhor_guinada(R, q_perto, folga_min=20.0):
    """Para uma orientação da IMU, procura a rotação em torno da vertical (que
    não muda a gravidade vista pela IMU) com solução e mais folga."""
    melhor = (None, -np.inf)
    for yaw in range(0, 360, 30):
        Rz = Rot.from_euler("z", yaw, degrees=True).as_matrix()
        q = ik_robusta(Rz @ R, P0, q_perto)
        if q is None:
            continue
        f = folga_pose(q)
        # entre as que têm folga, prefere a que obriga a mexer menos
        nota = f - 0.2 * np.abs(q - q_perto).max() if f >= folga_min else -np.inf
        if nota > melhor[1]:
            melhor = (q, nota)
    return melhor[0]


def poses(n_acaso=12, espera=8.0, semente=4):
    from simular import faces
    q0 = q_inicial(R_IMU_BAIXO)
    pl = Plano("poses", q0).parar(3, "parado")
    alvos = [(f"face {k}", R.as_matrix()) for k, R in faces().items()]
    # orientações ao acaso, com a pinça a não apontar para cima, bem espalhadas
    rng = np.random.default_rng(semente)
    cand = [R.as_matrix() for R in Rot.random(400, random_state=rng)]
    cand = [R for R in cand if (R @ G.R_F_IMU.T)[2, 2] < 0.2]      # z da ferramenta
    escolhidas = []
    for _ in range(n_acaso):
        g = lambda R: R.T @ [0, 0, 1]                                # gravidade vista pela IMU
        usadas = [g(R) for _, R in alvos] + [g(R) for R in escolhidas]
        melhor = max(cand, key=lambda R: min(np.arccos(np.clip(np.dot(g(R), u), -1, 1)) for u in usadas))
        escolhidas.append(melhor)
    alvos += [(f"acaso {i + 1}", R) for i, R in enumerate(escolhidas)]
    falhas = []
    for nome, R in alvos:
        guarda = len(pl.q), len(pl.etiquetas)
        try:
            q = melhor_guinada(R, pl.atual)
            if q is None:
                raise RuntimeError("sem solução com folga")
            pl.juntas(q, vmax=30, texto=f"→ {nome}").parar(espera, nome)
        except RuntimeError as e:
            del pl.q[guarda[0]:], pl.etiquetas[guarda[1]:]
            falhas.append(f"{nome} ({e})")
    pl.juntas(q0, texto="volta").parar(3, "parado")
    r = pl.resultado()
    r["falhas"] = falhas
    return r


def rotacoes(vels=(30, 90, 140), espera=3.0):
    q0 = q_inicial(R_IMU_BAIXO)
    pl = Plano("rotacoes", q0).parar(3, "parado")
    s6 = -np.sign(q0[5]) or 1.0          # J6 roda para o lado do zero: fica dentro de ±360°
    for v in vels:
        pl.junta_constante(5, 270 * s6, v, texto=f"J6 {270 * s6:+.0f}° a {v} °/s").parar(espera)
        pl.junta_constante(5, -270 * s6, v, texto=f"J6 volta a {v} °/s").parar(espera)
    for rodar6 in (0, 90):
        if rodar6:
            pl.juntas(pl.atual + [0, 0, 0, 0, 0, rodar6], texto="J6 +90°").parar(espera)
        for v in vels:
            # J5 roda para o lado que afasta a pinça do antebraço (|q5| a descer)
            s5 = -np.sign(pl.atual[4]) or 1.0
            pl.junta_constante(4, 45 * s5, min(v, 120), texto=f"J5 {45 * s5:+.0f}° a {min(v, 120)} °/s").parar(espera)
            pl.junta_constante(4, -45 * s5, min(v, 120), texto=f"J5 volta a {min(v, 120)} °/s").parar(espera)
    pl.juntas(q0, texto="volta").parar(3, "parado")
    return pl.resultado()


def _rodar_no_sitio(pl, eixo_imu, graus, vmax=20.0, texto=""):
    """Roda a IMU em torno de um dos seus eixos, sem sair do sítio."""
    R0, p0 = G.fk_imu(pl.atual)
    u = np.asarray(eixo_imu, float)
    dur = max(1.875 * abs(graus) / vmax, 0.5)
    return pl.imu(lambda s: R0 @ Rot.from_rotvec(u * np.radians(graus) * minjerk(s)).as_matrix(),
                  lambda s: p0, dur, texto)


def inclinacoes(passo=15, n=6, espera=5.0, guinada=30.0):
    # De nível, mas rodada 30° em torno da vertical: com a pinça a apontar a
    # direito para a frente, o rolamento leva a junta 5 a 0 (pulso singular).
    q0 = q_inicial(Rot.from_euler("z", guinada, degrees=True).as_matrix() @ R_IMU_NIVEL)
    pl = Plano("inclinacoes", q0).parar(5, "nível")
    for eixo, nome in (([1, 0, 0], "rolamento"), ([0, 1, 0], "arfagem")):
        for k in range(1, n + 1):
            _rodar_no_sitio(pl, eixo, passo, texto=f"{nome} → {k * passo}°").parar(espera, f"{nome} {k * passo}°")
        _rodar_no_sitio(pl, eixo, -passo * n, vmax=30, texto=f"{nome} → 0°").parar(espera, "nível")
    return pl.resultado()


def abanar(amp=10.0, f=2.0, espera=10.0):
    q0 = q_inicial(R_IMU_NIVEL)
    pl = Plano("abanar", q0).parar(5, "nível")
    # em arfagem (eixo y da IMU ≈ eixo da pinça): a junta 5 fica longe de 0, onde
    # o pulso é singular e os abanões pediriam velocidades altas às juntas 4 e 6
    for k, ang in enumerate((30, 30)):
        _rodar_no_sitio(pl, [0, 1, 0], ang, texto=f"arfagem {30 * (k + 1)}°").parar(2)
        R0, p0 = G.fk_imu(pl.atual)
        env = lambda s: np.sin(np.pi * s) ** 2
        pl.imu(lambda s: R0, lambda s: p0 + [0, amp * env(s) * np.sin(2 * np.pi * f * s * espera), 0],
               espera, texto=f"abanões ±{amp:.0f} mm a {f:.0f} Hz").parar(3)
    _rodar_no_sitio(pl, [0, 1, 0], -60, vmax=30, texto="volta").parar(3, "nível")
    return pl.resultado()


def _vaivem(pl, eixo, amp, f, ciclos, texto):
    R0, p0 = G.fk_imu(pl.atual)
    dur = ciclos / f
    # envolvente suave: sobe no 1.º ciclo, desce no último (começa e acaba parado)
    def env(s):
        x = s * ciclos
        return minjerk(np.clip(x, 0, 1)) * minjerk(np.clip(ciclos - x, 0, 1))
    u = np.asarray(eixo, float)
    return pl.imu(lambda s: R0, lambda s: p0 + u * amp * env(s) * np.sin(2 * np.pi * ciclos * s), dur, texto)


def vaivem():
    """0,1 g (±50 mm a 0,7 Hz) e 0,2 g (±50 mm a 1 Hz), de lado e na vertical.
    A aceleração é A·ω² e a velocidade A·ω: com 50 mm fica abaixo de 320 mm/s."""
    q0 = q_inicial(R_IMU_NIVEL)
    pl = Plano("vaivem", q0).parar(5, "parado")
    for eixo, nome in (([0, 1, 0], "horizontal"), ([0, 0, 1], "vertical")):
        for amp, f in ((50.0, 0.7), (50.0, 1.0)):
            a = amp * (2 * np.pi * f) ** 2 / 9806.65
            _vaivem(pl, eixo, amp, f, 6, f"{nome} ±{amp:.0f} mm a {f} Hz ({a:.2f} g)").parar(4, "parado")
    return pl.resultado()


def deriva():
    p_ini = np.array([280.0, -150.0, 240.0])
    q0 = q_inicial(R_IMU_NIVEL, p_ini)
    pl = Plano("deriva", q0).parar(4, "parado")
    R0, _ = G.fk_imu(q0)
    def reta(a, b, dur, texto):
        a, b = np.asarray(a, float), np.asarray(b, float)
        return pl.imu(lambda s: R0, lambda s: a + (b - a) * minjerk(s), dur, texto)
    reta(p_ini, p_ini + [0, 300, 0], 2.0, "soco +300 mm").parar(1.0)
    reta(p_ini + [0, 300, 0], p_ini, 2.0, "volta").parar(4, "parado")
    cantos = [p_ini, p_ini + [150, 0, 0], p_ini + [150, 200, 0], p_ini + [0, 200, 0], p_ini]
    for a, b in zip(cantos[:-1], cantos[1:]):
        reta(a, b, 1.5, "quadrado").parar(0.5)
    pl.parar(4, "parado")
    return pl.resultado()


def acenar(raio=150.0, amp=30.0, f=0.5, ciclos=5):
    q0 = q_inicial(R_IMU_NIVEL)
    pl = Plano("acenar", q0).parar(4, "parado")
    R0, p0 = G.fk_imu(q0)
    cot = p0 - [0, 0, raio]                     # "cotovelo" por baixo da IMU
    def ang(s):
        x = s * ciclos
        return np.radians(amp) * minjerk(np.clip(x, 0, 1)) * minjerk(np.clip(ciclos - x, 0, 1)) \
            * np.sin(2 * np.pi * ciclos * s)
    rot = lambda s: Rot.from_rotvec([ang(s), 0, 0]).as_matrix()
    pl.imu(lambda s: rot(s) @ R0, lambda s: cot + rot(s) @ (p0 - cot), ciclos / f,
           f"acenar ±{amp:.0f}° a {f} Hz, raio {raio:.0f} mm").parar(4, "parado")
    return pl.resultado()


ENSAIOS = {"poses": poses, "rotacoes": rotacoes, "inclinacoes": inclinacoes, "abanar": abanar,
           "vaivem": vaivem, "deriva": deriva, "acenar": acenar}
