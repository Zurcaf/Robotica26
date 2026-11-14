"""Simulador da IMU: movimento conhecido -> leituras com erros conhecidos.

Escreve no mesmo formato do gravador (data/sim/<cenário>.csv) e, ao lado, a
verdade (<cenário>_verdade.npz: t, quaterniões, posição, parâmetros). Serve
para testar a análise sabendo a resposta certa.

    .venv/bin/python src/simular.py               # todos os cenários
    .venv/bin/python src/simular.py seis_posicoes acenar

Modelo (referencial do sensor s, mundo w com z para cima):
    acc  = M_a · Rᵀ(p̈ + g ẑ)/g + b_a(T) + ruído          [g]
    gyro = M_g · ω + b_g(T) + passeio aleatório + ruído    [dps]
com M = (I + desalinhamento) · diag(escala).

Os valores por omissão dos erros são escolhas nossas, da ordem de grandeza de
uma IMU MEMS barata; não são os da ficha do ICM-20600.
"""
from __future__ import annotations

import json
import sys
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as Rot

sys.path.insert(0, str(Path(__file__).resolve().parent))
from imu_io import DADOS, G, escrever

FS_INT = 2000      # Hz, taxa interna da simulação
FS = 200           # Hz, taxa de saída (como o sketch)
META = {"acc_fs_g": 4, "gyro_fs_dps": 500, "rate_hz": FS}


# =============================================================== parâmetros
@dataclass
class ParametrosIMU:
    bias_a: list = field(default_factory=lambda: [0.025, -0.015, 0.040])   # g
    escala_a: list = field(default_factory=lambda: [1.012, 0.991, 1.006])
    desal_a: list = field(default_factory=lambda: [0.004, -0.003, 0.002])  # rad
    ruido_a: float = 120e-6        # g/√Hz
    temp_a: list = field(default_factory=lambda: [0.2e-3, -0.1e-3, 0.3e-3])  # g/°C
    bias_g: list = field(default_factory=lambda: [0.62, -0.35, 0.18])     # dps
    escala_g: list = field(default_factory=lambda: [1.010, 0.985, 1.004])
    desal_g: list = field(default_factory=lambda: [0.006, 0.004, -0.005])  # rad
    ruido_g: float = 0.006         # dps/√Hz
    passeio_g: float = 0.002       # dps/√s (deriva lenta do desvio)
    temp_g: list = field(default_factory=lambda: [0.020, -0.015, 0.010])  # dps/°C
    t_amb: float = 21.0            # °C
    t_final: float = 28.0          # °C, depois de aquecer
    tau_aquecer: float = 300.0     # s
    semente: int = 1

    @staticmethod
    def ideal():
        p = ParametrosIMU(bias_a=[0, 0, 0], escala_a=[1, 1, 1], desal_a=[0, 0, 0], ruido_a=0,
                          temp_a=[0, 0, 0], bias_g=[0, 0, 0], escala_g=[1, 1, 1],
                          desal_g=[0, 0, 0], ruido_g=0, passeio_g=0, temp_g=[0, 0, 0])
        p.t_final = p.t_amb
        return p


def matriz(escala, desal):
    """M = D · diag(escala). D triangular: eixos não perpendiculares (Tedaldi
    2014). Uma rotação pura não entra aqui: é a montagem, que só o robô vê."""
    a, b, c = desal
    D = np.array([[1, -a, b], [0, 1, -c], [0, 0, 1]])
    return D @ np.diag(escala)


# =============================================================== movimentos
def minjerk(n):
    s = np.linspace(0, 1, n)
    return 10 * s**3 - 15 * s**4 + 6 * s**5


class Movimento:
    """Constrói a orientação por segmentos de rotação no referencial do sensor.

    rodar(eixo, graus, dur): roda em torno de um eixo do sensor, perfil suave.
    rodar_constante(eixo, graus_s, dur): velocidade constante, rampas suaves.
    parar(dur): fica parado.
    ir_para(R, dur): roda pelo caminho mais curto até à orientação R.
    """

    def __init__(self, R0: Rot = Rot.identity()):
        self.R = [Rot.concatenate([R0])]
        self.w = []                    # ω no referencial do sensor, rad/s

    @property
    def atual(self) -> Rot:
        return self.R[-1][-1]

    def _perfil(self, eixo, th):
        """th: ângulo acumulado (rad) nas amostras novas, a começar em 0."""
        eixo = np.asarray(eixo, float)
        nrm = np.linalg.norm(eixo)
        eixo = eixo / nrm if nrm > 0 else np.zeros(3)
        R = self.atual * Rot.from_rotvec(np.outer(th[1:], eixo))
        self.R.append(R)
        self.w.append(np.outer(np.diff(th) * FS_INT, eixo))
        return self

    def rodar(self, eixo, graus, dur):
        n = max(int(round(dur * FS_INT)), 1)
        return self._perfil(eixo, np.radians(graus) * minjerk(n + 1))

    def rodar_constante(self, eixo, graus_s, dur, rampa=0.5):
        n = int(round(dur * FS_INT))
        nr = int(round(rampa * FS_INT))
        v = np.ones(n)
        v[:nr] = minjerk(nr)
        v[-nr:] = minjerk(nr)[::-1]
        th = np.concatenate([[0], np.cumsum(v) * np.radians(graus_s) / FS_INT])
        return self._perfil(eixo, th)

    def parar(self, dur):
        n = max(int(round(dur * FS_INT)), 1)
        return self._perfil([0, 0, 0], np.zeros(n + 1))

    def ir_para(self, alvo: Rot, dur):
        rv = (self.atual.inv() * alvo).as_rotvec()
        ang = np.linalg.norm(rv)
        return self.rodar(rv / ang if ang > 0 else [0, 0, 0], np.degrees(ang), dur)

    def resultado(self):
        """t (N,), R (Rotation, N), ω (N,3) rad/s, à taxa interna."""
        w = np.vstack(self.w + [np.zeros((1, 3))])
        R = Rot.concatenate(self.R)
        t = np.arange(len(w)) / FS_INT
        return t, R, w


# =============================================================== simulação
def simular(t, R: Rot, w_s, p_w=None, par: ParametrosIMU | None = None):
    """Leituras da IMU a FS Hz a partir do movimento à taxa interna."""
    par = par or ParametrosIMU()
    rng = np.random.default_rng(par.semente)
    n = len(t)
    if p_w is None:
        p_w = np.zeros((n, 3))
    acc_w = np.gradient(np.gradient(p_w, axis=0), axis=0) * FS_INT**2      # m/s²
    f_s = R.inv().apply(acc_w + np.array([0, 0, G])) / G                  # g
    T = par.t_final + (par.t_amb - par.t_final) * np.exp(-t / par.tau_aquecer)

    Ma = matriz(par.escala_a, par.desal_a)
    Mg = matriz(par.escala_g, par.desal_g)
    passeio = np.cumsum(rng.normal(0, par.passeio_g / np.sqrt(FS_INT), (n, 3)), axis=0)
    acc = f_s @ Ma.T + np.array(par.bias_a) + np.outer(T - 25, par.temp_a)
    gyro = np.degrees(w_s) @ Mg.T + np.array(par.bias_g) + np.outer(T - 25, par.temp_g) + passeio

    # decimação: média de blocos (faz de filtro anti-aliasing do chip)
    k = FS_INT // FS
    m = (n // k) * k
    dec = lambda x: x[:m].reshape(-1, k, *x.shape[1:]).mean(axis=1)
    t_o, acc_o, gyro_o, T_o = dec(t), dec(acc), dec(gyro), dec(T)
    acc_o += rng.normal(0, par.ruido_a * np.sqrt(FS), acc_o.shape)    # σ(τ=1 s) = densidade
    gyro_o += rng.normal(0, par.ruido_g * np.sqrt(FS), gyro_o.shape)
    idx = np.arange(0, m, k) + k // 2
    return t_o - t_o[0], acc_o, gyro_o, T_o, R[idx], p_w[idx], np.degrees(w_s[idx])


# =============================================================== cenários
def faces():
    """Seis orientações: cada eixo do sensor para cima e para baixo."""
    return {
        "+z": Rot.identity(), "-z": Rot.from_euler("x", 180, degrees=True),
        "+x": Rot.from_euler("y", -90, degrees=True), "-x": Rot.from_euler("y", 90, degrees=True),
        "+y": Rot.from_euler("x", 90, degrees=True), "-y": Rot.from_euler("x", -90, degrees=True),
    }


def cen_parado(dur=600):
    return Movimento().parar(dur).resultado(), None


def cen_seis_posicoes(espera=20, trans=3):
    m = Movimento().parar(espera)
    for nome, R in faces().items():
        if nome != "+z":
            m.ir_para(R, trans).parar(espera)
    return m.resultado(), None


def cen_posicoes_acaso(n=18, espera=10, trans=3, semente=7):
    rng = np.random.default_rng(semente)
    m = Movimento().parar(espera)
    for R in Rot.random(n, random_state=rng):
        m.ir_para(R, trans).parar(espera)
    return m.resultado(), None


def cen_rotacao_conhecida(espera=5):
    """90° e 360° em torno de cada eixo, nos dois sentidos (ensaio C)."""
    m = Movimento().parar(espera)
    for eixo in np.eye(3):
        for ang, dur in ((90, 2), (-90, 2), (360, 5), (-360, 5)):
            m.rodar(eixo, ang, dur).parar(espera)
    return m.resultado(), None


def cen_inclinacoes(espera=5, passo=2):
    """0, 15, ..., 90° em rolamento e depois em arfagem, e volta (ensaio D)."""
    m = Movimento().parar(espera)
    for eixo in ([1, 0, 0], [0, 1, 0]):
        for _ in range(6):
            m.rodar(eixo, 15, passo).parar(espera)
        m.rodar(eixo, -90, 4).parar(espera)
    return m.resultado(), None


def _abanao(t, t0, t1, amp=0.01, f=3.0, eixo=(0, 1, 0)):
    p = np.zeros((len(t), 3))
    m = (t >= t0) & (t < t1)
    env = np.sin(np.pi * (t[m] - t0) / (t1 - t0)) ** 2
    p[m] = np.outer(amp * env * np.sin(2 * np.pi * f * (t[m] - t0)), eixo)
    return p


def cen_inclinar_abanar():
    """Ensaio D com abanões laterais a meio (D2)."""
    (t, R, w), _ = cen_inclinacoes()
    return (t, R, w), _abanao(t, 25, 45, amp=0.015, f=3.0)


def cen_vaivem(amp=0.10, f=0.5, ciclos=5):
    """Vaivém horizontal e depois vertical, orientação fixa (D3)."""
    dur = ciclos / f
    (t, R, w), _ = (Movimento().parar(5 + dur + 5 + dur + 5).resultado(), None)
    p = np.zeros((len(t), 3))
    for t0, k in ((5, 0), (10 + dur, 2)):
        m = (t >= t0) & (t < t0 + dur)
        p[m, k] = amp * np.sin(2 * np.pi * f * (t[m] - t0)) * np.sin(np.pi * (t[m] - t0) / dur) ** 2
    return (t, R, w), p


def cen_acenar(raio=0.30, amp=30, f=0.5, ciclos=5):
    """Antebraço a oscilar à volta do cotovelo; sensor a `raio` m do cotovelo (F1)."""
    m = Movimento(Rot.from_euler("y", -60, degrees=True)).parar(3)
    m.rodar([0, 1, 0], amp, 1.0 / f / 2)
    for _ in range(ciclos):
        m.rodar([0, 1, 0], -2 * amp, 1.0 / f / 2).rodar([0, 1, 0], 2 * amp, 1.0 / f / 2)
    m.rodar([0, 1, 0], -amp, 1.0 / f / 2).parar(3)
    t, R, w = m.resultado()
    p = R.apply([0, 0, raio])            # sensor ao longo do antebraço
    return (t, R, w), p - p[0]


def cen_soco(dist=0.30, dur=1.0, repet=3):
    """Mão a avançar `dist` m e a voltar, parada no início e no fim (F2)."""
    pausa = 3
    T = pausa + repet * (2 * dur + 1) + pausa
    (t, R, w), _ = (Movimento().parar(T).resultado(), None)
    p = np.zeros((len(t), 3))
    t0 = pausa
    for _ in range(repet):
        for a, b, ini, fim in ((t0, t0 + dur, 0, dist), (t0 + dur + 0.5, t0 + 2 * dur + 0.5, dist, 0)):
            m = (t >= a) & (t < b)
            p[m, 0] = ini + (fim - ini) * minjerk(m.sum())
        p[(t >= t0 + dur) & (t < t0 + dur + 0.5), 0] = dist
        t0 += 2 * dur + 1
    return (t, R, w), p


CENARIOS = {
    "parado": cen_parado,
    "seis_posicoes": cen_seis_posicoes,
    "posicoes_acaso": cen_posicoes_acaso,
    "rotacao_conhecida": cen_rotacao_conhecida,
    "inclinacoes": cen_inclinacoes,
    "inclinar_abanar": cen_inclinar_abanar,
    "vaivem": cen_vaivem,
    "acenar": cen_acenar,
    "soco": cen_soco,
}


def gerar(nome, par: ParametrosIMU | None = None, pasta=DADOS / "sim"):
    par = par or ParametrosIMU()
    (t, R, w), p = CENARIOS[nome]()
    t_o, acc, gyro, T, R_o, p_o, w_o = simular(t, R, w, p, par)
    csv = escrever(Path(pasta) / f"{nome}.csv", t_o, acc, gyro, T, META,
                   comentarios=["simulado (src/simular.py)", f"cenario={nome}"])
    np.savez(Path(pasta) / f"{nome}_verdade.npz", t=t_o, q=R_o.as_quat(), p=p_o, w_dps=w_o,
             parametros=json.dumps(asdict(par)))
    return csv


def ler_verdade(csv):
    v = np.load(Path(csv).with_name(Path(csv).stem + "_verdade.npz"))
    return {"t": v["t"], "R": Rot.from_quat(v["q"]), "p": v["p"], "w_dps": v["w_dps"],
            "par": ParametrosIMU(**json.loads(str(v["parametros"])))}


if __name__ == "__main__":
    nomes = sys.argv[1:] or list(CENARIOS)
    for n in nomes:
        print(gerar(n))
