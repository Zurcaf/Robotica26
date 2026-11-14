"""Calibração da IMU (Tarefa 2): funções usadas pelos scripts t2_*.

Modelo:  medido = M · verdadeiro + b,  M = D · diag(s), D triangular superior
(eixos não perpendiculares). O acelerómetro calibra-se com a gravidade
(|a| = 1 g parado); o giroscópio com o desvio parado e rotações conhecidas.
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares


# ================================================================ troços parados
def _std_movel(x, n):
    """Desvio-padrão numa janela centrada de n amostras (por coluna)."""
    x = np.asarray(x, float)
    c1 = np.cumsum(np.vstack([np.zeros((1, x.shape[1])), x]), axis=0)
    c2 = np.cumsum(np.vstack([np.zeros((1, x.shape[1])), x**2]), axis=0)
    i = np.arange(len(x))
    a = np.clip(i - n // 2, 0, len(x))
    b = np.clip(i + n // 2 + 1, 0, len(x))
    k = (b - a)[:, None]
    m = (c1[b] - c1[a]) / k
    return np.sqrt(np.maximum((c2[b] - c2[a]) / k - m**2, 0))


def troços_parados(reg, janela=0.5, lim_w=0.3, lim_a=0.005, dur_min=2.0, margem=0.3):
    """Intervalos [i0, i1) em que o sensor está parado.

    Parado = em janelas de `janela` s, desvio-padrão de cada eixo do
    giroscópio < lim_w dps e do acelerómetro < lim_a g. Corta `margem` s em
    cada ponta e guarda só os troços com mais de `dur_min` s.
    """
    n = max(int(janela * reg.fs), 3)
    parado = (_std_movel(reg.gyro, n).max(1) < lim_w) & (_std_movel(reg.acc, n).max(1) < lim_a)
    d = np.diff(np.concatenate([[0], parado.astype(int), [0]]))
    ini, fim = np.flatnonzero(d == 1), np.flatnonzero(d == -1)
    m = int(margem * reg.fs)
    return [(a + m, b - m) for a, b in zip(ini, fim) if (b - a - 2 * m) >= dur_min * reg.fs]


def medias(reg, troços):
    acc = np.array([reg.acc[a:b].mean(0) for a, b in troços])
    gyro = np.array([reg.gyro[a:b].mean(0) for a, b in troços])
    temp = np.array([reg.temp[a:b].mean() for a, b in troços])
    return acc, gyro, temp


# ================================================================ modelo
def matriz(s, d=(0, 0, 0)):
    a, b, c = d
    return np.array([[1, -a, b], [0, 1, -c], [0, 0, 1]]) @ np.diag(s)


@dataclass
class Calibracao:
    bias_a: list = field(default_factory=lambda: [0.0, 0.0, 0.0])   # g
    M_a: list = field(default_factory=lambda: np.eye(3).tolist())
    bias_g: list = field(default_factory=lambda: [0.0, 0.0, 0.0])   # dps
    M_g: list = field(default_factory=lambda: np.eye(3).tolist())
    temp_ref: float = 25.0
    temp_g: list = field(default_factory=lambda: [0.0, 0.0, 0.0])   # dps/°C
    notas: dict = field(default_factory=dict)

    def corrigir_acc(self, acc):
        return (np.asarray(acc) - self.bias_a) @ np.linalg.inv(self.M_a).T

    def corrigir_gyro(self, gyro, temp=None):
        b = np.asarray(self.bias_g, float)
        if temp is not None:
            b = b + np.outer(np.asarray(temp) - self.temp_ref, self.temp_g)
        return (np.asarray(gyro) - b) @ np.linalg.inv(self.M_g).T

    def aplicar(self, reg):
        """Devolve uma cópia do registo com acc e gyro corrigidos."""
        from copy import copy
        r = copy(reg)
        r.acc = self.corrigir_acc(reg.acc)
        r.gyro = self.corrigir_gyro(reg.gyro, reg.temp)
        return r

    def guardar(self, caminho):
        Path(caminho).parent.mkdir(parents=True, exist_ok=True)
        Path(caminho).write_text(json.dumps(asdict(self), indent=2, default=float))

    @staticmethod
    def carregar(caminho):
        return Calibracao(**json.loads(Path(caminho).read_text()))


# ================================================================ acelerómetro
def seis_posicoes(acc_med):
    """Método das seis posições.

    Cada média é atribuída à face cujo eixo tem a maior componente. Para cada
    eixo: desvio = (cima + baixo)/2, escala = (cima − baixo)/2 (em g).
    Também ajusta M completa e b por mínimos quadrados (12 incógnitas).
    """
    acc_med = np.asarray(acc_med)
    k = np.abs(acc_med).argmax(1)
    sinal = np.sign(acc_med[np.arange(len(k)), k])
    b, s = np.zeros(3), np.zeros(3)
    for e in range(3):
        cima = acc_med[(k == e) & (sinal > 0), e]
        baixo = acc_med[(k == e) & (sinal < 0), e]
        if not len(cima) or not len(baixo):
            raise ValueError(f"falta a face {'+' if not len(cima) else '-'}{'xyz'[e]}")
        b[e] = (cima.mean() + baixo.mean()) / 2
        s[e] = (cima.mean() - baixo.mean()) / 2
    U = np.zeros_like(acc_med)
    U[np.arange(len(k)), k] = sinal                        # gravidade ideal em cada face
    X, *_ = np.linalg.lstsq(np.column_stack([U, np.ones(len(U))]), acc_med, rcond=None)
    return {"bias": b, "escala": s, "M_ls": X[:3].T, "bias_ls": X[3]}


def elipsoide(acc_med, desalinhamento=True):
    """Posições ao acaso: b, s (e D) que fazem |M⁻¹(a − b)| = 1 em todas.

    Precisa de 6 posições (sem D) ou 9 (com D), bem espalhadas.
    """
    acc_med = np.asarray(acc_med)
    n_par = 9 if desalinhamento else 6

    def res(x):
        M = matriz(x[3:6], x[6:9] if desalinhamento else (0, 0, 0))
        a = (acc_med - x[:3]) @ np.linalg.inv(M).T
        return np.linalg.norm(a, axis=1) - 1

    x0 = np.r_[np.zeros(3), np.ones(3), np.zeros(n_par - 6)]
    sol = least_squares(res, x0)
    J = sol.jac
    cov = np.linalg.pinv(J.T @ J) * (sol.fun @ sol.fun) / max(len(acc_med) - n_par, 1)
    x = sol.x
    return {"bias": x[:3], "escala": x[3:6], "desal": x[6:9] if desalinhamento else np.zeros(3),
            "M": matriz(x[3:6], x[6:9] if desalinhamento else (0, 0, 0)),
            "residuo_rms": float(np.sqrt(np.mean(sol.fun**2))), "dp": np.sqrt(np.diag(cov))}


# ================================================================ giroscópio
def bias_giro(reg, troços):
    """Desvio do giroscópio: média de todas as amostras paradas."""
    idx = np.concatenate([np.arange(a, b) for a, b in troços])
    return reg.gyro[idx].mean(0), reg.gyro[idx].std(0)


def giro_temperatura(reg, troços, janela_s=10.0):
    """Desvio do giroscópio contra a temperatura (médias em janelas de 10 s)."""
    n = int(janela_s * reg.fs)
    T, W = [], []
    for a, b in troços:
        for i in range(a, b - n + 1, n):
            T.append(reg.temp[i:i + n].mean())
            W.append(reg.gyro[i:i + n].mean(0))
    T, W = np.array(T), np.array(W)
    if len(T) < 3 or np.ptp(T) < 0.5:
        return {"T": T, "W": W, "declive": np.zeros(3), "temp_ref": float(T.mean()) if len(T) else 25.0}
    A = np.column_stack([T - T.mean(), np.ones(len(T))])
    X, *_ = np.linalg.lstsq(A, W, rcond=None)
    return {"T": T, "W": W, "declive": X[0], "ordenada": X[1], "temp_ref": float(T.mean())}


ANGULOS_NOMINAIS = np.array([90, 180, 270, 360, 720])


def rotacoes_conhecidas(reg, troços, bias):
    """Escala do giroscópio a partir de rotações de ângulo conhecido.

    Entre dois troços parados seguidos, integra-se o giroscópio (sem o
    desvio). Com rotação em torno de um eixo fixo do sensor, o integral é o
    ângulo vezes o eixo. O ângulo nominal é o múltiplo de 90° mais próximo.
    Devolve a escala por eixo e, por linha, os termos cruzados (desalinhamento).
    """
    dt = 1.0 / reg.fs
    linhas = []
    for (a0, b0), (a1, b1) in zip(troços[:-1], troços[1:]):
        th = ((reg.gyro[b0:a1] - bias) * dt).sum(0)        # graus
        k = np.abs(th).argmax()
        nom = ANGULOS_NOMINAIS[np.abs(ANGULOS_NOMINAIS - abs(th[k])).argmin()] * np.sign(th[k])
        linhas.append({"eixo": int(k), "nominal": float(nom), "medido": th})
    M = np.full((3, 3), np.nan)
    for e in range(3):
        L = [l for l in linhas if l["eixo"] == e]
        if L:
            # coluna e de M: medido / nominal (média ponderada pelo ângulo)
            num = sum(l["medido"] * l["nominal"] for l in L)
            den = sum(l["nominal"] ** 2 for l in L)
            M[:, e] = num / den
    return {"M": M, "escala": np.diag(M), "rotacoes": linhas}


# ================================================================ ruído
def allan(x, fs, n_tau=40):
    """Desvio de Allan sobreposto. x (N,) ou (N,3). Devolve tau [s], adev."""
    x = np.atleast_2d(np.asarray(x, float).T).T
    N = len(x)
    m = np.unique(np.logspace(0, np.log10(N // 3), n_tau).astype(int))
    tau = m / fs
    th = np.vstack([np.zeros((1, x.shape[1])), np.cumsum(x, axis=0) / fs])
    ad = np.empty((len(m), x.shape[1]))
    for i, mi in enumerate(m):
        d = th[2 * mi:] - 2 * th[mi:-mi] + th[:-2 * mi]
        ad[i] = np.sqrt((d**2).sum(0) / (2 * tau[i]**2 * (N - 2 * mi + 1)))
    return tau, ad


def ruido_de_allan(tau, ad):
    """Densidade de ruído branco (adev a τ = 1 s) e instabilidade do desvio
    (mínimo / 0,664), por coluna."""
    n = np.array([np.exp(np.interp(0.0, np.log(tau), np.log(ad[:, j]))) for j in range(ad.shape[1])])
    return n, ad.min(0) / 0.664
