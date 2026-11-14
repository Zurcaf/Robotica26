"""Verdade do robô para a IMU: juntas medidas -> orientação e posição da IMU,
no instante de cada amostra da IMU.

Sincronização, em dois passos:
1. o relógio do Arduino (t, exato mas com origem própria) passa para a hora
   do computador por uma reta ajustada ao contorno inferior de t_host − t (a
   USB só atrasa as linhas, nunca as adianta);
2. um desfasamento fino entre o robô e a IMU é o que melhor sobrepõe |ω| do
   robô e |ω| do giroscópio (correlação cruzada, ±0,5 s).

    from robo.verdade import verdade_para_imu
    q_ver, p_ver, info = verdade_para_imu(registo_imu, "data/..._robo.csv")
"""
from __future__ import annotations

import ast
import re
import sys
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation as Rot
from scipy.spatial.transform import Slerp

sys.path.insert(0, str(Path(__file__).resolve().parent))
import geometria as G   # noqa: E402


def ler_robo(caminho):
    meta = {}
    with open(caminho) as f:
        for ln in f:
            if not ln.startswith("#"):
                break
            for k, v in re.findall(r"(\w+)=(\[[^\]]*\]|\S+)", ln):
                meta[k] = v
    x = np.loadtxt(caminho, delimiter=",", comments="#")
    R_F = np.array(ast.literal_eval(meta["R_F_IMU"])).reshape(3, 3) if "R_F_IMU" in meta else G.R_F_IMU
    P_F = np.array(ast.literal_eval(meta["P_F_IMU"])) if "P_F_IMU" in meta else G.P_F_IMU
    return {"t_host": x[:, 0], "q_ref": x[:, 1:7], "q": x[:, 7:13], "R_F_IMU": R_F, "P_F_IMU": P_F, "meta": meta}


def pose_imu(q, R_F_IMU, P_F_IMU):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R_F_IMU, P_F_IMU
    Ts = np.array([G.fk(qk) @ T for qk in q])
    return Rot.from_matrix(Ts[:, :3, :3]), Ts[:, :3, 3]


def hora_imu(reg):
    """Hora do computador de cada amostra da IMU, a partir do relógio do Arduino."""
    t, th = reg.t, reg.t_host
    d = th - t
    # contorno inferior: mínimo em janelas de 2 s, depois reta
    n = max(int(2 * reg.fs), 1)
    idx = [i + np.argmin(d[i:i + n]) for i in range(0, len(d) - n + 1, n)] or [int(np.argmin(d))]
    a, b = np.polyfit(t[idx], d[idx], 1) if len(idx) > 1 else (0.0, d[idx[0]])
    return t + a * t + b


def _w_norma(R, t):
    w = (R[:-1].inv() * R[1:]).as_rotvec() / np.diff(t)[:, None]
    return np.r_[np.linalg.norm(np.degrees(w), axis=1), 0.0]


def verdade_para_imu(reg, robo_csv, janela=0.5):
    rob = ler_robo(robo_csv)
    R, p = pose_imu(rob["q"], rob["R_F_IMU"], rob["P_F_IMU"])
    tr = rob["t_host"]
    ti = hora_imu(reg)
    # desfasamento fino: correlação de |ω| numa grelha comum a 200 Hz
    grelha = np.arange(max(tr[0], ti[0]), min(tr[-1], ti[-1]), 0.005)
    wr = np.interp(grelha, tr, _w_norma(R, tr))
    wi = np.interp(grelha, ti, np.linalg.norm(reg.gyro, axis=1))
    wr, wi = wr - wr.mean(), wi - wi.mean()
    lags = np.arange(-int(janela / 0.005), int(janela / 0.005) + 1)
    cc = [np.dot(wi[max(0, -k):len(wi) - max(0, k)], wr[max(0, k):len(wr) - max(0, -k)]) for k in lags]
    desf = lags[int(np.argmax(cc))] * 0.005          # s: o robô vê o mesmo instante `desf` s depois
    tq = np.clip(ti + desf, tr[0], tr[-1])
    _, uniq = np.unique(tr, return_index=True)
    Rq = Slerp(tr[uniq], R[uniq])(tq)
    pq = np.column_stack([np.interp(tq, tr, p[:, j]) for j in range(3)])
    fora = (ti + desf < tr[0]) | (ti + desf > tr[-1])
    return Rq.as_quat(scalar_first=True), pq, {"desfasamento_s": desf, "fora": int(fora.sum())}
