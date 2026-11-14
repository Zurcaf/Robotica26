"""Orientação a partir da IMU (Tarefas 3 e 4).

Quaterniões de Hamilton, q = [w, x, y, z], a rodar do sensor para o mundo
(z para cima). Ângulos ZYX: R = Rz(guinada) · Ry(arfagem) · Rx(rolamento).
Implementação própria (a do scipy só se usa para a verdade, nos testes).
"""
from __future__ import annotations

import numpy as np


# ================================================================ quaterniões
def qmul(p, q):
    pw, px, py, pz = np.moveaxis(np.asarray(p), -1, 0)
    qw, qx, qy, qz = np.moveaxis(np.asarray(q), -1, 0)
    return np.stack([pw*qw - px*qx - py*qy - pz*qz,
                     pw*qx + px*qw + py*qz - pz*qy,
                     pw*qy - px*qz + py*qw + pz*qx,
                     pw*qz + px*qy - py*qx + pz*qw], axis=-1)


def qconj(q):
    q = np.asarray(q)
    return q * np.array([1, -1, -1, -1])


def qexp(rotvec):
    """Quaternião da rotação de ângulo |v| em torno de v/|v| (v em rad)."""
    v = np.asarray(rotvec, float)
    ang = np.linalg.norm(v, axis=-1, keepdims=True)
    meio = ang / 2
    with np.errstate(invalid="ignore", divide="ignore"):
        k = np.where(ang > 1e-12, np.sin(meio) / ang, 0.5 - ang**2 / 48)
    return np.concatenate([np.cos(meio), v * k], axis=-1)


def qrodar(q, v):
    """Roda o vetor v (do sensor para o mundo)."""
    v4 = np.concatenate([np.zeros(np.shape(v)[:-1] + (1,)), v], axis=-1)
    return qmul(qmul(q, v4), qconj(q))[..., 1:]


def angulo_entre(q1, q2):
    """Ângulo (graus) da rotação que leva q1 a q2."""
    d = qmul(qconj(q1), q2)
    return np.degrees(2 * np.arctan2(np.linalg.norm(d[..., 1:], axis=-1), np.abs(d[..., 0])))


def de_euler(rolamento, arfagem, guinada=0.0):
    """Graus -> quaternião (ZYX)."""
    r, p, y = (np.radians(np.asarray(a, float)) / 2 for a in (rolamento, arfagem, guinada))
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.stack([cr*cp*cy + sr*sp*sy, sr*cp*cy - cr*sp*sy,
                     cr*sp*cy + sr*cp*sy, cr*cp*sy - sr*sp*cy], axis=-1)


def para_euler(q):
    """Quaternião -> (rolamento, arfagem, guinada) em graus (ZYX)."""
    w, x, y, z = np.moveaxis(np.asarray(q), -1, 0)
    rol = np.arctan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y))
    arf = np.arcsin(np.clip(2 * (w*y - z*x), -1, 1))
    gui = np.arctan2(2 * (w*z + x*y), 1 - 2 * (y*y + z*z))
    return np.degrees(np.stack([rol, arf, gui], axis=-1))


def erro_inclinacao(q_est, q_ver):
    """Ângulo (graus) entre a vertical estimada e a verdadeira, vista no
    sensor. Ignora a guinada, que o acelerómetro não vê."""
    z = np.array([0.0, 0.0, 1.0])
    a = qrodar(qconj(q_est), np.broadcast_to(z, np.shape(q_est)[:-1] + (3,)))
    b = qrodar(qconj(q_ver), np.broadcast_to(z, np.shape(q_ver)[:-1] + (3,)))
    return np.degrees(np.arccos(np.clip((a * b).sum(-1), -1, 1)))


# ================================================================ Tarefa 3
def inclinometro(acc):
    """Rolamento e arfagem (graus) a partir do acelerómetro.

    Parado, o acelerómetro mede a gravidade no referencial do sensor:
    a = (−sin θ, sin φ cos θ, cos φ cos θ) g, donde
    φ = atan2(a_y, a_z), θ = atan2(−a_x, √(a_y² + a_z²)).
    """
    a = np.asarray(acc)
    rol = np.arctan2(a[..., 1], a[..., 2])
    arf = np.arctan2(-a[..., 0], np.hypot(a[..., 1], a[..., 2]))
    return np.degrees(rol), np.degrees(arf)


def q_inclinometro(acc):
    rol, arf = inclinometro(acc)
    return de_euler(rol, arf, 0.0)


# ================================================================ Tarefa 4
def integrar_giro(gyro_dps, t, q0):
    """q_{k+1} = q_k ⊗ exp(ω_k Δt / 2): compõe as pequenas rotações medidas.

    gyro_dps (N,3) no referencial do sensor; t (N,) em s; q0 inicial.
    Usa a média de ω_k e ω_{k+1} (trapézio) em cada passo.
    """
    w = np.radians(np.asarray(gyro_dps, float))
    dt = np.diff(np.asarray(t, float), append=t[-1])
    w_med = np.vstack([(w[:-1] + w[1:]) / 2, w[-1:]])
    dq = qexp(w_med * dt[:, None])
    q = np.empty((len(w), 4))
    q[0] = q0
    for k in range(len(w) - 1):
        qk = qmul(q[k], dq[k])
        q[k + 1] = qk / np.linalg.norm(qk)
    return q
