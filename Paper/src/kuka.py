"""KUKA KR 240-2 C 2000 (de teto) com a extrusora: parâmetros, cinemática direta e inversa.

Ângulos em graus, iguais aos da consola (A1..A6): o zero e os sentidos são os da
KUKA, e a posição de casa é {0, -90, 90, 0, 0, 0}. Distâncias em mm.

Fontes (por ordem de confiança):
- Limites das juntas: controlador ($SOFTN_END / $SOFTP_END em $machine.dat, os
  dois backups, 2016 e 2024). A ficha PF0023 dá -146° no A2; o controlador -140°.
- Cotas dos elos: ficha PF0023 da KUKA (desenho "Dimensions").
- Sinal do desvio de 55 mm entre A3 e A4: decidido pelos pontos ensinados do
  backup (validar_backup.py), não pela ficha, onde não se lê.
- Montagem: $ROBROOT = {C 180} (robô de pernas para o ar); $TRAFONAME
  "#KR2240_2 H C2 CLG ZH240" (CLG = teto).

Referencial do robô (o da KUKA): z ao longo do A1, a sair da base; x para a
frente com A1 = 0. Montado no teto, este z aponta para o chão.
"""
from __future__ import annotations

import numpy as np

# ------------------------------------------------------------------ cotas (ficha PF0023)
D1 = 750.0      # base -> eixo do A2, ao longo do A1
A1_OFF = 350.0  # eixo do A1 -> eixo do A2, na horizontal
L2 = 1250.0     # A2 -> A3 (braço)
A3_OFF = 55.0   # desvio do eixo do A4 em relação ao A3 (sinal: ver SINAL_55)
L3 = 1100.0     # A3 -> centro do punho, ao longo do antebraço
D6 = 230.0      # centro do punho -> flange
# +1: em casa, o eixo do A4 fica 55 mm ACIMA do A3 (no referencial do robô);
# -1: abaixo. Escolhido pelo teste 1 (validar_backup.py).
SINAL_55 = -1

# ------------------------------------------------------------------ limites (controlador)
LIM = np.array([[-185.0, 185.0],
                [-140.0, 0.0],      # ficha: -146°
                [-119.0, 155.0],
                [-350.0, 350.0],
                [-125.0, 125.0],
                [-350.0, 350.0]])
# Velocidades com 240 kg (ficha PF0023), °/s. Ainda não entram no estudo.
VEL = np.array([86.0, 80.0, 79.0, 100.0, 110.0, 184.0])
CASA = np.array([0.0, -90.0, 90.0, 0.0, 0.0, 0.0])

# ------------------------------------------------------------------ extrusora
# Um "pau" preso de lado ao flange, perpendicular ao eixo do flange e a passar
# pelo centro: 605 mm para um lado (motor/tremonha), 723 mm para o outro até ao
# bico. Fica ao longo do x do flange (em casa, x do flange aponta para baixo).
EXT_BICO = 723.0
EXT_TRAS = 605.0
EXT_DZ = 0.0        # afastamento do eixo do pau à face do flange (por medir)
EXT_RAIO = 75.0     # raio provisório do pau, para colisões (por medir no CAD)
# Ferramenta do bico, como na KUKA: z da ferramenta = direção do bico.
TOOL_BICO = (EXT_BICO, 0.0, EXT_DZ, 0.0, 90.0, 0.0)

# ------------------------------------------------------------------ sala (esboço de 17/11)
SALA = (4000.0, 7000.0, 3000.0)    # x (largura), y (comprimento), altura
# A1 a 1,95 m da parede da esquerda e 1,65 m da parede de cima. Mundo: origem no
# canto de baixo à esquerda do esboço, no chão; x para a direita, y para cima, z
# para cima. A parede de cima é y = 7 m.
ROBO_XY = (1950.0, 7000.0 - 1650.0)
ROBO_Z = 3000.0                      # altura do prato do robô (≈ 3 m)
ROBO_YAW = 0.0                       # rotação do robô em torno da vertical (°)
PLACA = 1300.0                       # placa da estrutura, quadrada


# ------------------------------------------------------------------ rotações
def rx(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def ry(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def frame(x, y, z, a, b, c):
    """Frame da KUKA {X,Y,Z,A,B,C} -> 4x4. R = Rz(A) Ry(B) Rx(C), em graus."""
    T = np.eye(4)
    T[:3, :3] = rz(np.radians(a)) @ ry(np.radians(b)) @ rx(np.radians(c))
    T[:3, 3] = (x, y, z)
    return T


def abc(T):
    """4x4 -> (X,Y,Z,A,B,C) da KUKA."""
    R = T[:3, :3]
    b = np.degrees(np.arcsin(-np.clip(R[2, 0], -1, 1)))
    a = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    c = np.degrees(np.arctan2(R[2, 1], R[2, 2]))
    return (*T[:3, 3], a, b, c)


# Orientação do flange com todas as juntas a zero: z do flange para a frente
# (+x do robô), x do flange para baixo (-z); é o B = 90° que a consola mostra.
R_F0 = np.array([[0.0, 0, 1], [0, 1, 0], [-1, 0, 0]])


def robo_no_mundo():
    """4x4 do referencial do robô no mundo da sala ($ROBROOT C 180 + posição)."""
    T = np.eye(4)
    T[:3, :3] = rz(np.radians(ROBO_YAW)) @ rx(np.pi)
    T[:3, 3] = (*ROBO_XY, ROBO_Z)
    return T


# ------------------------------------------------------------------ cinemática direta
def _r03(q):
    """Rotação do antebraço (depois do A3) no referencial do robô."""
    return rz(-np.radians(q[0])) @ ry(np.radians(q[1] + q[2]))


def punho(q, sinal=None):
    """Centro do punho (cruzamento A4/A5) no referencial do robô."""
    s = SINAL_55 if sinal is None else sinal
    q = np.asarray(q, float)
    R1 = rz(-np.radians(q[0]))
    ombro = R1 @ np.array([A1_OFF, 0, D1])
    cotovelo = ombro + R1 @ ry(np.radians(q[1])) @ np.array([L2, 0, 0])
    return cotovelo + _r03(q) @ np.array([L3, 0, s * A3_OFF])


def fk(q, sinal=None):
    """Juntas (graus, como na consola) -> flange 4x4 no referencial do robô."""
    q = np.asarray(q, float)
    R = (_r03(q) @ rx(-np.radians(q[3])) @ ry(np.radians(q[4]))
         @ rx(-np.radians(q[5])) @ R_F0)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = punho(q, sinal) + D6 * R[:, 2]
    return T


def fk_bico(q):
    """Juntas -> ferramenta do bico 4x4, no referencial do robô."""
    return fk(q) @ frame(*TOOL_BICO)


# ------------------------------------------------------------------ cinemática inversa
def _dentro(a, lo, hi):
    """Ângulo a (graus) e as suas voltas de ±360 que caem em [lo, hi]."""
    return [x for x in (a - 360, a, a + 360) if lo - 1e-9 <= x <= hi + 1e-9]


def ik(T_flange, sinal=None):
    """Todas as soluções (graus) dentro dos limites para um flange 4x4 no
    referencial do robô. Analítica: braço (A1-A3) pelo centro do punho, punho
    (A4-A6) por ângulos de Euler XYX."""
    s = SINAL_55 if sinal is None else sinal
    R = T_flange[:3, :3]
    pw = T_flange[:3, 3] - D6 * R[:, 2]
    L3e = np.hypot(L3, A3_OFF)
    delta = np.arctan2(s * A3_OFF, L3)
    rho = np.hypot(pw[0], pw[1])
    fi = np.degrees(np.arctan2(pw[1], pw[0]))
    sols = []
    for tras, a1 in ((False, -fi), (True, -fi + 180.0)):
        for a1v in _dentro(((a1 + 180) % 360) - 180, *LIM[0]):
            r = (-rho if tras else rho) - A1_OFF
            h = pw[2] - D1
            D = (r * r + h * h - L2 * L2 - L3e * L3e) / (2 * L2 * L3e)
            if abs(D) > 1:
                continue
            for g in (np.arccos(D), -np.arccos(D)):
                e2 = np.arctan2(h, r) - np.arctan2(L3e * np.sin(g), L2 + L3e * np.cos(g))
                a2 = -np.degrees(e2)
                a3 = np.degrees(delta - g)
                a2 = ((a2 + 180) % 360) - 180
                a3 = ((a3 + 180) % 360) - 180
                if not (LIM[1, 0] <= a2 <= LIM[1, 1] and LIM[2, 0] <= a3 <= LIM[2, 1]):
                    continue
                M = _r03([a1v, a2, a3]).T @ R @ R_F0.T     # = Rx(-A4) Ry(A5) Rx(-A6)
                b = np.arccos(np.clip(M[0, 0], -1, 1))
                for bb in (b, -b):
                    sb = np.sin(bb)
                    if abs(sb) < 1e-9:                      # punho esticado: A4 + A6 livres
                        a, c = np.arctan2(-M[1, 2], M[1, 1]), 0.0
                    else:
                        a = np.arctan2(M[1, 0] / sb, -M[2, 0] / sb)
                        c = np.arctan2(M[0, 1] / sb, M[0, 2] / sb)
                    a4, a5, a6 = -np.degrees(a), np.degrees(bb), -np.degrees(c)
                    if not LIM[4, 0] <= a5 <= LIM[4, 1]:
                        continue
                    for a4v in _dentro(a4, *LIM[3]):
                        for a6v in _dentro(a6, *LIM[5]):
                            sols.append(np.array([a1v, a2, a3, a4v, a5, a6v]))
    return sols


# ------------------------------------------------------------------ Status e Turn da KUKA
def status(q, sinal=None):
    """S da KUKA. Bit 0: centro do punho atrás do A1 (zona de cima da cabeça).
    Bit 1: A3 >= ângulo em que o antebraço fica alinhado com o braço.
    Bit 2: A5 <= 0."""
    s = SINAL_55 if sinal is None else sinal
    q = np.asarray(q, float)
    pw = punho(q, s)
    x1 = (rz(np.radians(q[0])) @ pw)[0]          # x do punho no referencial do A1
    fi = np.degrees(np.arctan2(s * A3_OFF, L3))
    return int(x1 < 0) | (int(q[2] >= fi) << 1) | (int(q[4] <= 0) << 2)


def turn(q):
    """T da KUKA: bit i = 1 se A(i+1) < 0."""
    return sum(int(a < 0) << i for i, a in enumerate(q))
