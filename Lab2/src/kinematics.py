"""
Tarefa 1 — Cinemática direta do UFactory Lite 6.

Uso (a partir de Lab2/):
    python src/kinematics.py                       # pose com todas as juntas a 0
    python src/kinematics.py --q 0 30 60 0 30 0    # pose para um q qualquer [graus]
    python src/kinematics.py --frames              # imprime os 7 referenciais

CONVENÇÃO
---------
D-H MODIFICADO (Craig): o referencial i está na junta i, e

    {i-1}T{i} = Rx(alpha_{i-1}) · Tx(a_{i-1}) · Rz(theta_i) · Tz(d_i)

com theta_i = q_i + offset_i. A tabela DH abaixo segue a documentação da
UFactory para o Lite 6 (CONFIRMAR) — é essa a convenção que o controlador usa,
por isso a pose devolvida aqui deve bater com arm.get_position() /
arm.get_forward_kinematics() no robô real.

Orientação em roll/pitch/yaw como no SDK do xArm:
    R = Rz(yaw) · Ry(pitch) · Rx(roll)

Também está aqui uma IK numérica (mínimos quadrados amortecidos) — NÃO é a
do enunciado (a Tarefa 4 pede a IK do robô); serve só para verificar
offline, na Tarefa 2, se a trajetória respeita os limites das juntas.
"""
import argparse

import numpy as np

from config import JOINT_LIMITS

#                offset[deg]  d[mm]   alpha[deg]  a[mm]
DH = np.array([
    [  0.0,      243.3,     0.0,       0.0],   # J1
    [-90.0,        0.0,   -90.0,       0.0],   # J2
    [-90.0,        0.0,   180.0,     200.0],   # J3
    [  0.0,      227.6,    90.0,      87.0],   # J4
    [  0.0,        0.0,    90.0,       0.0],   # J5
    [  0.0,       61.5,   -90.0,       0.0],   # J6
])


def link_transform(alpha, a, theta, d):
    """{i-1}T{i} em D-H modificado. Ângulos em rad, comprimentos em mm."""
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [ct,      -st,      0.0,  a],
        [st * ca,  ct * ca, -sa, -sa * d],
        [st * sa,  ct * sa,  ca,  ca * d],
        [0.0,      0.0,     0.0,  1.0],
    ])


def fk(q_deg, all_frames=False):
    """Transformação base -> flange para q em graus (ou a lista {0}T{i})."""
    q = np.radians(np.asarray(q_deg, dtype=float))
    T = np.eye(4)
    frames = [T]
    for i, (off, d, alpha, a) in enumerate(DH):
        T = T @ link_transform(np.radians(alpha), a, q[i] + np.radians(off), d)
        frames.append(T)
    return frames if all_frames else T


def rpy_from_R(R):
    """R = Rz(yaw)Ry(pitch)Rx(roll) -> (roll, pitch, yaw) em graus."""
    pitch = np.arcsin(-np.clip(R[2, 0], -1.0, 1.0))
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return np.degrees([roll, pitch, yaw])


def R_from_rpy(roll, pitch, yaw):
    """(roll, pitch, yaw) em graus -> matriz de rotação."""
    r, p, y = np.radians([roll, pitch, yaw])
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def fk_pose(q_deg):
    """q [graus] -> [x, y, z, roll, pitch, yaw] em mm / graus (formato do SDK)."""
    T = fk(q_deg)
    return np.concatenate([T[:3, 3], rpy_from_R(T[:3, :3])])


def rotvec(R):
    """Logaritmo de SO(3): vetor de rotação (rad) equivalente a R."""
    c = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    ang = np.arccos(c)
    if ang < 1e-9:
        return np.zeros(3)
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    s = np.sin(ang)
    if s < 1e-6:  # perto de 180 graus: eixo pela diagonal
        axis = np.sqrt(np.maximum((np.diag(R) + 1.0) / 2.0, 0.0))
        axis *= np.sign(w + 1e-12)
        return ang * axis / np.linalg.norm(axis)
    return ang * w / (2.0 * s)


def orientation_error_deg(R_a, R_b):
    """Ângulo (graus) da rotação que leva R_b a R_a."""
    return np.degrees(np.linalg.norm(rotvec(R_a @ R_b.T)))


def ik_numeric(T_target, q0_deg, iters=200, tol_mm=1e-3, tol_rad=1e-5, lam=1.0, w_rot=100.0):
    """
    IK numérica por mínimos quadrados amortecidos com jacobiano por diferenças
    finitas. w_rot pondera o erro de orientação (rad) face ao de posição (mm).
    Devolve (q_deg, convergiu).
    """
    q = np.asarray(q0_deg, dtype=float).copy()
    lo = np.array([l for l, _ in JOINT_LIMITS])
    hi = np.array([h for _, h in JOINT_LIMITS])

    def err(qd):
        T = fk(qd)
        e_p = T_target[:3, 3] - T[:3, 3]
        e_r = rotvec(T_target[:3, :3] @ T[:3, :3].T)
        return np.concatenate([e_p, w_rot * e_r]), e_p, e_r

    for _ in range(iters):
        e, e_p, e_r = err(q)
        if np.linalg.norm(e_p) < tol_mm and np.linalg.norm(e_r) < tol_rad:
            return q, True
        J = np.zeros((6, 6))
        h = 1e-4  # graus
        for j in range(6):
            dq = np.zeros(6)
            dq[j] = h
            J[:, j] = -(err(q + dq)[0] - e) / h
        dq = J.T @ np.linalg.solve(J @ J.T + lam ** 2 * np.eye(6), e)
        q = np.clip(q + dq, lo, hi)
    return q, False


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--q", type=float, nargs=6, default=[0.0] * 6, help="ângulos das juntas [graus]")
    ap.add_argument("--frames", action="store_true", help="imprime todos os referenciais")
    args = ap.parse_args()

    np.set_printoptions(precision=3, suppress=True)
    pose = fk_pose(args.q)
    print(f"q            : {np.array(args.q)} deg")
    print(f"pose flange  : x={pose[0]:.2f} y={pose[1]:.2f} z={pose[2]:.2f} mm  "
          f"roll={pose[3]:.2f} pitch={pose[4]:.2f} yaw={pose[5]:.2f} deg")
    if args.frames:
        for i, T in enumerate(fk(args.q, all_frames=True)):
            print(f"\n{{0}}T{{{i}}} =\n{T}")


if __name__ == "__main__":
    main()
