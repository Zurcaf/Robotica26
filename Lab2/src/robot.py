"""
Ligação ao Lite 6 (partilhado pelas Tarefas 3 e 4).

Só um portátil de cada vez pode estar ligado ao robô (cabo Ethernet). O IP vem
de config.ROBOT_IP ou da variável de ambiente LITE6_IP.
"""
import numpy as np

import config as C


def connect(ip=None, enable_motion=True):
    """Liga ao controlador. Com enable_motion=False serve só para consultas (IK/FK)."""
    from xarm.wrapper import XArmAPI

    arm = XArmAPI(ip or C.ROBOT_IP, is_radian=False)
    arm.clean_warn()
    arm.clean_error()
    if enable_motion:
        arm.motion_enable(enable=True)
        arm.set_mode(0)     # 0 = posição; 1 = servo (streaming)
        arm.set_state(0)    # 0 = pronto a mexer
    return arm


def controller_tcp(arm):
    """
    Flange -> TCP que o controlador está a usar (arm.tcp_offset, configurável no
    UFactory Studio). set_position / get_position / IK / FK do SDK referem-se a
    este ponto, por isso as poses enviadas têm de ser convertidas para ele.
    O código lê-o mas não o altera, porque o robô é partilhado.
    """
    from kinematics import T_from_pose

    return T_from_pose(np.asarray(arm.tcp_offset[:6], dtype=float))


def wrap180(a):
    """Ângulos em graus para (-180, 180] — o SDK não aceita valores desembrulhados."""
    return (np.asarray(a) + 180.0) % 360.0 - 180.0


def pose_sdk(pos, rpy):
    """[x, y, z, roll, pitch, yaw] no formato do SDK (mm, graus)."""
    return [float(v) for v in np.concatenate([pos, wrap180(rpy)])]


def check(code, what):
    if code != 0:
        raise RuntimeError(f"{what} devolveu código {code} (ver xArm-Python-SDK, códigos de erro)")
