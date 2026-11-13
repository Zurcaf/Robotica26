"""
Configuração partilhada do Lab 2 — UFactory Lite 6.

Os números do fabricante estão todos aqui, para serem fáceis de encontrar e de
citar. Fontes: especificações técnicas do Lite 6
(https://docs.lite6.ufactory.cc/9.technical_specifications.html e
https://www.ufactory.cc/lite-6-collaborative-robot/).

Unidades: mm e graus (as mesmas do xArm-Python-SDK com is_radian=False).
"""
import os

# --- Ligação -----------------------------------------------------------------
# IP da caixa de controlo (etiqueta na caixa). Pode ser sobreposto com a
# variável de ambiente LITE6_IP para não ter de editar o ficheiro.
ROBOT_IP = os.environ.get("LITE6_IP", "192.168.1.158")   # Lite 6 do laboratório

# --- Limites das juntas [graus] ------------------------------------------------
# Confirmados nas especificações oficiais do Lite 6 (UFACTORY, "Robot
# specifications", docs.api.ufactory.cc/private_modbus_tcp/p04robot-specifications.html).
JOINT_LIMITS = [
    (-360.0, 360.0),   # J1
    (-150.0, 150.0),   # J2
    (-3.5,   300.0),   # J3
    (-360.0, 360.0),   # J4
    (-124.0, 124.0),   # J5
    (-360.0, 360.0),   # J6
]

# --- Velocidades e acelerações máximas (especificações técnicas do fabricante).
# Repetibilidade do robô: ±0.5 mm.
JOINT_SPEED_MAX = 180.0     # graus/s, por junta
JOINT_ACC_MAX = 1145.0      # graus/s², por junta (mesma página das especificações acima)
TCP_SPEED_MAX = 500.0       # mm/s, velocidade linear do efetor final
# Alcance: a distância do ombro (J2) ao centro do pulso (J5) não pode passar o
# comprimento do braço esticado, 200 + sqrt(87² + 227.6²) mm (tabela D-H). Perto
# desse valor o cotovelo fica singular, por isso só se aceita até REACH_FACTOR.
# (Os 440 mm da ficha técnica contam até ao flange, não até ao pulso.)
REACH = 200.0 + (87.0 ** 2 + 227.6 ** 2) ** 0.5   # mm, = 443.7
REACH_FACTOR = 0.95

# Margem de segurança na verificação de viabilidade (Tarefa 2): só se aceitam
# trajetórias que usem no máximo 80 % de cada limite.
SAFETY_FACTOR = 0.8
# Folga mínima a cada limite de junta. A IK numérica corta nos limites, por isso
# uma junta encostada ao limite indica uma pose forçada.
JOINT_LIMIT_MARGIN = 5.0    # graus

# --- Pinça (Gripper Lite) ---------------------------------------------------------
# Montada no flange sem desvio nem rotação; a ponta (entre os dedos) fica a
# 83.6 mm ao longo do z da ferramenta (URDF oficial da UFACTORY). Nos scripts,
# --garra 0 corresponde a não ter pinça.
GRIPPER_TCP_Z = 83.6        # mm, flange -> pontas dos dedos
GRIPPER_MASS = 0.29         # kg

# --- Ambiente ------------------------------------------------------------------
# Plano da mesa (z no referencial da base) e cilindro a evitar à volta do eixo
# de J1: a ferramenta não pode descer abaixo da mesa nem entrar na base.
TABLE_Z = 0.0               # mm
TABLE_CLEARANCE = 30.0      # mm, altura mínima da ferramenta acima da mesa
BASE_RADIUS = 80.0          # mm, raio do cilindro da base a evitar
