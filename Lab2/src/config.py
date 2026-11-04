"""
Configuração partilhada do Lab 2 — UFactory Lite 6.

Tudo o que é "número do fabricante" está aqui, num só sítio, para o relatório
poder citar a fonte. Os valores marcados CONFIRMAR vieram da datasheet /
manual do Lite 6 e devem ser verificados no robô do laboratório
(https://ufactory.cc/lite-6-collaborative-robot/).

Unidades: mm e graus (as mesmas do xArm-Python-SDK com is_radian=False).
"""
import os

# --- Ligação -----------------------------------------------------------------
# IP da caixa de controlo (etiqueta na caixa). Pode ser sobreposto com a
# variável de ambiente LITE6_IP para não ter de editar o ficheiro.
ROBOT_IP = os.environ.get("LITE6_IP", "192.168.1.185")

# --- Limites das juntas [graus] (CONFIRMAR) ----------------------------------
JOINT_LIMITS = [
    (-360.0, 360.0),   # J1
    (-150.0, 150.0),   # J2
    (-3.5,   300.0),   # J3
    (-360.0, 360.0),   # J4
    (-124.0, 124.0),   # J5
    (-360.0, 360.0),   # J6
]

# --- Velocidades máximas (CONFIRMAR) -----------------------------------------
JOINT_SPEED_MAX = 180.0     # graus/s, por junta
TCP_SPEED_MAX = 500.0       # mm/s, velocidade linear da ferramenta
REACH = 440.0               # mm, alcance máximo (do eixo de J2 ao flange)

# Margem de segurança aplicada aos limites acima nas verificações de
# viabilidade (Tarefa 2): 0.8 => só se aceitam trajetórias que usem <= 80 %.
SAFETY_FACTOR = 0.8

# --- Ambiente ------------------------------------------------------------------
# Plano da mesa (z do referencial da base) e raio "proibido" em torno do eixo
# de J1 — a ferramenta nunca pode passar abaixo da mesa nem dentro da base.
TABLE_Z = 0.0               # mm
TABLE_CLEARANCE = 30.0      # mm, altura mínima da ferramenta acima da mesa
BASE_RADIUS = 80.0          # mm, raio do cilindro da base a evitar
