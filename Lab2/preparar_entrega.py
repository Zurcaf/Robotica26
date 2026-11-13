"""
Prepara a pasta de entrega do Lab 2 (Lab2/entrega/) a partir do repositório.

Uso (a partir de Lab2/):
    python preparar_entrega.py          # atualiza entrega/ e cria entrega_lab2_grupo1.zip

Copia só o que é preciso para reproduzir o relatório: o PDF compilado, os
scripts usados, o modelo 3D e os dados dos ensaios. O README.md da entrega é
mantido à mão em entrega/README.md e não é tocado.
"""
import os
import shutil

AQUI = os.path.dirname(os.path.abspath(__file__))
ENTREGA = os.path.join(AQUI, "entrega")

PDF = os.path.join("report", "ROB_LAB2_Report", "main.pdf")
SCRIPTS = ["config", "kinematics", "trajectory", "robot", "run_robot", "ik_compare",
           "robot_info", "mujoco_preview", "plot_frames", "plot_run", "ajuste_dh", "ir_para_zero"]
DADOS = ["robot_info.txt", "ik_compare_golfista_perto.csv",
         "run_servo_lento2_golfista_perto.csv", "run_servo_lento2_golfista_perto_rep2.csv",
         "run_servo_lento1_golfista_perto.csv", "run_servo_lento1_golfista_perto_rep2.csv",
         "run_servo_lento0.7_golfista_perto.csv",
         "run_servo_lento2_golfista_perto_sem_tampa.csv", "run_servo_lento2_golfista_perto_tampa.csv",
         "ik_compare.csv", "run_servo_lento2_phi50.csv", "run_servo_lento2_phi50_rep2.csv",
         "run_servo_lento1_phi50.csv", "run_servo_lento1_phi50_rep2.csv", "run_v3_lento2.csv"]
REQUISITOS = """# Python >= 3.10
numpy>=2.0
matplotlib>=3.8
xarm-python-sdk>=1.18     # só para falar com o robô (Tarefas 3 e 4)
mujoco>=3.1               # só para src/mujoco_preview.py
"""


def copia(origem, destino):
    os.makedirs(os.path.dirname(destino), exist_ok=True)
    shutil.copy2(origem, destino)


def main():
    for sub in ("src", "data", "models"):
        shutil.rmtree(os.path.join(ENTREGA, sub), ignore_errors=True)
    os.makedirs(os.path.join(ENTREGA, "report"), exist_ok=True)
    copia(os.path.join(AQUI, PDF), os.path.join(ENTREGA, "Relatorio_Lab2_Grupo1.pdf"))
    for s in SCRIPTS:
        copia(os.path.join(AQUI, "src", s + ".py"), os.path.join(ENTREGA, "src", s + ".py"))
    for d in DADOS:
        copia(os.path.join(AQUI, "data", d), os.path.join(ENTREGA, "data", d))
    shutil.copytree(os.path.join(AQUI, "models", "lite6"), os.path.join(ENTREGA, "models", "lite6"))
    with open(os.path.join(ENTREGA, "requirements.txt"), "w", encoding="utf-8") as f:
        f.write(REQUISITOS)
    with open(os.path.join(ENTREGA, "report", "LEIAME.txt"), "w", encoding="utf-8") as f:
        f.write("Pasta onde os scripts de src/ gravam as figuras.\n")
    zip_ = shutil.make_archive(os.path.join(AQUI, "entrega_lab2_grupo1"), "zip", ENTREGA)
    print(f"  entrega atualizada em {ENTREGA}\n  zip: {zip_}")


if __name__ == "__main__":
    main()
