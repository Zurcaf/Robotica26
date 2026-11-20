"""Sessão no robô: corre os ensaios um a um, por ordem, e espera entre cada um
(para começar e parar o vídeo de cada ensaio). Cada ensaio grava os seus
ficheiros (data/<data>_<ensaio>_imu.csv e _robo.csv), como correr.py.

    .venv/bin/python src/robo/sessao.py                     # todos, pela ordem de baixo
    .venv/bin/python src/robo/sessao.py poses inclinacoes   # só estes

Antes de cada ensaio:  sim = correr   s = saltar   q = acabar a sessão.
Depois de cada ensaio: r = repetir o mesmo, Enter = seguir.
"""
import subprocess
import sys
from pathlib import Path

import numpy as np

AQUI = Path(__file__).resolve().parent
LAB3 = AQUI.parents[1]
# Direções no referencial do robô: "frente" = para onde o braço aponta na casa
# (junta 1 a 0°); "direita"/"esquerda" vistas de trás do robô, a olhar para a frente.
ORDEM = [
    ("deriva", "--lento 2", "soco de 300 mm e quadrado, a meia velocidade (primeiro com a IMU a gravar)",
     "mexe-se num plano horizontal a 24 cm de altura, 30 cm à frente e até 23 cm para a direita. "
     "Câmara à frente-direita, a 1,2 m, alta (a olhar para baixo ~45°), para ver o quadrado."),
    ("poses", "", "17 orientações paradas: confirma a orientação da IMU (mandar os dados antes de seguir)",
     "a IMU anda numa caixa de ~20 cm à volta de um ponto 36 cm à frente, a 30 cm de altura. "
     "Câmara à frente-esquerda, a 1,5 m, à altura da IMU, com o braço todo no quadro."),
    ("inclinacoes", "", "rolamento e arfagem de 15 em 15°",
     "roda no sítio, 35 cm à frente, a 30 cm de altura; o kit fica virado para cima e para a esquerda. "
     "Câmara à frente-esquerda, a 1 m, um pouco acima da IMU."),
    ("abanar", "", "inclinada a 30 e 60° com abanões de ±10 mm a 2 Hz",
     "mesmo sítio que as inclinações; abanões de lado (esquerda-direita). Câmara à frente, a 1 m."),
    ("vaivem", "", "vaivém de ±50 mm (acelerações conhecidas)",
     "35 cm à frente, a 30 cm de altura, ±5 cm para os lados e para cima e para baixo. Câmara à frente, a 1 m."),
    ("acenar", "", "acenar ±30° em torno de um cotovelo",
     "35 cm à frente, a 30 cm de altura, ±7,5 cm para os lados. Câmara à frente, a 1 m."),
    ("deriva", "", "soco e quadrado à velocidade normal", "como o primeiro deriva."),
    ("rotacoes", "", "junta 6 a 30, 90 e 140 °/s: VIGIAR O CABO USB",
     "o pulso roda no sítio, ~36 cm à frente, a 30 cm de altura. "
     "Câmara à frente, a 0,8 m, a apanhar a pinça e o cabo."),
]


def duracao(nome, lento):
    f = LAB3 / "data" / "planos" / f"{nome}.npz"
    return float(np.load(f)["t"][-1]) * lento if f.exists() else float("nan")


def main():
    pedidos = sys.argv[1:]
    ordem = [e for e in ORDEM if not pedidos or e[0] in pedidos]
    print("Frente = para onde o braço aponta na casa (junta 1 a 0°); "
          "direita/esquerda vistas de trás do robô, a olhar para a frente.")
    feitos = []
    k = 0
    while k < len(ordem):
        nome, extra, texto, camara = ordem[k]
        lento = float(extra.split()[-1]) if "--lento" in extra else 1.0
        print(f"\n=== [{k + 1}/{len(ordem)}] {nome} {extra}".rstrip() + f"  (~{duracao(nome, lento) + 10:.0f} s)")
        print(f"    {texto}")
        print(f"    Câmara: {camara}")
        print("    Começa e acaba sempre na casa (juntas 0, 10, 40, 0, 30, 0).")
        r = input("    Começa a filmar. Mão no botão de emergência. sim / s (saltar) / q (acabar): ").strip().lower()
        if r == "q":
            break
        if r != "sim":
            print("    saltado.")
            k += 1
            continue
        cmd = [sys.executable, str(AQUI / "correr.py"), nome, "--go", "--ja-confirmado", *extra.split()]
        res = subprocess.run(cmd, cwd=LAB3)
        feitos.append((nome, "ok" if res.returncode == 0 else f"FALHOU (código {res.returncode})"))
        if res.returncode != 0:
            print(f"    FALHOU {nome}: ver a mensagem acima (o robô pode não ter mexido). Para o vídeo.")
        else:
            print(f"    Acabou {nome}. Para o vídeo (nome: {nome}).")
        if input("    r (repetir) ou Enter (seguir): ").strip().lower() != "r":
            k += 1
    print("\nResumo:")
    for nome, estado in feitos:
        print(f"  {nome:12s} {estado}")


if __name__ == "__main__":
    main()
