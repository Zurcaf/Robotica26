# Lab 2 — Swing de golfe com o UFactory Lite 6

Enunciado: [lab_2_2026.pdf](lab_2_2026.pdf). Entregue a 13 nov 2026.
Grupo: Afonso Cruz, Eduardo Batista, Henrique Póvoa.

O swing do Lab 1 passado para um braço real de 6 juntas. O swing final
(`--golfista --perto`) é desenhado nas juntas: base = tronco, junta 2 = ombro,
junta 5 = pulso, com a pinça a fazer de taco. Foi simulado no MuJoCo e depois
executado no robô a três velocidades.

- Relatório: [report/ROB_LAB2_Report/main.pdf](report/ROB_LAB2_Report/main.pdf)
- Vídeo (simulação + ensaios com os dados ao lado): <https://youtu.be/F_t07cNw1qI>
- Notas das sessões de laboratório: [data/LAB_2026-11-13.md](data/LAB_2026-11-13.md) (final), e as de 5 e 6/11 na mesma pasta

## Pastas

```
src/        código do relatório (um script por tarefa + módulos partilhados)
data/       dados medidos no robô e notas de cada sessão
models/     modelo 3D oficial do Lite 6 com pinça (MuJoCo Menagerie)
report/     figuras geradas pelos scripts; ROB_LAB2_Report/ = relatório em LaTeX
video/      scripts que montaram o vídeo do YouTube (media/ não vai para o git)
arquivo/    versões abandonadas e documentação antiga (swing V3, guias de 5–9/11)
scripts_curso/   exemplos da página da cadeira
```

A pasta de entrega monta-se com `python preparar_entrega.py` (copia o PDF, os
12 scripts do relatório, o modelo e os dados).

## `src/`

| Ficheiro | Tarefa | O que faz |
|---|---|---|
| `config.py` | — | limites do fabricante, IP do robô, pinça, mesa |
| `kinematics.py` | 1 | cinemática direta (D-H modificado) e validação |
| `plot_frames.py` | 1 | figura dos referenciais |
| `trajectory.py` | 2 | trajetória de swing e verificação de viabilidade |
| `mujoco_preview.py` | 2 | o swing no modelo 3D: colisões, folgas, imagens, janela interativa |
| `robot.py` | 3, 4 | ligação ao robô (SDK `xarm-python-sdk`) |
| `run_robot.py` | 3 | executa a trajetória no robô e grava a pose medida |
| `plot_run.py` | 3 | pedido vs medido, erro e atraso |
| `ik_compare.py` | 4 | inversa do robô → a nossa direta → erro |
| `ajuste_dh.py` | 4 | testa se a diferença para o controlador é geométrica |
| `robot_info.py` | 3, 4 | lê a configuração do controlador (não mexe o robô) |
| `ir_para_zero.py`, `ir_para_pose.py` | — | levar o braço ao zero ou a uma pose, devagar |

## Correr

Python ≥ 3.10; `bash setup.sh` (ou `setup.ps1`) cria a `.venv` com as versões
do `requirements.txt`. Tudo a partir de `Lab2/`.

```bash
# sem robô
python src/kinematics.py --validar
python src/trajectory.py --golfista --perto --plot
python src/mujoco_preview.py --golfista --perto --folgas
python src/mujoco_preview.py --golfista --perto --view
python src/plot_run.py --golfista-perto
python src/ik_compare.py --golfista --perto --from-csv

# com o robô (192.168.1.158, IP fixo no portátil em 192.168.1.x)
python src/robot_info.py
python src/run_robot.py --go --metodo servo --lento 2 --golfista --perto
python src/ik_compare.py --golfista --perto --n 100
python src/ir_para_zero.py
```

`--lento N` estica o tempo N vezes (2 = metade da velocidade; 0.7 = 43 % mais
rápido). Sem `--golfista --perto` os scripts correm a primeira versão (arco
cartesiano), que o relatório só refere na Secção 3.2.
