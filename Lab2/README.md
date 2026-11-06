# Lab 2 — Swing com o UFactory Lite 6

Enunciado: [lab_2_2026.pdf](lab_2_2026.pdf). Prazo: **13 nov 2026, 23:59:59**.
Grupos até 3.

O swing do Lab 1 passa para um braço real: o **Lite 6** (6 juntas rotacionais,
série), controlado por Python a partir de um portátil ligado por Ethernet (só
um portátil de cada vez).

- Robô: <https://ufactory.cc/lite-6-collaborative-robot/>
- SDK: <http://github.com/xArm-Developer/xArm-Python-SDK>

## Tarefas

| # | Pede | Onde |
|---|------|------|
| 1 | Cinemática direta; explicar referenciais e método | [src/kinematics.py](src/kinematics.py) |
| 2 | Trajetória (posição + orientação da garra em função de t) análoga ao swing, bola fixa; viável (velocidades, alcance, limites das juntas) e com gráficos que mostrem que não há contacto perigoso | [src/trajectory.py](src/trajectory.py) |
| 3 | Script que põe o robô a seguir a trajetória com as funções do SDK | [src/run_robot.py](src/run_robot.py) |
| 4 | IK do robô em muitos pontos da trajetória → FK da Tarefa 1 → discutir diferenças e precisão da IK | [src/ik_compare.py](src/ik_compare.py) |

## Setup

Precisa de **Python ≥ 3.10**.

A `.venv` não vai para o git (só funciona no PC onde foi criada). Recria-se
com um comando, com as versões exatas fixadas em `requirements.txt`:

```bash
cd Lab2
powershell -ExecutionPolicy Bypass -File setup.ps1   # Windows
bash setup.sh                                         # Mac/Linux
```

Depois: `.venv\Scripts\activate` (Windows) ou `source .venv/bin/activate`
(Mac/Linux).

IP do robô: em [src/config.py](src/config.py) (`ROBOT_IP`) ou pela variável de
ambiente `LITE6_IP`. O portátil tem de estar na mesma sub-rede (ex.:
192.168.1.x, IP fixo).

## Correr

Todos os comandos a partir da pasta `Lab2/`.

### Offline (sem robô)

```bash
python src/kinematics.py                      # FK com q = 0 -> (87, 0, 154.2) mm, roll 180
python src/kinematics.py --q 0 30 60 0 30 0   # FK para outro q
python src/kinematics.py --validar            # DH modificado vs DH clássico do PDF oficial
python src/trajectory.py --plot               # gera o swing, verifica viabilidade, gráficos em report/
python src/trajectory.py --t-down 3 --raio 180
python src/mujoco_preview.py                  # swing no modelo 3D oficial: colisões elo a elo + FK vs URDF
python src/mujoco_preview.py --view           # viewer interativo (swing em loop); --lento 2 --phi-end 50 etc.
python src/mujoco_preview.py --fotos --strip  # 7 fases x 3 câmaras -> report/mj_*.png e tiras mj_sequencia_*.png
python src/robot_info.py                      # (com robô) lê a configuração do controlador, não mexe nada
```

O `mujoco_preview.py` é um replay cinemático (põe as juntas nos ângulos da IK
numérica, sem física) sobre o modelo oficial do Lite 6 com pinça
(`models/lite6/`, do `mujoco_menagerie`, licença BSD). Serve para ver o braço
inteiro e verificar colisões com os elos a sério; as malhas de colisão são
invólucros convexos, por isso as distâncias que imprime são conservadoras.

Resultado atual com os `DEFAULTS` (swing `GOLFISTA`, ver abaixo):

```
  1073 pontos, 10.72 s, IK numérica convergiu
  vel. ponta      : max    302.9 mm/s   (limite c/ margem 400.0)  OK
  vel. flange     : max    269.4 mm/s   (limite c/ margem 400.0)  OK
  vel. juntas     : max     35.9 deg/s  (limite c/ margem 144.0)  OK
  alcance         : max    418.4 mm     (limite c/ margem 421.5)  OK
  limites juntas  : menor folga 74.0 deg (J5, mínimo 5)  OK
  acima da mesa   : z min 79.7 mm (ponta 79.7, flange 163.3, juntas 163.3)  OK
  fora da base    : r min 408.3 mm  OK
  => trajetória VIÁVEL
```

### O swing ativo: `GOLFISTA` (modo juntas)

Desenhado nas juntas como o V17 do Lab 1 (tronco = J1, ombro = J2, pulso = J5),
com o robô direito: braço quase esticado (J3 = 120°) e a pinça a pique na bola.
A trajetória cartesiana da pinça (o que a Tarefa 2 pede) é a FK destes perfis.

| Junta | Perfil | Valor |
|---|---|---|
| J1 tronco | `-0.8·phi` | ±72° / 40° |
| J2 ombro | `71 - 35·(1 - cos phi)` | topo a z ≈ 400 mm |
| J3 cotovelo | fixo | 120° |
| J5 pulso | `50 - psi` | arma 30° no backswing, solta aos 60 % do downswing |
| J4, J6 | fixos | 0° |

Fases do Lab 1: backswing quíntico (4.5 s) até `phi_top = -90°`, downswing com
aceleração constante (4 s), follow-through até +50°. Bola em (415, 0, 80) mm.
Tudo em `GOLFISTA`, no topo de [src/trajectory.py](src/trajectory.py); os
outros modos (arco cartesiano original, híbrido, cópia do vídeo) ficam lá para
comparação.

### No laboratório (robô ligado)

```bash
python src/robot_info.py                          # lê a configuração do controlador, não mexe (grava data/robot_info.txt)
python src/run_robot.py                           # ensaio: verifica, não mexe
python src/run_robot.py --go --metodo servo --lento 2   # executa 2x mais lento (começar assim), grava data/run_servo_lento2_golfista.csv
python src/run_robot.py --go --metodo servo --lento 1   # velocidade planeada
python src/ik_compare.py --n 100                  # Tarefa 4 (não mexe o robô), grava data/ik_compare.csv
python src/ik_compare.py --from-csv               # refaz o gráfico em casa
```

**Segurança:** emergência sempre à mão; `run_robot.py` recusa enviar
trajetórias que falhem a verificação e pede confirmação antes de mexer.

## Estrutura

```
lab2.ipynb  notebook com tudo (teoria + código + resultados); GUIA.md explica em texto
src/      config.py       IP, limites do fabricante, mesa/base (CONFIRMAR limites)
          kinematics.py   Tarefa 1: DH do PDF oficial, fk(), validação, rpy, IK numérica
          trajectory.py   Tarefa 2: swing (GOLFISTA nas juntas; arco cartesiano e híbrido em alternativa), viabilidade, gráficos
          robot.py        ligação ao robô (SDK xArm)
          run_robot.py    Tarefa 3
          ik_compare.py   Tarefa 4
          mujoco_preview.py  pré-visualização 3D do swing e colisões (MuJoCo, modelo oficial)
          robot_info.py   lê a configuração do controlador (tcp_load, limites, firmware)
models/lite6/  modelo MJCF oficial do Lite 6 + Gripper Lite (mujoco_menagerie) e a cena swing_scene.xml
data/     CSVs recolhidos no laboratório
report/   gráficos e relatorio.tex
scripts_curso/  scripts de exemplo da página da cadeira
```

## O modelo em poucas linhas

- **Referencial:** o da base do robô (o mesmo do SDK), mm e graus; orientação
  em roll/pitch/yaw com `R = Rz(yaw)·Ry(pitch)·Rx(roll)`.
- **Swing:** a garra faz de cabeça do taco. Percorre um arco de raio `raio` num
  plano vertical que passa na bola (`phi = 0` na bola) — é o "braço" do Lab 1.
  A orientação roda `phi + psi`, onde `psi` é o "pulso": armado no topo, solto a
  `f_release` do downswing.
- **Perfis:** backswing quíntico; downswing com aceleração constante
  (`phi = phi_top·(1-(u/T)²)`, velocidade máxima na bola, como no Lab 1);
  follow-through com desaceleração constante até parar.
- **Pinça:** UFACTORY Gripper for Lite 6, TCP a 83.6 mm do flange (URDF
  oficial). A **ponta da pinça** é a cabeça do taco e passa na bola em
  (190, 0, 80). Os scripts leem o `tcp_offset` do controlador e convertem as
  poses — não alteram a configuração do robô. `--garra 0` = sem pinça.
- **Singularidade:** com a garra a pique na bola (`rpy = 180, 0, 0`) o J5 passa
  por 0 e J4/J6 disparam (~1900 °/s). Por isso a garra está inclinada 45°
  (`rpy_enderecio`). A velocidade de J6 é o que limita `t_down`.

## A confirmar no laboratório

- Limites e velocidades em `config.py` (datasheet/manual do Lite 6). A tabela
  DH já está confirmada com o PDF oficial
  (`ufactory-lite-6-Kinematic-and-Dynamic-Parameters.pdf`).
- IP do robô e se a pinça montada é a Gripper Lite (o `tcp_offset` do
  controlador é lido automaticamente).
- Mesa ao nível da base e espaço livre à volta da bola (190, 0, 80).
