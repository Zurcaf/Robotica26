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
python src/trajectory.py --plot               # gera o swing, verifica viabilidade, gráficos em report/
python src/trajectory.py --t-down 3 --raio 180
```

Resultado atual com os `DEFAULTS`:

```
  597 pontos, 5.96 s, IK numérica convergiu
  vel. TCP        : max    245.4 mm/s   (limite c/ margem 400.0)  OK
  vel. juntas     : max    123.0 deg/s  (limite c/ margem 144.0)  OK
  alcance         : max    298.6 mm     (limite c/ margem 352.0)  OK
  ...
  => trajetória VIÁVEL
```

### No laboratório (robô ligado)

```bash
python src/run_robot.py                           # ensaio: verifica, não mexe
python src/run_robot.py --go --lento 3            # executa 3x mais lento (começar assim)
python src/run_robot.py --go --metodo servo       # streaming ponto a ponto, grava data/run_servo.csv
python src/ik_compare.py --n 60                   # Tarefa 4 (não mexe o robô), grava data/ik_compare.csv
python src/ik_compare.py --from-csv               # refaz o gráfico em casa
```

**Segurança:** emergência sempre à mão; `run_robot.py` recusa enviar
trajetórias que falhem a verificação e pede confirmação antes de mexer.

## Estrutura

```
lab2.ipynb  notebook com tudo (teoria + código + resultados); GUIA.md explica em texto
src/      config.py       IP, limites do fabricante, mesa/base (CONFIRMAR valores)
          kinematics.py   Tarefa 1: DH modificado, fk(), rpy, IK numérica (só p/ verificação)
          trajectory.py   Tarefa 2: swing (arco + "pulso"), viabilidade, gráficos
          robot.py        ligação ao robô (SDK xArm)
          run_robot.py    Tarefa 3
          ik_compare.py   Tarefa 4
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
- **Singularidade:** com a garra a pique na bola (`rpy = 180, 0, 0`) o J5 passa
  por 0 e J4/J6 disparam (~1900 °/s). Por isso a garra está inclinada 45°
  (`rpy_enderecio`). A velocidade de J6 é o que limita `t_down`.

## A confirmar no laboratório

- Tabela DH e limites em `config.py` (datasheet/manual do Lite 6).
- IP do robô e se a garra/ferramenta tem offset de TCP (se sim, somar à FK).
- Altura da mesa e posição da bola.
