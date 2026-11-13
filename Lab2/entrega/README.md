# Robótica 2026/27 — Laboratório 2: swing de golfe com o UFactory Lite 6

Grupo 1: Afonso Cruz (102815), Eduardo Batista (99931), Henrique Póvoa (102109).

O relatório é o `Relatorio_Lab2_Grupo1.pdf`. Esta pasta tem o código, o modelo
3D e os dados dos ensaios necessários para reproduzir todos os resultados e
figuras do relatório.

## Estrutura

```
Relatorio_Lab2_Grupo1.pdf   relatório
README.md                   este ficheiro
requirements.txt            dependências Python
src/                        código (um script por tarefa + módulos partilhados)
data/                       dados medidos no robô
models/lite6/               modelo 3D oficial do Lite 6 com pinça (MuJoCo)
report/                     onde os scripts gravam as figuras
```

### `src/`

| Ficheiro | Tarefa | O que faz |
|---|---|---|
| `config.py` | — | limites do fabricante, IP do robô, pinça, mesa (um só sítio para todos os números) |
| `kinematics.py` | 1 | cinemática direta (D-H modificado) e validação contra a tabela D-H clássica |
| `plot_frames.py` | 1 | figura dos referenciais D-H |
| `trajectory.py` | 2 | trajetória de swing e verificação de viabilidade |
| `mujoco_preview.py` | 2 | o swing no modelo 3D do robô: colisões e folgas |
| `robot.py` | 3, 4 | ligação ao robô (SDK `xarm-python-sdk`) |
| `run_robot.py` | 3 | executa a trajetória no robô (modo servo) e grava a pose medida |
| `plot_run.py` | 3 | pose pedida vs medida, erro e atraso, a partir dos dados gravados |
| `ik_compare.py` | 4 | inversa do robô → a nossa direta → erro |
| `ajuste_dh.py` | 4 | testa se a diferença para o controlador é geométrica |
| `robot_info.py` | 3, 4 | lê a configuração do controlador (não mexe o robô) |
| `ir_para_zero.py` | — | leva o braço à posição zero por etapas, em segurança |

### `data/`

| Ficheiro | Usado em |
|---|---|
| `robot_info.txt` | configuração lida do controlador (Tarefas 3 e 4) |
| `run_servo_lento{2,1,0.7}_golfista_perto*.csv` | Tarefa 3: ensaios da trajetória final (Tabela 5, Figura 7) |
| `ik_compare_golfista_perto.csv` | Tarefa 4: 100 pontos da trajetória final (Tabela 6, Figura 8) |
| `run_servo_lento{2,1}_phi50*.csv`, `ik_compare.csv` | primeira versão da trajetória |
| `run_v3_lento2.csv` | dados de validação do ajuste dos parâmetros D-H (`ajuste_dh.py`) |

Colunas dos `run_servo_*`: `t`, pose pedida ao controlador e pose medida com
`get_position()` (mm e graus; ambas do flange, porque o controlador tem o
desvio de ferramenta a zero).

## Instalação

Python 3.10 ou superior.

```bash
python -m venv .venv
source .venv/bin/activate          # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

Todos os comandos correm **a partir desta pasta**.

## Reproduzir os resultados sem o robô

```bash
# Tarefa 1 — cinemática direta
python src/kinematics.py                     # pose com q = 0: flange em (87; 0; 154,2) mm
python src/kinematics.py --q 0 30 60 0 30 0  # pose para um q qualquer [graus]
python src/kinematics.py --validar           # D-H modificado vs clássico, 10 000 q aleatórios
python src/plot_frames.py                    # figura dos referenciais

# Tarefa 2 — trajetória, viabilidade e ausência de contacto
python src/trajectory.py --golfista --perto --plot
python src/mujoco_preview.py --golfista --perto --folgas
python src/mujoco_preview.py --golfista --perto --fotos --strip --prefixo mjgp_
python src/mujoco_preview.py --golfista --perto --view   # janela interativa

# Tarefa 3 — análise dos ensaios gravados
python src/plot_run.py --golfista-perto

# Tarefa 4 — análise dos 100 pontos gravados
python src/ik_compare.py --golfista --perto --from-csv
python src/ajuste_dh.py
```

As figuras ficam em `report/` e os números aparecem no terminal.

## No laboratório, com o robô

O portátil liga-se ao controlador por cabo Ethernet, com um IP fixo na rede
`192.168.1.x`. O IP do robô está em `src/config.py` e pode ser mudado com a
variável de ambiente `LITE6_IP` ou com a opção `--ip`.

```bash
python src/robot_info.py                     # lê a configuração; não mexe o robô

# Tarefa 3. Sem --go só verifica e mostra o que faria.
python src/run_robot.py --metodo servo --lento 2 --golfista --perto
python src/run_robot.py --go --metodo servo --lento 2 --golfista --perto   # metade da velocidade
python src/run_robot.py --go --metodo servo --lento 1 --golfista --perto   # velocidade planeada

# Tarefa 4 (o robô não se mexe)
python src/ik_compare.py --golfista --perto --n 100

python src/ir_para_zero.py                   # no fim, braço à posição zero
```

Segurança: a trajetória só é enviada se a verificação de viabilidade passar, e
`run_robot.py` pede confirmação antes de mexer o robô. Começar sempre com
`--lento 2` ou mais e ter a mão no botão de emergência.

## Notas

- `trajectory.py` sem opções corre a primeira versão (arco cartesiano), que o
  relatório só refere na Secção 3.2; a trajetória do relatório é
  `--golfista --perto`.
- O modelo em `models/lite6/` vem do MuJoCo Menagerie (licença no ficheiro
  `LICENSE`).
