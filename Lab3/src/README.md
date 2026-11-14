# Lab 3 — código

Tudo corre a partir de `Lab3/`, com a `.venv` criada por `bash setup.sh`
(Python 3.12; versões em `requirements.txt`).

## Gravar

| Ficheiro | O que faz |
|---|---|
| `arduino/imu_bruto/imu_bruto.ino` | Sketch do Uno: lê o ICM-20600 por I2C e envia contagens em bruto a 200 Hz, 500000 baud. Só usa `Wire`. |
| `gravar.py` | Grava a porta série para `data/<data>_<nome>.csv`; mostra taxa, falhas e \|a\|. A classe `Gravador` é usada pelo robô. |
| `imu_io.py` | Lê e escreve os ficheiros (contagens -> g, °/s, °C). |

Formato dos ficheiros: linhas `#` com a configuração e
`t_us,ax,ay,az,temp,gx,gy,gz,t_host`, em contagens do chip. `t_us` é o
relógio do Arduino; `t_host` a hora do computador quando a linha chegou.

## Simular

`simular.py` gera leituras com erros conhecidos (desvio, escala, eixos não
perpendiculares, ruído, deriva do desvio, temperatura) para nove cenários à
mão. Escreve `data/sim/<cenário>.csv` e a verdade em `<cenário>_verdade.npz`.
Os dados simulados não vão para o git: geram-se em 10 s.

```
.venv/bin/python src/simular.py
```

## Analisar

| Ficheiro | Tarefa |
|---|---|
| `calibracao.py` | Funções: troços parados, seis posições, posições ao acaso (elipsoide), desvio e temperatura do giroscópio, rotações conhecidas, variância de Allan. |
| `orientacao.py` | Quaterniões (implementação própria), inclinómetro, integração do giroscópio. |
| `t2_calibracao.py` | Tarefa 2: parâmetros e figuras; grava `resultados/<sim\|real>/calibracao.json`. |
| `t3_t4_orientacao.py` | Tarefas 3 e 4: inclinómetro e giroscópio sobre os mesmos dados, com a verdade do simulador ou do robô (`--robo`). |

```
.venv/bin/python src/t2_calibracao.py --sim
.venv/bin/python src/t3_t4_orientacao.py --sim inclinar_abanar
```

Com `--sim`, cada parâmetro aparece ao lado do valor verdadeiro: é a
verificação de que a análise está certa antes de haver dados reais.

## Robô (`robo/`)

| Ficheiro | O que faz |
|---|---|
| `geometria.py` | Cinemática do Lab 2 até à IMU; modelo MuJoCo com o suporte e o kit no lugar de um dedo; hipótese da orientação do chip (`R_F_IMU`, `P_F_IMU`). |
| `ensaios.py` | Os sete ensaios (poses, rotacoes, inclinacoes, abanar, vaivem, deriva, acenar), como juntas a 100 Hz. |
| `verificar.py` | Limites, velocidades, folgas no MuJoCo; `--ver` (janela), `--fotos`, `--sim` (IMU simulada no robô). Grava os planos em `data/planos/`. |
| `correr.py` | Corre um plano no robô (`--go`) e grava a IMU e as juntas ao mesmo tempo. |
| `verdade.py` | Juntas medidas -> orientação e posição da IMU no instante de cada amostra (sincronização pelo relógio e por \|ω\|). |

```
.venv/bin/python src/robo/verificar.py
.venv/bin/python src/robo/verificar.py acenar --ver
.venv/bin/python src/robo/correr.py acenar --go
.venv/bin/python src/t3_t4_orientacao.py data/<..._acenar_imu>.csv --robo data/<..._acenar_robo>.csv
```

No macOS a janela do MuJoCo abre com o python normal (`.venv/bin/python`),
como no Lab 2.
