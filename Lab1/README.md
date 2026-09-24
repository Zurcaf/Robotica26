# Lab 1 — Swing de golfe em MuJoCo

Enunciado: [lab_1_2026.pdf](lab_1_2026.pdf). Prazo: 9 out 2026.

Simulação de um jogador de golfe + taco (o "robot") a bater numa bola, em
MuJoCo + Python. O modelo tem **2 graus de liberdade atuados**: o braço e o
pulso — o pêndulo duplo clássico do swing (Cochran & Stobbs, *Search for the
Perfect Swing*, 1968).

## Setup (Mac / Windows / Linux)

Precisa de **Python ≥ 3.10** (o 3.9 não tem wheel do MuJoCo recente).

```bash
cd Lab1
python -m venv .venv
source .venv/bin/activate      # Mac/Linux
.venv\Scripts\activate         # Windows
pip install -r requirements.txt
```

## Correr

Todos os comandos a partir da pasta `Lab1/`.

### O swing

```bash
python src/swing.py                 # corre um swing e imprime os resultados
python src/swing.py --view          # viewer interativo, swing em tempo real
python src/swing.py --f-release 0.3 # pulso a soltar CEDO ("casting"): perde velocidade
python src/swing.py --t-down 0.34   # downswing mais lento
```

Parâmetros disponíveis: `--t-down`, `--q-top-shoulder`, `--q-top-wrist`,
`--f-release`. Os valores por omissão estão em `DEFAULTS`, no topo de
[src/swing.py](src/swing.py).

Se o impacto cair longe de (0,0) o script avisa: significa que os motores
saturaram e a trajetória deixou de ser seguida, e a velocidade que aparece é de
uma batida má — não a uses.

Resultado esperado:

```
  cabeça do taco no impacto :   37.2 m/s     (real, amador c/ ferro 7: 30-38)
  bola                      :   45.6 m/s  (smash 1.22)
  lançamento                :   31.7 deg
  juntas no impacto         : braço +0.8 deg, pulso -1.1 deg   (alvo: 0, 0)
  carry (até aterrar)       :  189.2 m
```

### Perturbações (Tarefa 2)

```bash
python src/perturbacoes.py                    # 20 swings, ruído de 10 N.m no pulso
python src/perturbacoes.py --n 50 --std 20    # mais repetições, mais ruído
python src/perturbacoes.py --junta shoulder   # ruído no braço
```

Grava `report/dispersao_perturbacao.png` com a dispersão dos pontos de aterragem.

O ruído é mantido durante 1/10 s (banda do tremor fisiológico, 8-12 Hz) em vez
de ser re-amostrado a cada passo — ruído branco por passo cancelava-se e o
resultado dependia do passo de integração.

**O pulso é o dof sensível**: com σ = 10 N·m, o ruído no pulso dá ~6 m de
dispersão lateral e o no braço ~0.2 m. Um pulso trémulo manda a bola para o lado
e às vezes faz uma batida falhada (bola "topada", lançamento negativo).

### Imagens e vídeo

```bash
python src/renders.py                         # 5 fases x 2 câmaras -> report/
python src/video.py --lento 8                 # vídeo em câmara lenta
python src/video.py --lento 25 --camera down_the_line
```

O `--lento N` controla exatamente quantas vezes mais lento se vê o swing.
Usa `ffmpeg` se existir; senão grava GIF (`--gif` força).

### Os pêndulos (validação da física, dica 1 do enunciado)

```bash
python src/run_pendulum.py            # mede o período; erro < 0.2 % vs teoria
python src/run_pendulum.py --view
python src/run_pendulum_ball.py       # pêndulo a bater na bola
```

## Viewer

O `--view` abre o swing a correr. Para abrir um XML solto:

```bash
# Mac/Linux — tem de ser caminho ABSOLUTO (o viewer perde a pasta atual)
python -m mujoco.viewer --mjcf="$PWD/models/golfer_v13.xml"
# Windows (PowerShell)
python -m mujoco.viewer --mjcf="$PWD\models\golfer_v13.xml"
```

Ou abre o viewer vazio (`python -m mujoco.viewer`) e arrasta o XML para a janela.

| Tecla / painel | Faz |
|---|---|
| **Space** | play / pausa |
| **Backspace** | repõe — faz novo swing |
| **Ctrl+L** | recarrega o XML depois de o editares |
| *Option → Help* | lista **todas** as teclas, incluindo a velocidade de reprodução |
| *Option → Sensor* | gráficos ao vivo: ângulos, velocidades e binário de cada junta |
| *Option → Info* | tempo, energia, nº de contactos |
| painel direito → *Joint* | sliders, um por dof — mexer o boneco à mão |
| *Rendering → Frame → Body* | desenha os referenciais de cada corpo |

Notas macOS: não usar `mjpython` (avariado nesta versão); o `python` normal
funciona. O `swing.py --view` regista o controlador com `set_mjcb_control` em vez
de `launch_passive` precisamente para não depender do `mjpython`.

## Estrutura

```
models/   golfer_v13.xml   modelo do jogador + taco (comentado)
          00_pendulum.xml  pêndulo simples (validação)
          01_pendulum_ball.xml  pêndulo a bater na bola
src/      swing.py         modelo de controlo + run_swing()
          perturbacoes.py  Tarefa 2: ruído nas juntas + gráfico
          renders.py       imagens das fases do swing
          video.py         vídeo em câmara lenta
          run_pendulum*.py validação da física
report/   imagens, gráficos e relatório
```

## O modelo em duas linhas

- **Referenciais:** origem do mundo na bola, X = direção do alvo, Z para cima;
  o jogador está do lado +Y.
- **dof 1 — braço:** os braços rodam em bloco em torno do centro do peito, num
  eixo normal ao plano de swing. Motor ±150 N·m.
- **dof 2 — pulso:** o taco roda em torno do punho. Motor ±40 N·m. No topo está
  armado ~95° e só solta a 65 % do downswing ("lag") — é isto que gera
  velocidade, porque com o taco dobrado a inércia é baixa. Soltar a 10 % dá
  26.8 m/s; a 65 % dá 37.2 m/s (+39 %).
- Tudo o resto (pernas, ancas, tronco, cotovelos) é **rígido** na pose de
  endereço.

O controlo é **binário calculado**: `tau = M(q)·a_cmd + h(q,v)`, com os binários
sempre saturados no `ctrlrange` do XML. A trajetória do downswing tem aceleração
constante e o **mesmo T nas duas juntas**, o que as faz cruzar q = 0 em
simultâneo — é a condição para a cabeça do taco passar na bola.

Lista completa de simplificações e do seu impacto: `report/simplificacoes.md`.

## Versões

| Tag   | Conteúdo |
|-------|----------|
| `pendulo-base` | pêndulo simples + pêndulo a bater na bola, validados |
| v13   | golfista 2 dof (braço + pulso) + testes de perturbação + relatório |
| v17   | + ombros articulados, parâmetros antropométricos, bateria de testes por dof |
| v20   | + STL, análise de sensibilidade |
