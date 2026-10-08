# Lab 1 — Swing de golfe em MuJoCo

Enunciado: [lab_1_2026.pdf](lab_1_2026.pdf). Prazo: 9 out 2026.

Simulação de um jogador de golfe + taco (o "robot") a bater numa bola, em
MuJoCo + Python. Há dois modelos:

- **V17** (`models/golfer_v17.xml`, o entregue): **3 dof atuados** — rotação
  do tronco, elevação do braço esquerdo e pulso — mais um braço direito
  passivo preso ao punho do taco. É o modelo descrito no relatório
  (`report/ROB_LAB1_Report/main.tex`).
- **V13** (`models/golfer_v13.xml`): **2 dof** — os braços em bloco e o pulso,
  o pêndulo duplo clássico (Cochran & Stobbs, 1968). Mais rápido (38.8 m/s),
  mas os braços atravessam o peito no backswing. Fica como comparação; a sua
  análise longa está em `report/simplificacoes.md`.

Todos os scripts aceitam `--modelo <xml>`; sem argumento usam o V17.

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
python src/swing.py                              # V17: corre um swing e imprime os resultados
python src/swing.py --view                       # viewer interativo, swing em tempo real
python src/swing.py --t-down 0.34                # downswing mais lento
python src/swing.py --q-top -90,-85,-95          # topo do backswing (tronco, ombro, pulso)
python src/swing.py --modelo models/golfer_v13.xml --f-release 0.3   # V13, pulso a soltar cedo
```

Os valores por omissão estão em `DEFAULTS_V17` / `DEFAULTS_V13`, no topo de
[src/swing.py](src/swing.py).

Se o impacto cair longe de (0,0) o script avisa: significa que os motores
saturaram e a trajetória deixou de ser seguida, e a velocidade que aparece é de
uma batida má — não a uses.

Resultado esperado (V17):

```
  cabeça do taco no impacto :   28.2 m/s     (real, ferro 7: 34-40; amador ~36)
  bola                      :   33.9 m/s  (smash 1.20)
  lançamento                :   33.6 deg
  juntas no impacto         : tronco -1.0 deg, ombro +1.1 deg, pulso +2.7 deg   (alvo: 0)
  carry (até aterrar)       :   94.6 m
  saturação no downswing    : tronco 0 %, ombro 0 %, pulso 13 %
  erro máx mão dir.-punho   :   2.2 mm
```

E com `--modelo models/golfer_v13.xml`: 38.8 m/s, bola 47.2 m/s, carry 151.3 m.

Demora ~7 s por swing. O passo de integração é 5e-5 s, mas desce para 5e-6 s
enquanto a cabeça do taco está perto da bola: sem isso o contacto não convergia
e a velocidade da bola variava ±5 % com mudanças de 1 ms no downswing.

### Perturbações (Tarefa 2)

```bash
python src/perturbacoes.py                    # todas as juntas, 20 swings, sigma = 10 N.m
python src/perturbacoes.py --junta wrist --n 50 --std 4
python src/perturbacoes.py --so-grafico       # redesenha a partir dos CSV
```

Grava `report/perturbacoes_<junta>.csv`, `report/dispersao_perturbacao.png` e
`report/ROB_LAB1_Report/Photos/v17_dispersao.png` (um painel por junta), e
imprime a linha LaTeX de cada junta. ~3-4 min por junta.

O ruído é mantido durante 1/10 s (banda do tremor fisiológico, 8-12 Hz) em vez
de ser re-amostrado a cada passo — ruído branco por passo cancelava-se e o
resultado dependia do passo de integração.

O ruído só atua depois de o taco sair do endereço (`t_noise_on`): com a face
a milímetros da bola, 10 N·m no pulso em repouso deslocavam-na antes do swing.
Resultados por junta no relatório (Tarefa 2) e, para o V13, em
`report/simplificacoes.md` §5.

### Aerodinâmica

```bash
python src/teste_aerodinamica.py          # Cd/CL efetivos da bola + ar no braço/taco
python src/trajetoria.py --perfeita       # bola lançada com as condições de um ferro 7
python src/trajetoria.py --tag com_ar     # 10 swings com ruído, voos em CSV + gráfico
```

O ar está ligado para **todo** o modelo (`density`/`viscosity` no `<option>`),
com `fluidshape="ellipsoid"` em todas as geoms. O `teste_aerodinamica.py` mede
o que o MuJoCo aplica de facto à bola (Cd 0.28, CL 0.20, rotação a decair
3.5 %/s), compara três modelos de ar no braço e no taco e grava
`report/aerodinamica.png`. Resultado: o ar tira 1.3 % à velocidade do taco e o
carry passa de 201 m (vácuo) para 151 m. O modelo de ar por omissão do MuJoCo
(sem `fluidshape`) exagerava o arrasto no taco 6×; ver `report/simplificacoes.md` §4.1.

O `trajetoria.py` grava `report/trajetoria_*.csv`, `report/metricas_voo.csv` e
`report/trajetoria.png`.

### Sequência cinemática e imagens

```bash
python src/sequencia.py --penetracao          # ângulos/velocidades/binários -> Photos/v17_sequencia.png
python src/renders.py --strip --prefixo v17_  # 7 fases x 2 câmaras + tiras para o relatório
python src/video.py --lento 8                 # vídeo em câmara lenta
python src/video.py --lento 25 --camera down_the_line
```

O `--lento N` controla exatamente quantas vezes mais lento se vê o swing.
Usa `ffmpeg` se existir; senão grava GIF (`--gif` força).

### Relatório (LaTeX)

Fontes em `report/ROB_LAB1_Report/` (`main.tex`, `refs.bib`, `Photos/`). As
figuras `Photos/v17_*.png` são geradas pelos scripts acima. Para compilar:

```bash
cd report/ROB_LAB1_Report && latexmk -pdf main.tex     # corre pdflatex + bibtex
```

No VS Code, com a extensão LaTeX Workshop (recomendada em `.vscode/`), basta
gravar o `main.tex`: a receita `latexmk (pdflatex + bibtex)` está em
`.vscode/settings.json`. O `main.pdf` final está no git; os ficheiros
auxiliares não.

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
python -m mujoco.viewer --mjcf="$PWD/models/golfer_v17.xml"
# Windows (PowerShell)
python -m mujoco.viewer --mjcf="$PWD\models\golfer_v17.xml"
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
models/   golfer_v17.xml   modelo entregue: 3 dof + braço direito passivo (comentado)
          golfer_v13.xml   modelo de 2 dof (comparação)
          00_pendulum.xml  pêndulo simples (validação)
          01_pendulum_ball.xml  pêndulo a bater na bola
src/      swing.py         trajetórias, controlador, run_swing(); genérico em N juntas
          perturbacoes.py  Tarefa 2: ruído por junta + CSV + gráfico
          sequencia.py     ângulos/velocidades/binários vs tempo (+ penetração)
          teste_aerodinamica.py  validação do modelo de ar (bola, braço, taco)
          trajetoria.py    voos em CSV e comparação com um ferro 7
          renders.py       imagens das fases do swing (+ tiras para o relatório)
          video.py         vídeo em câmara lenta
          run_pendulum*.py validação da física
report/   ROB_LAB1_Report/ relatório LaTeX (main.tex, refs.bib, Photos/, main.pdf)
          simplificacoes.md  análise longa do V13; bibliografia.md  referências anotadas
          imagens, CSVs e gráficos
```

## O modelo em poucas linhas

- **Referenciais:** origem do mundo na bola, X = direção do alvo, Z para cima;
  o jogador está do lado +Y. Em q = 0 (endereço) todos os corpos estão
  alinhados com o mundo.
- **V17, dof 1 — tronco:** roda em torno da coluna (35° da vertical). ±150 N·m:
  representa toda a cadeia pernas + ancas + tronco.
- **V17, dof 2 — ombro esquerdo:** o braço da frente sobe à frente do peito, em
  torno da linha dos ombros. ±100 N·m. Com este eixo a mão direita (presa ao
  punho) nunca tem de esticar mais do que no endereço.
- **V17/V13, dof pulso:** o taco roda em torno do punho. ±40 N·m. Armado no topo
  (~95°) e solto tarde ("lag"): com o taco dobrado a inércia é baixa, e a
  energia é entregue no fim, quando o braço de alavanca é longo.
- **V17, passivo:** braço direito (rótula + cotovelo) preso ao punho por uma
  restrição; cabeça acoplada a −tronco (fica parada).
- Pernas, ancas e cotovelo esquerdo são **rígidos**.

O controlo é **binário calculado**: `tau = M(q)·a_cmd + h(q,v)`, saturado no
`ctrlrange` do XML. No V17 a referência são polinómios de 5.º grau por junta,
todos a cruzar q = 0 no mesmo instante (o impacto), cada um com a sua
velocidade de chegada; no V13 é a parábola de aceleração constante com o mesmo
T nas duas juntas.

Lista completa de simplificações e do seu impacto: `report/simplificacoes.md`.
Bibliografia verificada, com a utilidade de cada entrada: `report/bibliografia.md`.

## Versões

| Tag   | Conteúdo |
|-------|----------|
| `pendulo-base` | pêndulo simples + pêndulo a bater na bola, validados |
| v13   | golfista 2 dof (braço + pulso) + testes de perturbação + relatório |
| v13-ar | + aerodinâmica (bola, braço e taco), medição do impacto corrigida, trajetórias |
| v17   | 3 dof (tronco + ombro esq. + pulso) + braço direito passivo; relatório LaTeX |
