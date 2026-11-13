# O que foi feito no branch `feature/swing-golfista` (6 a 9 nov 2026)

> **Atualização de 13/11/2026.** Este documento é o registo de 6 a 9/11 e já não descreve o estado final.
> Depois disto: o swing do relatório passou a ser o `GOLFISTA_PERTO` (ensaios e conclusões em
> `data/LAB_2026-11-13.md`); o V3 foi abandonado e as opções `--v3`, `--dj2` e `--bola-demo` foram retiradas do
> código; os pontos da secção 8 ("O que falta") ficaram fechados ou sem efeito. A entrega é a pasta `entrega/`.

Para o Henrique e o Eduardo (e para mim daqui a uma semana). Resume o que
mudou neste branch em relação ao `Henrique`, onde está cada coisa e o que
falta. Tudo o que o Henrique fez está cá dentro: o branch já inclui os commits
dele até `2ef5277`.

## Em seis linhas

1. Os dois branches foram juntos. **Por omissão tudo corre a V1 final do
   Henrique**, com os mesmos números; nada do que ele mediu deixou de bater.
2. Há um terceiro swing, o **GOLFISTA** (robô direito, desenhado nas juntas),
   que só corre com `--golfista`. Ainda não foi ao robô.
3. Os três swings foram verificados no **MuJoCo** com o modelo 3D oficial:
   nenhum tem contacto.
4. Há um **rascunho do relatório** em `report/ROB_LAB2_Report/`, no formato do
   Lab 1, com a V1 como trajetória principal.
5. Foram criados os scripts que geram as figuras e os números que faltavam ao
   relatório (referenciais, pedido vs medido, folgas, ajuste dos D-H).
6. Falta uma ida ao laboratório para fechar três pontos (lista no fim).

## 1. Junção dos branches

Os dois branches tocavam nos mesmos ficheiros. Ficou assim:

| Ficheiro | Do Henrique | Meu |
|---|---|---|
| `src/trajectory.py` | pinça inclina sem girar (`n2 = z × d`), `phi_end = 50`, aviso de velocidade observada | modo juntas (`GOLFISTA`), modo híbrido, alcance medido ombro → centro do pulso, velocidade do flange no gráfico |
| `src/config.py` | velocidades confirmadas na documentação, `CART_SPEED_OBSERVED` | `REACH = 443.7` (braço esticado) e `REACH_FACTOR = 0.95` |
| `src/run_robot.py` | `pontos` só com `--lento ≥ 2`, CSV nunca reescrito | `--golfista`, `--k-phi` |

Verificação da junção: `python src/trajectory.py` dá os números do Henrique
(543 pontos, 220.9 / 169.9 mm/s, 94.7 °/s, folga de J5 44.1°) e
`python src/ik_compare.py --from-csv` dá os mesmos 1.36 mm.

Única diferença visível: a linha "alcance" passou de 270 mm (até ao flange,
limite 352) para 299 mm (até ao centro do pulso, limite 421.5). A medida
antiga chumbava poses que o robô fez sem problema.

## 2. Swing GOLFISTA

Desenhado nas juntas como o modelo do Lab 1: J1 = tronco, J2 = ombro,
J5 = pulso, J3 fixo em 110° (91 % da extensão), J4 = J6 = 0. A pinça chega à
bola a pique, com o braço quase esticado à frente do robô. Bola em
(390, 0, 80) mm. A trajetória cartesiana é a FK destes perfis.

```bash
python src/trajectory.py --plot --golfista
python src/mujoco_preview.py --golfista --view
python src/run_robot.py --go --metodo servo --lento 2 --golfista   # no laboratório
```

Estado: viável na verificação (288 mm/s na ponta, 36 °/s nas juntas) e sem
contacto no MuJoCo. **Sem dados do robô**, por isso não está no relatório.

## 3. Verificação no MuJoCo (`src/mujoco_preview.py`)

Reproduz as juntas no modelo oficial do Lite 6 com pinça (`models/lite6/`, do
`mujoco_menagerie`) e mede a distância entre as malhas de colisão de todos os
elos, da pinça (corpo e dedos) e da mesa. É a resposta ao "no dangerous
physical contact" da Tarefa 2.

| Swing | Folga mínima entre elos | Folga mínima à mesa |
|---|---|---|
| V1 | 25.3 mm (corpo da pinça – elo 4) | 75.1 mm |
| GOLFISTA | 25.4 mm | 81.4 mm |
| V3, bola da demonstração (o que correu no robô a 6/11) | 21.1 mm | **17.1 mm** (dedos, logo após o impacto) |
| V3, bola marcada no lab | 21.1 mm | 18.6 mm (corpo da pinça) |
| V3, bola da demonstração com `--dj2 -6` | 21.1 mm | 33.1 mm |

```bash
python src/mujoco_preview.py --view                 # V1 em loop
python src/mujoco_preview.py --v3 --bola-demo --view
python src/mujoco_preview.py --folgas               # gráfico da folga ao longo do tempo
```

Precisa do pacote `mujoco` (está no `requirements.txt`).

## 4. Relatório

`report/ROB_LAB2_Report/main.tex` (14 páginas, `main.pdf` ao lado). Mesmo
preâmbulo e estilo do Lab 1: uma secção por tarefa, comentário final,
contribuições, bibliografia.

- A trajetória do relatório é a **V1**: é a única com dados para as Tarefas 3
  e 4.
- A secção do **V3** está escrita mas desligada. Só entra se o swing for
  repetido no robô com 30 mm de folga à mesa. Para a ligar: no `main.tex`,
  trocar `\segundoswingfalse` por `\segundoswingtrue`.
- Os pontos por confirmar aparecem a vermelho no PDF (`\todo{...}`).
- Que script gera cada figura: `report/README.md`.

```bash
bash report/ROB_LAB2_Report/copiar_figuras.sh       # depois de regenerar figuras
cd report/ROB_LAB2_Report && latexmk -pdf main.tex && latexmk -c
```

## 5. Scripts novos

| Script | Para quê |
|---|---|
| `src/plot_frames.py` | figura dos referenciais D-H (Tarefa 1) |
| `src/plot_run.py` | Tarefa 3: pedido vs medido, atraso, erro por fase; `--v3` para o V3 |
| `src/ajuste_dh.py` | Tarefa 4: testa se a diferença para o controlador é geométrica |
| `src/mujoco_preview.py` | visualização 3D, colisões, `--folgas`, tiras de fotos |
| `src/robot_info.py` | lê a configuração do controlador sem mexer o robô |

Também: `ik_compare.py` desenha agora as duas FK, a diferença por eixo e os
pontos de pulso virado; `swing_v3.py`, `run_robot_v3.py` e `mujoco_preview.py`
aceitam `--dj2` (altura da ponta no impacto, sem mudar o comportamento por
omissão).

## 6. O que os dados dizem (recalculado dos CSV)

- **Tarefa 3, V1.** A metade da velocidade: 0.54 mm médio, 2.5 máx, atraso de
  10 ms. À velocidade planeada: 8.4 mm médio, 28 máx. O erro está no backswing
  (11 mm) e no follow-through (16 mm); no downswing é 1.4 mm. Não é saturação
  de velocidade (173 mm/s medidos, 171 pedidos) nem um atraso puro (descontar
  58 ms só baixa para 7.2 mm).
- **Tarefa 4.** IK do robô + FK do robô: 0.002 mm. IK do robô + nossa FK:
  1.36 mm médio, 2.02 máx, viés (−0.2, +0.6, −0.9) mm. Em 19 dos 60 pontos a
  IK devolve o pulso virado.
- **A diferença de 1.4 mm é geométrica.** Ajustando os parâmetros D-H aos 60
  pontos desce para 0.06 mm; nos dados do V3, que não entraram no ajuste,
  desce de 1.6 para 0.9 mm. Não sabemos que parâmetros são.

## 7. Dois erros meus, já corrigidos

- Disse que o controlador usava a geometria do URDF (243.5 / 200.2 / 227.61 /
  62.5 mm). Testei: não se confirma (na Tarefa 4 o erro até sobe para
  1.54 mm).
- A verificação de colisões não contava os dedos da pinça, só o corpo. Na V1
  e no GOLFISTA nada muda; no V3 a folga à mesa é 17.1 mm e não 21 mm.

## 8. O que falta

No laboratório:

1. `python src/robot_info.py` — lê a carga configurada (hipótese do C31) e os
   parâmetros D-H do controlador (fecha a Tarefa 4). Não mexe o robô.
2. V3 com a ponta mais alta, para poder entrar no relatório:
   `python src/run_robot_v3.py --go --lento 2 --bola-demo --dj2 -6`
   (a ponta passa a 61 mm da mesa em vez de 45; a bola tem de estar a essa
   altura). Depois `python src/plot_run.py --v3` com o CSV novo.
3. Opcional: GOLFISTA a `--lento 2` e `--lento 1`.

Fora do laboratório:

4. Rever as contribuições no relatório (a do Eduardo está em branco).
5. Rever o texto do relatório e apagar os `\todo`.

Feito a 12/11: limites das juntas confirmados nas especificações oficiais; a
bola é um ponto fixo no espaço (não houve objeto na mesa), e o relatório diz-o.
