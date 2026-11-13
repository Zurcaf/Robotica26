# Quem escreve o quê (13/11, entrega às 23:59)

Cada secção está no seu ficheiro para trabalharmos ao mesmo tempo sem
conflitos. **Só mexer nos ficheiros da tua coluna.** Antes de cada commit:
`git pull --rebase`. Compilar: `latexmk -pdf main.tex` (em `ROB_LAB2_Report/`).

| Ficheiro | Quem | O que falta |
|---|---|---|
| `t1.tex` | Eduardo | só o bullet "Controlador do robô": 100 pontos, 1,19 mm; o controlador devolve a tabela nominal mas a pose a q = 0 é (86,82; −0,61; 154,32) |
| `t2.tex` | Afonso | reescrever para o GOLFISTA_PERTO (desenho nas juntas, viabilidade, MuJoCo) |
| `t3.tex` | Afonso | resultados de 13/11; a V1 passa a um parágrafo |
| `t4.tex` | Eduardo | reescrever com os 100 pontos de 13/11 e o `robot_info` |
| `comentario.tex` | Eduardo | comentário, contribuições, declaração de IA |
| `videos.tex` | Afonso | links do YouTube |
| `../main.tex` (resumo) | Afonso | no fim |
| `../refs.bib` | Eduardo | todas as fontes citadas |

Decisões: a trajetória das Tarefas 2–4 é o **GOLFISTA_PERTO** (swing nas
juntas, bola a 347 mm, ensaios de 13/11). A V1 (arco cartesiano, 5–6/11) fica
como primeira versão, num parágrafo da Tarefa 3. O V3 saiu. Relatório curto:
10–11 páginas. O que valorizam: comparação com o MuJoCo, método (prever →
medir → comparar), fontes.

## Números de 13/11 (fonte: `Lab2/data/LAB_2026-11-13.md`)

**Tarefa 4** (`data/ik_compare_golfista_perto.csv`, figura
`Photos/ik_erros_golfista_perto.png`), 100 pontos:
- IK do robô + FK do robô vs pedido: 0,1 µm médio, 1,1 µm máx.
- IK do robô + nossa FK vs pedido: 1,19 mm médio, 1,95 mm máx; 0,080° médio,
  0,146° máx; viés (0,32; 0,36; −0,86) mm.
- Pulso virado em 22 dos 100 pontos.
- Na V1 (60 pontos, 6/11): 1,36 / 2,02 mm; ajuste dos D-H baixa para 0,06 mm
  (`src/ajuste_dh.py`).

**Controlador** (`data/robot_info.txt`):
- `tcp_load` = 0,3 kg a (0; 0; 30) mm → a pinça ESTÁ declarada; a hipótese
  "carga não configurada" para o C31 fica refutada.
- D-H do controlador = tabela nominal (243,3 / 200 / 227,6 / 87 / 61,5). Mas com
  q = 0 a pose lida é (86,82; −0,61; 154,32) mm, não (87; 0; 154,2): há uma
  calibração por cima da tabela. É a origem dos ~1,2–1,4 mm.
- Limites configurados = fabricante: 500 mm/s, 5000 mm/s², 180 °/s, 1146 °/s².
  Não há nenhum limite de 220 mm/s.

**Tarefa 3** (flange medido vs pedido, servo cartesiano):

| `--lento` | Flange pedido | Erro médio / máx | Atraso |
|---|---|---|---|
| 2 (×2) | 108 mm/s | 0,56 / 1,08 mm | 10 ms |
| 1 (×2) | 215 mm/s | 1,1 / 2,2 mm | 10 ms |
| 0,7 | 306 mm/s | 1,6 / 3,1 mm | 10 ms |
| 2, com tampa | 107 mm/s | 0,56 / 1,07 mm | 10 ms |

V1 a 6/11, para comparação: 0,54 / 2,5 mm a meia velocidade; 8,4 / 28 mm à
planeada, com o flange a apenas 170 mm/s (os 298 mm/s eram da 1.ª orientação,
de 5/11, com 18,9 mm de erro).
