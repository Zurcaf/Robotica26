# report/ — figuras e relatório

Todas as figuras desta pasta são geradas pelos scripts de `../src` (correr a
partir de `Lab2/`). Nenhuma é editada à mão: para mudar uma figura, muda-se o
script e volta-se a gerar.

- `ROB_LAB2_Report/` — o relatório: `main.tex`, `refs.bib`, `main.pdf` e
  `Photos/` (cópias das figuras que o relatório usa).
- `frames/` — poses soltas do MuJoCo (7 fases × 3 câmaras por swing). Não vão
  para o git; regeneram-se com `mujoco_preview.py --fotos`.

## Que script gera cada figura

| Figura | Comando | Swing | No relatório |
|---|---|---|---|
| `referenciais.png` | `python src/plot_frames.py` | — | Tarefa 1 |
| `traj_cartesiana.png`, `traj_juntas.png`, `traj_3d.png` | `python src/trajectory.py --plot` | V1 | Tarefa 2 |
| `mj_sequencia_<câmara>.png` | `python src/mujoco_preview.py --fotos --strip` | V1 | Tarefa 2 (`down_the_line`) |
| `mj_folgas.png` | `python src/mujoco_preview.py --folgas` | V1 | Tarefa 2 |
| `run_servo.png` | `python src/plot_run.py` | V1 | Tarefa 3 |
| `ik_erros.png` | `python src/ik_compare.py --from-csv` | V1 | Tarefa 4 |
| `v3_caminho.png`, `v3_juntas.png` | `python src/swing_v3.py --plot --bola-demo` | V3 (o que correu no robô) | secção 5 (`v3_caminho`) |
| `mjv3demo_sequencia_<câmara>.png`, `mjv3demo_folgas.png` | `python src/mujoco_preview.py --v3 --bola-demo --fotos --strip --folgas --prefixo mjv3demo_` | V3 (o que correu no robô) | secção 5 (`face_on`) |
| `v3_caminho_bola_lab.png`, `v3_juntas_bola_lab.png` | `python src/swing_v3.py --plot` | V3 com a bola marcada no lab | não |
| `mjv3_sequencia_<câmara>.png`, `mjv3_folgas.png` | `python src/mujoco_preview.py --v3 --fotos --strip --folgas --prefixo mjv3_` | V3 com a bola marcada no lab | não |
| `run_v3.png` | `python src/plot_run.py --v3` | V3 | não |
| `traj_golfista_*.png` | `python src/trajectory.py --plot --golfista` | GOLFISTA | não |
| `mjg_sequencia_<câmara>.png`, `mjg_folgas.png` | `python src/mujoco_preview.py --golfista --fotos --strip --folgas --prefixo mjg_` | GOLFISTA | não |

Câmaras: `face_on`, `down_the_line`, `cima`. Os comandos com `mujoco_preview.py`
precisam do pacote `mujoco` (está no `requirements.txt`).

## Atualizar o relatório depois de mudar uma figura

```bash
bash report/ROB_LAB2_Report/copiar_figuras.sh     # copia as figuras usadas para Photos/
cd report/ROB_LAB2_Report && latexmk -pdf main.tex && latexmk -c
```
