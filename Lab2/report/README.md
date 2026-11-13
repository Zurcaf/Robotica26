# report/ — figuras e relatório

Todas as figuras são geradas pelos scripts de `../src` (a partir de `Lab2/`).

- `ROB_LAB2_Report/` — relatório: `main.tex`, `secoes/`, `refs.bib`, `main.pdf`
  e `Photos/` (cópias das figuras usadas; `copiar_figuras.sh` atualiza-as).
- `frames/` — poses soltas do MuJoCo; não vão para o git.

| Figura | Comando | No relatório |
|---|---|---|
| `referenciais.png` | `python src/plot_frames.py` | Tarefa 1 |
| `traj_golfista_perto_{cartesiana,juntas,3d}.png` | `python src/trajectory.py --golfista --perto --plot` | Tarefa 2 |
| `mjgp_sequencia_<câmara>.png`, `mjgp_folgas.png` | `python src/mujoco_preview.py --golfista --perto --fotos --strip --folgas --prefixo mjgp_` | Tarefa 2 |
| `run_servo_golfista_perto.png` | `python src/plot_run.py --golfista-perto` | Tarefa 3 |
| `ik_erros_golfista_perto.png` | `python src/ik_compare.py --golfista --perto --from-csv` | Tarefa 4 |
| `traj_{cartesiana,juntas,3d}.png`, `mj_*.png`, `run_servo.png`, `ik_erros.png` | os mesmos scripts sem `--golfista --perto` | não (primeira versão) |

```bash
bash report/ROB_LAB2_Report/copiar_figuras.sh
cd report/ROB_LAB2_Report && latexmk -pdf main.tex && latexmk -c
```
