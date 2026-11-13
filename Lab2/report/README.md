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
| `traj_golfista_perto_{cartesiana,juntas,3d}.png` | `python src/trajectory.py --golfista --perto --plot` | GOLFISTA_PERTO | Tarefa 2 |
| `mjgp_sequencia_<câmara>.png`, `mjgp_folgas.png` | `python src/mujoco_preview.py --golfista --perto --fotos --strip --folgas --prefixo mjgp_` | GOLFISTA_PERTO | Tarefa 2 (`face_on`, folgas) |
| `run_servo_golfista_perto.png` | `python src/plot_run.py --golfista-perto` | GOLFISTA_PERTO | Tarefa 3 |
| `ik_erros_golfista_perto.png` | `python src/ik_compare.py --golfista --perto --from-csv` | GOLFISTA_PERTO | Tarefa 4 |
| `traj_{cartesiana,juntas,3d}.png`, `run_servo.png`, `ik_erros.png` | os mesmos scripts sem `--golfista --perto` | primeira versão (arco cartesiano) | não |
| `video_endereco.jpg` | fotograma do vídeo do ensaio (cartão inicial do vídeo do YouTube) | GOLFISTA_PERTO | não |

Câmaras: `face_on`, `down_the_line`, `cima`. Os comandos com `mujoco_preview.py`
precisam do pacote `mujoco` (está no `requirements.txt`).

## Atualizar o relatório depois de mudar uma figura

```bash
bash report/ROB_LAB2_Report/copiar_figuras.sh     # copia as figuras usadas para Photos/
cd report/ROB_LAB2_Report && latexmk -pdf main.tex && latexmk -c
```
