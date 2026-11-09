#!/usr/bin/env bash
# Copia para Photos/ as figuras que o main.tex usa, a partir de Lab2/report/.
# Correr depois de regenerar qualquer figura (ver ../README.md).
#   bash report/ROB_LAB2_Report/copiar_figuras.sh      (de qualquer pasta)
set -euo pipefail
aqui="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$aqui/Photos"
for f in referenciais.png traj_cartesiana.png traj_juntas.png traj_3d.png \
         mj_sequencia_down_the_line.png mj_folgas.png run_servo.png ik_erros.png \
         v3_caminho.png mjv3demo_sequencia_face_on.png; do
  cp "$aqui/../$f" "$aqui/Photos/$f"
  echo "  Photos/$f"
done
