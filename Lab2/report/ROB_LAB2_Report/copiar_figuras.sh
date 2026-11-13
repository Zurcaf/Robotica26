#!/usr/bin/env bash
# Copia para Photos/ as figuras que o main.tex usa, a partir de Lab2/report/.
# Correr depois de regenerar qualquer figura (ver ../README.md).
#   bash report/ROB_LAB2_Report/copiar_figuras.sh      (de qualquer pasta)
set -euo pipefail
aqui="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$aqui/Photos"
for f in referenciais.png traj_golfista_perto_cartesiana.png traj_golfista_perto_3d.png \
         traj_golfista_perto_juntas.png mjgp_sequencia_face_on.png mjgp_folgas.png \
         run_servo_golfista_perto.png ik_erros_golfista_perto.png; do
  cp "$aqui/../$f" "$aqui/Photos/$f"
  echo "  Photos/$f"
done
