#!/usr/bin/env bash
# Cria Lab2/.venv e instala tudo (Mac / Linux).
# Uso, a partir de Lab2/:   bash setup.sh
set -euo pipefail
cd "$(dirname "$0")"

PY=$(command -v python3.11 || command -v python3)
"$PY" -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt

echo
echo "Pronto. Ativar com:  source .venv/bin/activate"
echo "Testar com:          python src/kinematics.py"
echo "Notebook:            jupyter lab lab2.ipynb"
