#!/usr/bin/env bash
# Cria Paper/.venv e instala tudo (Mac / Linux).
# Uso, a partir de Paper/:   bash setup.sh
set -euo pipefail
cd "$(dirname "$0")"
PY=$(command -v python3.12 || command -v python3)
"$PY" -m venv .venv
.venv/bin/python -m pip install --upgrade pip
.venv/bin/python -m pip install -r requirements.txt
echo "Pronto. Ativar com:  source .venv/bin/activate"
