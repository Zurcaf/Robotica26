"""
Robótica, Lab 1 — abre um modelo MuJoCo (XML) na app Simulate.

Uso:
    python3 simular_pendulo.py <modelo.xml>

Exemplos (a partir da pasta lab1):
    python3 simular_pendulo.py pendulo.xml
    python3 simular_pendulo.py exemplo_prof_hello.xml
"""

import argparse
import sys
from pathlib import Path

import mujoco
import mujoco.viewer

PASTA = Path(__file__).resolve().parent

parser = argparse.ArgumentParser(description="Abre um modelo MuJoCo na app Simulate.")
parser.add_argument("xml", help="ficheiro XML do modelo (ex.: pendulo.xml)")
args = parser.parse_args()

# Procura o ficheiro primeiro na pasta atual do terminal, depois na pasta deste script
xml = Path(args.xml)
if not xml.is_file():
    xml = PASTA / args.xml
if not xml.is_file():
    disponiveis = ", ".join(sorted(p.name for p in PASTA.glob("*.xml"))) or "nenhum"
    sys.exit(f"Erro: '{args.xml}' não encontrado.\nModelos em {PASTA.name}/: {disponiveis}")

try:
    model = mujoco.MjModel.from_xml_path(str(xml))   # lê e compila o XML
except ValueError as erro:
    sys.exit(f"Erro no XML '{xml.name}':\n{erro}")

data = mujoco.MjData(model)                          # estado da simulação
mujoco.viewer.launch(model, data)                    # abre a Simulate (bloqueia até fechar)
