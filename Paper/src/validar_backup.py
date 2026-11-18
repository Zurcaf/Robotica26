"""Teste 1: o modelo chega aos pontos que o dono anterior ensinou ao robô real?

Os pontos estão no backup do controlador (R1/Program/*.dat) como E6POS: posição
e orientação da ferramenta no referencial do mundo, mais o Status (S) e o Turn
(T), que dizem em que configuração o robô estava. O T dá o sinal de cada junta;
com o S, a solução da inversa é única. Para cada ponto procura-se uma solução
dentro dos limites com o mesmo S e o mesmo T. Corre-se com os dois sinais do
desvio de 55 mm: o certo tem de acertar em todos.

Só entram os pontos com BASE_NO 0 (referencial do mundo). A ferramenta é a
TOOL_DATA[1] do backup.

    .venv/bin/python src/validar_backup.py [pasta do backup]
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

import numpy as np

import kuka as K

BACKUP = Path("/Users/afons/Library/CloudStorage/GoogleDrive-afonsobpcruz@gmail.com/My Drive/"
              "1. Composites Arqu/1.Projectos/Kuka/BackUp Kuka/archive 2/KRC")
NUM = r"(-?[\d.]+(?:[eE][-+]?\d+)?)"


def ler_frame(txt, nome):
    m = re.search(re.escape(nome) + r"=\{X " + NUM + ",Y " + NUM + ",Z " + NUM +
                  ",A " + NUM + ",B " + NUM + ",C " + NUM, txt)
    return tuple(float(v) for v in m.groups())


def ler_pontos(pasta):
    pontos = []
    for f in sorted((pasta / "R1" / "Program").glob("*.dat")):
        txt = f.read_text(encoding="latin-1")
        for m in re.finditer(r"DECL E6POS X(\w+)=\{X " + NUM + ",Y " + NUM + ",Z " + NUM + ",A " + NUM +
                             ",B " + NUM + ",C " + NUM + r",S (\d+),T (\d+)", txt):
            nome = m.group(1)
            fd = re.search(r"DECL FDAT F" + re.escape(nome) + r"=\{TOOL_NO (\d+),BASE_NO (\d+)", txt)
            if fd is None:
                continue
            pontos.append(dict(fich=f.name, nome=nome, xyzabc=tuple(float(v) for v in m.groups()[1:7]),
                               S=int(m.group(8)), T=int(m.group(9)),
                               tool=int(fd.group(1)), base=int(fd.group(2))))
    return pontos


def main():
    pasta = Path(sys.argv[1]) if len(sys.argv) > 1 else BACKUP
    cfg = (pasta / "R1" / "System" / "$config.dat").read_text(encoding="latin-1")
    mada = (pasta / "R1" / "Mada" / "$machine.dat").read_text(encoding="latin-1")
    robroot = ler_frame(mada, "$ROBROOT")
    tools = {1: ler_frame(cfg, "TOOL_DATA[1]")}
    # o programa troca de base no código ($BASE=BASE_DATA[2] ou [3]) sem que o
    # ponto o registe: experimentam-se as três e diz-se em qual bate
    bases = {0: (0, 0, 0, 0, 0, 0), 2: ler_frame(cfg, "BASE_DATA[2]"), 3: ler_frame(cfg, "BASE_DATA[3]")}
    pontos = ler_pontos(pasta)
    usados = [p for p in pontos if p["base"] == 0 and p["tool"] in tools]
    print(f"{len(pontos)} pontos no backup; {len(usados)} no referencial do mundo (BASE_NO 0).")
    print(f"$ROBROOT = {robroot};  TOOL_DATA[1] = {tools[1]}")

    # 0) a inversa é a inversa da direta (juntas ao acaso dentro dos limites)
    rng = np.random.default_rng(0)
    pior = 0.0
    for _ in range(2000):
        q = rng.uniform(K.LIM[:, 0], K.LIM[:, 1])
        sols = K.ik(K.fk(q))
        ok = [s for s in sols if K.status(s) == K.status(q) and K.turn(s) == K.turn(q)]
        assert ok, f"inversa sem solução para {q}"
        pior = max(pior, min(np.abs(s - q).max() for s in ok))
    print(f"Direta -> inversa em 2000 poses ao acaso: maior diferença {pior:.1e}° (mesmo S e T).\n")

    W_R = np.linalg.inv(K.frame(*robroot))
    for sinal in (+1, -1):
        acertos = 0
        linhas = []
        for p in usados:
            for b, fb in bases.items():
                T_tcp = W_R @ K.frame(*fb) @ K.frame(*p["xyzabc"])
                T_fl = T_tcp @ np.linalg.inv(K.frame(*tools[p["tool"]]))
                sols = K.ik(T_fl, sinal)
                iguais = [s for s in sols if K.status(s, sinal) == p["S"] and K.turn(s) == p["T"]]
                if iguais:
                    break
            if iguais:
                acertos += 1
                q = iguais[0]
                err = np.linalg.norm(K.fk(q, sinal)[:3, 3] - T_fl[:3, 3])
                linhas.append(f"  {p['nome']:>6s} S{p['S']} T{p['T']:<3d} -> "
                              + " ".join(f"{v:7.1f}" for v in q) + f"   base {b}  (erro {err:.1e} mm)")
            else:
                st = sorted({(K.status(s, sinal), K.turn(s)) for s in sols})
                linhas.append(f"  {p['nome']:>6s} S{p['S']} T{p['T']:<3d} -> FALHA; soluções nos limites: "
                              + (", ".join(f"S{a} T{b}" for a, b in st) or "nenhuma"))
        print(f"Desvio de 55 mm com sinal {sinal:+d}: {acertos}/{len(usados)} pontos com o mesmo S e T")
        for l in linhas:
            print(l)
        print()


if __name__ == "__main__":
    main()
