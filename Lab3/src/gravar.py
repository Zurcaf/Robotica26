"""Grava a IMU (sketch imu_bruto) para CSV.

Uso, a partir de Lab3/:

    .venv/bin/python src/gravar.py A_parado --dur 600
    .venv/bin/python src/gravar.py B_seis_posicoes          # até Ctrl+C
    .venv/bin/python src/gravar.py --listar                 # portas série

O ficheiro fica em data/AAAA-MM-DD_HHMMSS_<nome>.csv. Durante a gravação
mostra, uma vez por segundo, a taxa de amostragem, as falhas, o tamanho da
aceleração e a temperatura. A classe Gravador serve para gravar em paralelo
com o robô (src/robo/).
"""
from __future__ import annotations

import argparse
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import serial
from serial.tools import list_ports

sys.path.insert(0, str(Path(__file__).resolve().parent))
from imu_io import COLUNAS, DADOS, _ler_meta, contagens_por_dps, contagens_por_g, temperatura_c

BAUD = 500000


def encontrar_porta() -> str:
    portas = list(list_ports.comports())
    preferidas = [p for p in portas if any(s in (p.device + " " + (p.description or "")).lower()
                  for s in ("arduino", "usbmodem", "usbserial", "wchusb", "ch340"))]
    if preferidas:
        return preferidas[0].device
    if len(portas) == 1:
        return portas[0].device
    nomes = ", ".join(p.device for p in portas) or "nenhuma"
    raise SystemExit(f"Não encontrei o Arduino. Portas: {nomes}. Use --porta.")


class Gravador:
    """Lê a porta série numa thread e escreve o CSV.

    g = Gravador("D_robo"); g.iniciar(); ...; g.parar()
    """

    def __init__(self, nome: str, porta: str | None = None, baud: int = BAUD,
                 pasta: Path = DADOS, notas: str = "", mostrar: bool = True):
        self.porta = porta or encontrar_porta()
        self.baud = baud
        carimbo = datetime.now().strftime("%Y-%m-%d_%H%M%S")
        self.caminho = Path(pasta) / f"{carimbo}_{nome}.csv"
        self.notas = notas
        self.mostrar = mostrar
        self.n = 0
        self.falhas = 0          # amostras perdidas (saltos no t_us)
        self.partidas = 0        # linhas que não se conseguiram ler
        self.meta = {}
        self._parar = threading.Event()
        self._pronto = threading.Event()
        self._thread = None
        self._ultimo = None
        self._erro = None

    # ------------------------------------------------------------------ API
    def iniciar(self, espera_s: float = 5.0):
        self._thread = threading.Thread(target=self._correr, daemon=True)
        self._thread.start()
        if not self._pronto.wait(espera_s):
            self._parar.set()
            if self._erro:
                raise RuntimeError(f"Erro na porta {self.porta}: {self._erro}")
            raise RuntimeError("Sem cabeçalho do Arduino: o sketch imu_bruto está carregado? "
                               "O baud do sketch é igual ao do gravador?")
        return self

    def parar(self):
        self._parar.set()
        if self._thread:
            self._thread.join(timeout=3)
        return self.caminho

    def __enter__(self):
        return self.iniciar()

    def __exit__(self, *_):
        self.parar()

    # ------------------------------------------------------------- interno
    def _correr(self):
        try:
            self._ler_porta()
        except Exception as e:          # mostrado por iniciar()
            self._erro = e

    def _ler_porta(self):
        self.caminho.parent.mkdir(parents=True, exist_ok=True)
        with serial.Serial(self.porta, self.baud, timeout=0.5) as s, open(self.caminho, "w") as f:
            s.reset_input_buffer()          # abrir a porta reinicia o Uno
            comentarios = []
            t0 = time.time()
            while not self._parar.is_set():
                ln = s.readline().decode("ascii", "replace").strip()
                if ln.startswith("#"):
                    comentarios.append(ln[1:].strip())
                    if ln.startswith("# t_us"):
                        break
                elif time.time() - t0 > 4 and not comentarios:
                    return
            self.meta = _ler_meta(comentarios)
            for c in comentarios[:-1]:
                f.write(f"# {c}\n")
            f.write(f"# gravado={datetime.now().isoformat(timespec='seconds')} porta={self.porta}\n")
            if self.notas:
                f.write(f"# notas: {self.notas}\n")
            f.write(",".join(COLUNAS) + "\n")
            self._pronto.set()

            periodo_us = 1e6 / self.meta.get("rate_hz", 200)
            t_ant = None
            while not self._parar.is_set():
                raw = s.readline()
                agora = time.time()
                if not raw:
                    continue
                try:
                    v = [int(x) for x in raw.decode("ascii").strip().split(",")]
                    if len(v) != 8:
                        raise ValueError
                except ValueError:
                    self.partidas += 1
                    continue
                if t_ant is not None:
                    dt = (v[0] - t_ant) % 2**32
                    if dt > 1.5 * periodo_us:
                        self.falhas += int(round(dt / periodo_us)) - 1
                t_ant = v[0]
                f.write(",".join(map(str, v)) + f",{agora:.6f}\n")
                self.n += 1
                self._ultimo = v


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("nome", nargs="?", help="ex.: A_parado, B_seis_posicoes, F1_acenar_3")
    ap.add_argument("--dur", type=float, default=None, help="duração em s (sem: até Ctrl+C)")
    ap.add_argument("--porta", default=None)
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--notas", default="", help="texto livre que fica no cabeçalho")
    ap.add_argument("--listar", action="store_true", help="lista as portas série e sai")
    a = ap.parse_args()

    if a.listar:
        for p in list_ports.comports():
            print(f"{p.device:30s} {p.description}")
        return
    if not a.nome:
        ap.error("falta o nome do ensaio")

    g = Gravador(a.nome, a.porta, a.baud, notas=a.notas)
    print(f"Porta {g.porta} a {g.baud} baud. A esperar pelo Arduino...")
    g.iniciar()
    m = g.meta
    print(f"IMU: addr=0x{int(m.get('addr', 0)):X} whoami=0x{int(m.get('whoami', 0)):X} "
          f"(ICM-20600 = 0x11), ±{m.get('acc_fs_g')} g, ±{m.get('gyro_fs_dps')} dps, "
          f"{m.get('rate_hz')} Hz")
    print(f"A gravar para {g.caminho}  (Ctrl+C para parar)")
    kg, kw = contagens_por_g(m.get("acc_fs_g", 4)), contagens_por_dps(m.get("gyro_fs_dps", 500))
    t_ini, n_ant = time.time(), 0
    try:
        while a.dur is None or time.time() - t_ini < a.dur:
            time.sleep(1.0)
            taxa = g.n - n_ant
            n_ant = g.n
            if g._ultimo:
                v = g._ultimo
                acc = np.array(v[1:4]) / kg
                w = np.array(v[5:8]) / kw
                print(f"{time.time()-t_ini:6.0f} s  {taxa:4d} Hz  falhas {g.falhas:4d}  "
                      f"|a| {np.linalg.norm(acc):.3f} g  "
                      f"w ({w[0]:+6.2f} {w[1]:+6.2f} {w[2]:+6.2f}) dps  "
                      f"T {temperatura_c(v[4]):.1f} °C")
    except KeyboardInterrupt:
        pass
    g.parar()
    dur = time.time() - t_ini
    print(f"\n{g.n} amostras em {dur:.0f} s ({g.n/max(dur,1e-9):.0f} Hz); "
          f"falhas {g.falhas}, linhas partidas {g.partidas}.\nFicheiro: {g.caminho}")


if __name__ == "__main__":
    main()
