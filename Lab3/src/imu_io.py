"""Leitura e escrita dos ficheiros da IMU (gravados ou simulados).

Formato (CSV com cabeçalho de comentários):

    # imu_bruto v1 (Lab 3, grupo 37)
    # addr=0x69 whoami=0x11
    # acc_fs_g=4 gyro_fs_dps=500 rate_hz=200 baud=500000
    # (outras linhas '#' do gravador ou do simulador)
    t_us,ax,ay,az,temp,gx,gy,gz,t_host
    123456,512,-30,8190,-1200,3,-2,1,1731600000.123
    ...

As colunas ax..gz e temp são contagens int16 do ICM-20600; t_us é o relógio
do Arduino (µs, dá a volta aos 2^32); t_host é a hora do computador quando a
linha chegou (s), usada para sincronizar com o robô.
"""
from __future__ import annotations

import re
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

COLUNAS = ["t_us", "ax", "ay", "az", "temp", "gx", "gy", "gz", "t_host"]
G = 9.80665  # m/s² por g

LAB3 = Path(__file__).resolve().parents[1]
DADOS = LAB3 / "data"


def contagens_por_g(acc_fs_g: float) -> float:
    return 32768.0 / acc_fs_g


def contagens_por_dps(gyro_fs_dps: float) -> float:
    return 32768.0 / gyro_fs_dps


def temperatura_c(contagens):
    return np.asarray(contagens, float) / 326.8 + 25.0


@dataclass
class Registo:
    """Um ensaio já convertido: t [s], acc [g] (N,3), gyro [dps] (N,3), temp [°C]."""
    t: np.ndarray
    acc: np.ndarray
    gyro: np.ndarray
    temp: np.ndarray
    t_host: np.ndarray
    meta: dict = field(default_factory=dict)
    caminho: Path | None = None

    @property
    def fs(self) -> float:
        return 1.0 / np.median(np.diff(self.t))

    @property
    def dt(self) -> np.ndarray:
        return np.diff(self.t, prepend=self.t[0] - 1.0 / self.fs)

    def troço(self, t0: float, t1: float) -> "Registo":
        m = (self.t >= t0) & (self.t < t1)
        return Registo(self.t[m], self.acc[m], self.gyro[m], self.temp[m],
                       self.t_host[m], dict(self.meta), self.caminho)

    def __len__(self):
        return len(self.t)


def _ler_meta(linhas: list[str]) -> dict:
    meta = {}
    for ln in linhas:
        for k, v in re.findall(r"(\w+)=([^\s]+)", ln):
            try:
                meta[k] = float(v) if not v.startswith("0x") else int(v, 16)
            except ValueError:
                meta[k] = v
    return meta


def ler(caminho: str | Path) -> Registo:
    caminho = Path(caminho)
    comentarios, n_cab = [], 0
    with open(caminho) as f:
        for ln in f:
            n_cab += 1
            if ln.startswith("#"):
                comentarios.append(ln[1:].strip())
            else:
                nomes = ln.strip().split(",")
                break
    meta = _ler_meta(comentarios)
    meta["comentarios"] = comentarios
    x = np.loadtxt(caminho, delimiter=",", skiprows=n_cab, ndmin=2)
    d = {n: x[:, i] for i, n in enumerate(nomes)}
    t_us = d["t_us"].astype(np.int64)
    voltas = np.cumsum(np.diff(t_us, prepend=t_us[0]) < 0)   # micros() dá a volta
    t = (t_us + voltas * 2**32) * 1e-6
    t = t - t[0]
    kg = contagens_por_g(meta.get("acc_fs_g", 4))
    kw = contagens_por_dps(meta.get("gyro_fs_dps", 500))
    acc = np.column_stack([d["ax"], d["ay"], d["az"]]) / kg
    gyro = np.column_stack([d["gx"], d["gy"], d["gz"]]) / kw
    t_host = d["t_host"] if "t_host" in d else np.full(len(t), np.nan)
    return Registo(t, acc, gyro, temperatura_c(d["temp"]), t_host, meta, caminho)


def escrever(caminho: str | Path, t_s, acc_g, gyro_dps, temp_c, meta: dict,
             t_host=None, comentarios: list[str] = ()) -> Path:
    """Escreve no mesmo formato do gravador (quantizando para contagens)."""
    caminho = Path(caminho)
    caminho.parent.mkdir(parents=True, exist_ok=True)
    kg = contagens_por_g(meta["acc_fs_g"])
    kw = contagens_por_dps(meta["gyro_fs_dps"])
    sat = lambda x: np.clip(np.round(x), -32768, 32767).astype(np.int64)
    a = sat(np.asarray(acc_g) * kg)
    w = sat(np.asarray(gyro_dps) * kw)
    tc = sat((np.asarray(temp_c) - 25.0) * 326.8)
    t_us = (np.round(np.asarray(t_s) * 1e6).astype(np.int64)) % 2**32
    if t_host is None:
        t_host = np.asarray(t_s, float)
    with open(caminho, "w") as f:
        for c in comentarios:
            f.write(f"# {c}\n")
        f.write("# " + " ".join(f"{k}={v}" for k, v in meta.items()) + "\n")
        f.write(",".join(COLUNAS) + "\n")
        for i in range(len(t_us)):
            f.write(f"{t_us[i]},{a[i,0]},{a[i,1]},{a[i,2]},{tc[i]},"
                    f"{w[i,0]},{w[i,1]},{w[i,2]},{t_host[i]:.6f}\n")
    return caminho
