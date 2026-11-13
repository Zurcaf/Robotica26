"""
Banda sonora do vídeo dos ensaios gerada a partir dos dados (sem som da sala).

Uso (a partir de Lab2/):
    python src/video_som.py PASTA VIDEO_SEM_SOM.mp4 VIDEO_COM_SOM.mp4

Em cada ensaio ouve-se um zumbido de motor cuja frequência e volume seguem a
velocidade do flange MEDIDA pelo robô nesse ensaio (parado = silêncio; mais
rápido = mais agudo e forte), e um toque curto no instante do impacto. Na simulação segue a
velocidade planeada. Por baixo, um fundo grave e constante, muito baixo.
PASTA tem os clips (k*.mp4, m0.mp4, c*.mp4) e a lista.txt usados para montar o vídeo.
"""
import os
import subprocess
import sys
import wave

import numpy as np

from trajectory import GOLFISTA_PERTO, joint_trajectory, swing_trajectory, check_feasibility

SR = 44100
V_REF = 306.0                 # mm/s: velocidade máxima de todos os ensaios (tom mais agudo)
F0, GAMA = 48.0, 2.2          # Hz do zumbido parado e quanto sobe até V_REF (48 -> 154 Hz)
# clip -> (csv, lento, início do corte no vídeo original [s], instante do vídeo em que o swing começa [s])
ENSAIOS = {
    "c1": ("run_servo_lento2_golfista_perto_rep2.csv", 2.0, 4.75, 6.25),
    "c2": ("run_servo_lento1_golfista_perto_rep2.csv", 1.0, 0.00, 0.86),
    "c3": ("run_servo_lento0.7_golfista_perto.csv", 0.7, 0.58, 2.08),
    "c4": ("run_servo_lento2_golfista_perto_tampa.csv", 2.0, 4.90, 6.40),
}
DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "data")


def dur(f):
    return float(subprocess.check_output(["ffprobe", "-v", "error", "-show_entries", "format=duration", "-of", "csv=p=0", f]).decode())


def tom(v):
    """
    Som tipo motor elétrico a partir de um perfil de velocidade v [mm/s] amostrado a SR:
    um zumbido grave cuja frequência sobe com a velocidade (harmónicos em 1/k, como um
    dente de serra suave), um "gemido" de engrenagem mais agudo e baixo, e um pouco de ruído.
    """
    x = np.clip(v / V_REF, 0.0, 1.2)
    f = F0 * (1.0 + GAMA * x)
    fase = 2 * np.pi * np.cumsum(f) / SR
    zumbido = sum(np.sin(k * fase) / k for k in range(1, 9))
    gemido = 0.10 * np.sin(6.5 * fase + 0.6 * np.sin(2 * np.pi * 5.0 * np.arange(len(v)) / SR))
    rng = np.random.default_rng(0)
    ruido = np.convolve(rng.standard_normal(len(v)), np.ones(24) / 24, mode="same")   # ruído sem os agudos
    return 0.16 * x ** 0.7 * (zumbido + gemido) + 0.10 * x * ruido


def toque(n, k):
    """Toque curto (tipo madeira) a começar na amostra k."""
    y = np.zeros(n)
    m = min(int(0.25 * SR), n - k)
    if k < 0 or m <= 0:
        return y
    tt = np.arange(m) / SR
    y[k:k + m] = 0.5 * np.exp(-tt / 0.035) * (np.sin(2 * np.pi * 880 * tt) + 0.5 * np.sin(2 * np.pi * 1320 * tt))
    return y


def main():
    pasta, entrada, saida = sys.argv[1:4]
    nomes = [l.split("'")[1][:-4] for l in open(os.path.join(pasta, "lista.txt")) if l.strip()]
    G = GOLFISTA_PERTO
    partes = []
    for nome in nomes:
        n = int(round(dur(os.path.join(pasta, nome + ".mp4")) * SR))
        tau = np.arange(n) / SR
        y = np.zeros(n)
        if nome in ENSAIOS:
            csv, lento, a, t0 = ENSAIOS[nome]
            d = np.loadtxt(os.path.join(DATA, csv), delimiter=",")
            t, med = d[:, 0], d[:, 7:10]
            w = 5
            v = np.zeros_like(t)
            v[w:-w] = np.linalg.norm(med[2 * w:] - med[:-2 * w], axis=1) / (t[2 * w:] - t[:-2 * w])
            ts = a + tau - t0
            y = tom(np.interp(ts, t, v, left=0.0, right=0.0))
            t_imp = (G["t_back"] + G["t_down"]) * lento
            y += toque(n, int((t_imp + t0 - a) * SR))
        elif nome == "m0":                                   # simulação: velocidade planeada da ponta
            traj = swing_trajectory(G)
            qs = joint_trajectory(traj)[0]
            vt = check_feasibility(traj, qs, verbose=False)[1]["v_tcp"]
            y = tom(np.interp(tau - 1.0, traj["t"], vt, left=0.0, right=0.0))
            y += toque(n, int((G["t_back"] + G["t_down"] + 1.0) * SR))
        partes.append(y)
    y = np.concatenate(partes)
    tt = np.arange(len(y)) / SR
    fundo = 0.012 * (np.sin(2 * np.pi * 73.42 * tt) + 0.7 * np.sin(2 * np.pi * 110.0 * tt)) * (1 + 0.25 * np.sin(2 * np.pi * 0.11 * tt))
    y = y + fundo
    fade = int(1.5 * SR)
    y[:fade] *= np.linspace(0, 1, fade)
    y[-fade:] *= np.linspace(1, 0, fade)
    y = np.clip(y, -0.95, 0.95)
    wav = os.path.join(pasta, "som.wav")
    with wave.open(wav, "wb") as f:
        f.setnchannels(1)
        f.setsampwidth(2)
        f.setframerate(SR)
        f.writeframes((y * 32767).astype(np.int16).tobytes())
    subprocess.check_call(["ffmpeg", "-v", "error", "-y", "-i", entrada, "-i", wav, "-c:v", "copy", "-c:a", "aac", "-b:a", "160k",
                           "-shortest", saida])
    print(f"  {len(y) / SR:.1f} s de som -> {saida}")


if __name__ == "__main__":
    main()
