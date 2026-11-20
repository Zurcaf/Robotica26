"""Tarefa 2 com o robô: calibração do acelerómetro e do giroscópio com a verdade
dada pela cinemática, calibrando numa sessão e verificando noutra.

    .venv/bin/python src/t2_robo.py                         # usa os ensaios de 20/11
    .venv/bin/python src/t2_robo.py --cal POSES ROTACOES --val POSES2

Cada argumento é o prefixo de um par de ficheiros (data/<prefixo>_imu.csv e
_robo.csv). Modelos (em g e °/s, no referencial nominal da IMU, R_F_IMU):

    acc  = M_a · g_verdade + b_a      g_verdade = gravidade vista pela IMU (parado)
    giro = M_g · ω_verdade + b_g      ω_verdade = velocidade angular do robô

M_a e M_g são matrizes 3x3 completas: juntam escala, eixos não perpendiculares
e a rotação da placa na peça. Parado = juntas do robô paradas (a IMU no robô
vibra com os servos, por isso não serve para o decidir). Grava
resultados/real/calibracao_robo.json, que t3_t4_orientacao.py usa com --cal.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from scipy.ndimage import uniform_filter1d
from scipy.spatial.transform import Rotation as Rot

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI))
sys.path.insert(0, str(AQUI / "robo"))
import calibracao as cal          # noqa: E402
import imu_io                     # noqa: E402
from imu_io import DADOS, LAB3    # noqa: E402
from verdade import hora_imu, ler_robo, verdade_para_imu   # noqa: E402

PADRAO = {"cal": ["2026-11-20_112716_poses", "2026-11-20_114952_rotacoes"],
          "val": ["2026-11-20_120614_poses"]}


def carregar(prefixo):
    reg = imu_io.ler(DADOS / f"{prefixo}_imu.csv")
    q, _, info = verdade_para_imu(reg, DADOS / f"{prefixo}_robo.csv")
    R = Rot.from_quat(q, scalar_first=True)
    # juntas paradas, no instante de cada amostra da IMU
    rob = ler_robo(DADOS / f"{prefixo}_robo.csv")
    tq = hora_imu(reg) + info["desfasamento_s"]
    vj = np.abs(np.gradient(rob["q"], rob["t_host"], axis=0)).max(1)
    vj = uniform_filter1d(vj, 25)
    parado_j = np.interp(tq, rob["t_host"], vj) < 0.2
    dentro = (tq > rob["t_host"][0]) & (tq < rob["t_host"][-1])
    # verdade de ω no referencial da IMU (°/s)
    w = (R[:-1].inv() * R[1:]).as_rotvec() / np.diff(reg.t)[:, None]
    w = np.degrees(np.vstack([w, w[-1:]]))
    return {"nome": prefixo, "reg": reg, "R": R, "w": w, "parado": parado_j & dentro, "dentro": dentro}


def troços(mask, fs, dur_min=1.5, margem=0.3):
    d = np.diff(np.concatenate([[0], mask.astype(int), [0]]))
    m = int(margem * fs)
    return [(a + m, b - m) for a, b in zip(np.flatnonzero(d == 1), np.flatnonzero(d == -1))
            if b - a - 2 * m >= dur_min * fs]


def medias_paradas(s):
    tr = troços(s["parado"], s["reg"].fs)
    g = np.array([s["R"][a:b].inv().apply([0, 0, 1.0]).mean(0) for a, b in tr])
    a = np.array([s["reg"].acc[a:b].mean(0) for a, b in tr])
    return g, a, tr


def ajustar_linear(x, y):
    """y ≈ M x + b, por mínimos quadrados. Devolve M, b."""
    X = np.hstack([x, np.ones((len(x), 1))])
    P = np.linalg.lstsq(X, y, rcond=None)[0]
    return P[:3].T, P[3]


def amostras_giro(s, suave=0.1):
    """ω medido e verdadeiro, alisados igual (a vibração dos servos é rápida)."""
    n = max(int(suave * s["reg"].fs), 1)
    gm = uniform_filter1d(s["reg"].gyro, n, axis=0)
    gv = uniform_filter1d(s["w"], n, axis=0)
    ok = s["dentro"].copy()
    ok[:n] = ok[-n:] = False
    return gv[ok], gm[ok]


def erro_acc(C, s):
    g, a, _ = medias_paradas(s)
    ac = C.corrigir_acc(a)
    ac /= np.linalg.norm(ac, axis=1, keepdims=True)
    ang = np.degrees(np.arccos(np.clip(np.sum(ac * g / np.linalg.norm(g, axis=1, keepdims=True), 1), -1, 1)))
    mod = np.linalg.norm(C.corrigir_acc(a), axis=1)
    return ang, mod


def erro_giro(C, s):
    gv, gm = amostras_giro(s)
    gc = C.corrigir_gyro(gm)
    mov = np.linalg.norm(gv, axis=1) > 5
    return np.linalg.norm(gc[mov] - gv[mov], axis=1), np.linalg.norm(gm[mov] - gv[mov], axis=1)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cal", nargs="+", default=PADRAO["cal"], help="prefixos para calibrar")
    ap.add_argument("--val", nargs="+", default=PADRAO["val"], help="prefixos para verificar")
    ap.add_argument("--saida", default=str(LAB3 / "resultados" / "real" / "calibracao_robo.json"))
    a = ap.parse_args()

    sc = [carregar(p) for p in a.cal]
    sv = [carregar(p) for p in a.val]

    # acelerómetro: médias dos troços parados de todas as sessões de calibração
    G, A = [], []
    for s in sc:
        g, ac, tr = medias_paradas(s)
        print(f"{s['nome']}: {len(tr)} troços parados (juntas paradas)")
        G.append(g), A.append(ac)
    M_a, b_a = ajustar_linear(np.vstack(G), np.vstack(A))
    # giroscópio: todas as amostras (alisadas) das sessões de calibração
    X, Y = zip(*[amostras_giro(s) for s in sc])
    M_g, b_g = ajustar_linear(np.vstack(X), np.vstack(Y))

    C = cal.Calibracao(bias_a=b_a.tolist(), M_a=M_a.tolist(), bias_g=b_g.tolist(), M_g=M_g.tolist(),
                       notas={"origem": "robô (t2_robo.py)", "calibracao": a.cal, "verificacao": a.val})
    C.guardar(a.saida)

    def resumo(M, nome):
        U, S, Vt = np.linalg.svd(M)
        rot = np.degrees(Rot.from_matrix(U @ Vt).as_rotvec())
        print(f"  {nome}: escalas {np.round(S, 4)}, rotação {np.round(rot, 2)}° ({np.linalg.norm(rot):.2f}°)")

    print("\nCalibração:")
    print(f"  desvio do acelerómetro: {np.round(b_a * 1000, 1)} mg")
    resumo(M_a, "acelerómetro")
    print(f"  desvio do giroscópio:   {np.round(b_g, 3)} °/s")
    resumo(M_g, "giroscópio  ")

    print("\nVerificação (sessões que não entraram na calibração):")
    sem = cal.Calibracao()
    for s in sv:
        ang0, mod0 = erro_acc(sem, s)
        ang1, mod1 = erro_acc(C, s)
        e1, e0 = erro_giro(C, s)
        print(f"  {s['nome']}:")
        print(f"    acelerómetro parado, ângulo à gravidade: sem calibrar {np.median(ang0):.2f}° "
              f"(máx {ang0.max():.2f}°), calibrado {np.median(ang1):.2f}° (máx {ang1.max():.2f}°)")
        print(f"    |a| parado: sem calibrar {mod0.min():.3f} a {mod0.max():.3f} g, "
              f"calibrado {mod1.min():.4f} a {mod1.max():.4f} g")
        print(f"    giroscópio em movimento, erro |ω|: sem calibrar {np.median(e0):.2f} °/s, "
              f"calibrado {np.median(e1):.2f} °/s (mediana)")
    print(f"\nGravado: {a.saida}")


if __name__ == "__main__":
    main()
