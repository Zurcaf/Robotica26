"""Corre um ensaio no robô e grava a IMU ao mesmo tempo.

    .venv/bin/python src/robo/correr.py inclinacoes              # só mostra o que faria
    .venv/bin/python src/robo/correr.py inclinacoes --go         # liga-se e corre
    .venv/bin/python src/robo/correr.py rotacoes --go --lento 2  # metade da velocidade
    .venv/bin/python src/robo/correr.py poses --go --sem-imu     # sem Arduino

Antes: verificar.py (gera data/planos/<ensaio>.npz e verifica limites e
colisões com o suporte). Com --go: pede confirmação, leva o robô à pose
inicial junta a junta e devagar, começa a gravar a IMU, espera 3 s parado,
envia uma pose de juntas a cada 10 ms (set_servo_angle_j, modo 1) e lê as
juntas em cada ciclo. Acaba com 3 s parado e grava:
    data/<data>_<ensaio>_imu.csv    (gravar.py)
    data/<data>_<ensaio>_robo.csv   t_host, juntas pedidas, juntas medidas
As duas usam a hora do computador (t_host), que serve para as sincronizar.

Segurança: só se enviam planos que passaram em verificar.py. Mão no botão
de emergência. A IP do robô vem do Lab 2 (config.ROBOT_IP ou LITE6_IP).
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

AQUI = Path(__file__).resolve().parent
sys.path.insert(0, str(AQUI))
sys.path.insert(0, str(AQUI.parent))
import geometria as G            # noqa: E402  (põe Lab2/src no caminho)
from imu_io import DADOS         # noqa: E402

PLANOS = DADOS / "planos"
DT = 0.01
V_APROX = 20.0                   # °/s, ida à pose inicial


def carregar_plano(nome, lento=1.0):
    f = PLANOS / f"{nome}.npz"
    if not f.exists():
        raise SystemExit(f"Falta {f}: corra antes  .venv/bin/python src/robo/verificar.py {nome}")
    z = np.load(f)
    if not (np.allclose(z["R_F_IMU"], G.R_F_IMU) and np.allclose(z["P_F_IMU"], G.P_F_IMU)):
        raise SystemExit("O plano foi feito com outra montagem da IMU: corra verificar.py outra vez.")
    t, q = z["t"], z["q"]
    if lento != 1.0:
        tn = np.arange(0, t[-1] * lento + 1e-9, DT)
        q = np.column_stack([np.interp(tn / lento, t, q[:, j]) for j in range(6)])
        t = tn
    return t, q, [str(e) for e in z["etiquetas"]]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ensaio")
    ap.add_argument("--go", action="store_true", help="liga-se ao robô e corre")
    ap.add_argument("--lento", type=float, default=1.0, help="estica o tempo N vezes")
    ap.add_argument("--sem-imu", action="store_true", help="corre sem gravar a IMU")
    ap.add_argument("--porta", default=None, help="porta série do Arduino (por omissão: procura)")
    ap.add_argument("--ip", default=None)
    a = ap.parse_args()

    t, q, etiquetas = carregar_plano(a.ensaio, a.lento)
    v = np.abs(np.gradient(q, t, axis=0)).max()
    print(f"Ensaio {a.ensaio}: {t[-1]:.0f} s, {len(t)} poses, junta mais rápida {v:.0f} °/s"
          + (f" (esticado {a.lento:g}×)" if a.lento != 1 else ""))
    print(f"Pose inicial: {np.round(q[0], 1)}")
    for e in etiquetas:
        t0, t1, txt = e.split("|")
        print(f"  {float(t0) * a.lento:6.1f} – {float(t1) * a.lento:6.1f} s  {txt}")
    if not a.go:
        print("\nSem --go: não me liguei ao robô.")
        return

    from robot import check, connect          # Lab 2
    arm = connect(a.ip)
    print(f"\nLigado. Juntas atuais: {np.round(arm.get_servo_angle()[1][:6], 1)}")
    if input("Mão no botão de emergência. Escreva 'sim' para avançar: ").strip().lower() != "sim":
        arm.disconnect()
        return

    carimbo = datetime.now().strftime("%Y-%m-%d_%H%M%S")
    print(f"A ir para a pose inicial a {V_APROX:.0f} °/s...")
    check(arm.set_servo_angle(angle=[float(x) for x in q[0]], speed=V_APROX, wait=True), "set_servo_angle")

    grav = None
    if not a.sem_imu:
        from gravar import Gravador
        grav = Gravador(f"{a.ensaio}_imu", porta=a.porta, mostrar=False,
                        notas=f"ensaio {a.ensaio} no robô, lento {a.lento:g}")
        grav.caminho = DADOS / f"{carimbo}_{a.ensaio}_imu.csv"
        grav.iniciar()
        print(f"IMU a gravar: {grav.caminho.name}")
    time.sleep(3.0)

    log = []
    arm.set_mode(1)
    arm.set_state(0)
    time.sleep(0.5)
    t0 = time.perf_counter()
    try:
        for k in range(len(q)):
            code = arm.set_servo_angle_j([float(x) for x in q[k]], is_radian=False)
            if code != 0:
                print(f"PAROU no ponto {k}/{len(q)} (t = {t[k]:.2f} s): código {code}, "
                      f"erro {arm.error_code}, aviso {arm.warn_code}")
                break
            _, med = arm.get_servo_angle(is_real=True)   # encoders, não o pedido
            log.append([time.time(), *q[k], *med[:6]])
            while time.perf_counter() - t0 < (k + 1) * DT:
                pass
    except KeyboardInterrupt:
        print("Interrompido.")
    finally:
        arm.set_mode(0)
        arm.set_state(0)
        # 3 s parado no fim, a gravar (dá o desvio do giroscópio no fim do ensaio)
        t_fim = time.time() + 3.0
        while time.time() < t_fim:
            _, med = arm.get_servo_angle(is_real=True)   # encoders, não o pedido
            log.append([time.time(), *q[min(len(log), len(q) - 1)], *med[:6]])
            time.sleep(DT)
        if grav:
            grav.parar()
            print(f"IMU: {grav.n} amostras, {grav.falhas} falhas -> {grav.caminho}")
        if log:
            out = DADOS / f"{carimbo}_{a.ensaio}_robo.csv"
            cab = (f"ensaio={a.ensaio} lento={a.lento:g} imu={grav.caminho.name if grav else 'nenhum'}\n"
                   f"R_F_IMU={json.dumps(G.R_F_IMU.flatten().tolist(), separators=(',', ':'))} "
                   f"P_F_IMU={json.dumps(G.P_F_IMU.tolist(), separators=(',', ':'))}\n"
                   "t_host," + ",".join(f"q{j}_ref" for j in range(1, 7)) + ","
                   + ",".join(f"q{j}" for j in range(1, 7)))
            np.savetxt(out, np.array(log), delimiter=",", fmt="%.6f", header=cab, comments="# ")
            print(f"Robô: {len(log)} amostras -> {out}")
        arm.disconnect()


if __name__ == "__main__":
    main()
