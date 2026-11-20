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
# Pose "casa": todos os ensaios começam e acabam aqui. Os caminhos casa -> pose
# inicial de cada ensaio (e de volta) foram verificados no MuJoCo (folga 25 mm).
CASA = np.array([0.0, 10.0, 40.0, 0.0, 30.0, 0.0])


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
    ap.add_argument("--ja-confirmado", action="store_true", help="não pede 'sim' (usado por sessao.py, que já pediu)")
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
    if not a.ja_confirmado and input("Mão no botão de emergência. Escreva 'sim' para avançar: ").strip().lower() != "sim":
        arm.disconnect()
        return

    carimbo = datetime.now().strftime("%Y-%m-%d_%H%M%S")
    # 1) Casa, em duas etapas (braço, depois pulso), porque o caminho desde onde
    #    o robô estiver não é verificado; se o controlador recusar (C22 =
    #    autocolisão no modelo dele), fica-se a saber em qual.
    # 2) Casa -> pose inicial do ensaio, num só movimento (caminho verificado).
    atual = np.array(arm.get_servo_angle()[1][:6], float)
    etapas = [("casa: braço (J1-J3)", np.r_[CASA[:3], atual[3:]]), ("casa: pulso (J4-J6)", CASA),
              ("pose inicial do ensaio", q[0])]
    for nome_etapa, alvo in etapas:
        print(f"A ir a {V_APROX:.0f} °/s para {nome_etapa} -> {np.round(alvo, 1)}")
        code = arm.set_servo_angle(angle=[float(x) for x in alvo], speed=V_APROX, wait=True)
        if code != 0:
            print(f"PAROU na etapa {nome_etapa}: código {code}, erro do controlador {arm.error_code}, "
                  f"juntas {np.round(arm.get_servo_angle()[1][:6], 1)}")
            arm.disconnect()
            sys.exit(1)

    # Muda para o modo servo ANTES de começar a gravar: a mudança de modo faz o
    # controlador prender as juntas por um instante (um solavanco), e não pode
    # cair nos troços parados que medem o desvio do giroscópio.
    arm.set_mode(1)
    arm.set_state(0)
    time.sleep(1.0)

    grav = None
    if not a.sem_imu:
        from gravar import Gravador
        grav = Gravador(f"{a.ensaio}_imu", porta=a.porta, mostrar=False,
                        notas=f"ensaio {a.ensaio} no robô, lento {a.lento:g}")
        grav.caminho = DADOS / f"{carimbo}_{a.ensaio}_imu.csv"
        grav.iniciar()
        print(f"IMU a gravar: {grav.caminho.name}")

    # 3 s parado no início e no fim, sempre em modo servo (a segurar a pose)
    n_parado = int(round(3.0 / DT))
    seq = np.vstack([np.repeat(q[:1], n_parado, 0), q, np.repeat(q[-1:], n_parado, 0)])
    log = []
    completo = False
    t0 = time.perf_counter()
    try:
        for k in range(len(seq)):
            code = arm.set_servo_angle_j([float(x) for x in seq[k]], is_radian=False)
            if code != 0:
                print(f"PAROU no ponto {k}/{len(seq)} (t = {k * DT - 3.0:.2f} s do plano): código {code}, "
                      f"erro {arm.error_code}, aviso {arm.warn_code}")
                break
            _, med = arm.get_servo_angle(is_real=True)   # encoders, não o pedido
            log.append([time.time(), *seq[k], *med[:6]])
            while time.perf_counter() - t0 < (k + 1) * DT:
                pass
        else:
            completo = True
    except KeyboardInterrupt:
        print("Interrompido.")
    finally:
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
        arm.set_mode(0)                            # já fora da gravação
        arm.set_state(0)
        time.sleep(0.5)
        if completo and arm.error_code == 0:      # acabou o ensaio sem erro: volta a casa
            print(f"A voltar a casa a {V_APROX:.0f} °/s...")
            arm.set_servo_angle(angle=[float(x) for x in CASA], speed=V_APROX, wait=True)
        arm.disconnect()
        if not completo:
            sys.exit(1)


if __name__ == "__main__":
    main()
