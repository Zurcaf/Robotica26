"""
Trajetória da bola: grava em CSV e compara com um ferro 7 típico.

Serve para avaliar o efeito da aerodinâmica. A ideia é correr uma vez antes de
a acrescentar ao modelo e outra vez depois, com etiquetas diferentes, e comparar
as duas no mesmo gráfico.

Cada corrida é um lote de swings com ruído no binário das juntas — o mesmo
"pulso trémulo" do enunciado que o perturbacoes.py usa —, por isso o que sai
não é uma curva mas um feixe de curvas, que mostra a dispersão.

Uso (a partir de Lab1/, NÃO de dentro de src/):
    python src/trajetoria.py --tag sem_ar     # 10 swings com ruído, grava
    python src/trajetoria.py --tag com_ar     # depois de acrescentar a aerodinâmica
    python src/trajetoria.py --tag nominal --std 0 --n 1   # sem ruído, 1 só
    python src/trajetoria.py --perfeita       # trajetória da tacada perfeita
    python src/trajetoria.py --so-grafico     # só redesenha a partir dos CSV

Produz:
    report/trajetoria_<tag>_NN.csv  t, x, y, z da bola durante o voo
    report/metricas_voo.csv         uma linha por corrida, com as métricas do voo
    report/trajetoria.png           gráfico com todas as etiquetas e a referência

As etiquetas de um mesmo lote (com_ar_01, com_ar_02, ...) são agrupadas na
mesma "família" no gráfico e na tabela: uma cor por família e média ± desvio
padrão em vez de dez linhas separadas.
"""

import argparse
import csv
import os
import numpy as np

from swing import load, run_swing, MODEL

REPORT = "report"

# ---------------------------------------------------------------------------
# Valores típicos de um ferro 7. TODO: confirmar numa fonte publicada (médias de
# monitor de lançamento) e citá-la no relatório — variam muito entre amador e
# profissional, e o prof vai perguntar de onde vêm.
# ---------------------------------------------------------------------------
# Gamas típicas de um ferro 7, do LPGA ao PGA Tour (ver PERFIS mais abaixo para
# a fonte). Servem só para as faixas do gráfico e para a coluna da direita da
# tabela; a comparação a sério é com a curva da tacada perfeita.
REF_FERRO7 = {
    "v_bola":  (46.5, 53.6),    # m/s
    "lancamento": (16.3, 19.0), # deg
    "spin":   (6699, 7097),     # rpm
    "carry":   (129, 157),      # m
    "apogeu":   (24, 29),       # m
    "t_voo":  (5.0, 6.5),       # s   (não publicado; estimado da integração)
    "descida":  (47, 50),       # deg
}
CAMPOS = ["tag", "v_cabeca", "v_bola", "smash", "lancamento", "spin",
          "carry", "apogeu", "t_voo", "descida", "land_y"]

# uma cor por família de etiquetas, atribuída por ordem alfabética da família
# (e não por ordem de chegada, para a cor de uma família não mudar quando se
# acrescenta outra)
CORES = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]


def familia(tag):
    """'com_ar_03' -> 'com_ar'; 'taco7' -> 'taco7'."""
    cabeca, _, cauda = tag.rpartition("_")
    return cabeca if cabeca and cauda.isdigit() else tag


# ---------------------------------------------------------------------------
# TACADA PERFEITA
#
# Em vez de desenhar uma curva "típica" com valores tabelados, lança-se a bola
# DENTRO do mesmo modelo MuJoCo, com as condições de um batimento perfeito, e
# deixa-se voar. Como é o mesmo integrador, o mesmo passo e a mesma
# aerodinâmica dos swings, a diferença entre as duas curvas é só o que o
# impacto produziu — e não uma diferença entre dois modelos de voo.
#
# Só três números são importados (velocidade da bola, ângulo de lançamento e
# spin). O carry, o apogeu, o tempo de voo e o ângulo de descida passam a ser
# PREVISÕES do modelo, que se comparam com a tabela para o validar.
#
# Fonte: Trackman Tour Averages (PGA e LPGA), tabela "7 Iron".
#   https://teeituprva.com/wp-content/uploads/2019/03/PGA-AVERAGES-INTERACTIVE.pdf
#   https://teeituprva.com/wp-content/uploads/2019/03/LPGA-AVERAGES-INTERACTIVE.pdf
# ---------------------------------------------------------------------------
PERFIS = {
    # entradas: o que se impõe à bola        | saídas publicadas: o que se verifica
    "lpga": dict(v_bola=46.5, lancamento=19.0, spin=6699,
                 carry=129.0, apogeu=24.0, descida=47.0,
                 fonte="LPGA Tour average, ferro 7"),
    "pga":  dict(v_bola=53.6, lancamento=16.3, spin=7097,
                 carry=157.0, apogeu=29.0, descida=50.0,
                 fonte="PGA Tour average, ferro 7"),
}


def tacada_perfeita(tag="perfeita", modelo=MODEL, perfil="lpga"):
    """Lança a bola no modelo com as condições de um batimento perfeito.

    O golfista fica congelado na pose de endereço (as suas 2 juntas levam
    posição e velocidade zero a cada passo), por isso só a bola se move.
    """
    import mujoco

    p = PERFIS[perfil]
    m, d = load(modelo)
    b_ball = m.body("ball").id
    g_ball, g_floor = m.geom("ball_geom").id, m.geom("floor").id
    adr = m.jnt_dofadr[m.joint("ball_free").id]     # 1.os 3 dofs lineares, 3 angulares

    ang = np.radians(p["lancamento"])
    d.qvel[adr:adr + 3] = p["v_bola"] * np.array([np.cos(ang), 0.0, np.sin(ang)])
    # backspin: omega em -y faz a força de Magnus apontar para cima
    d.qvel[adr + 3:adr + 6] = [0.0, -p["spin"] * 2.0 * np.pi / 60.0, 0.0]

    t0, apogeu, traj, v_ant = d.time, 0.0, [], np.zeros(3)
    carry = t_voo = descida = np.nan
    while d.time - t0 < 20.0:
        d.qpos[:2] = 0.0                            # congela o golfista
        d.qvel[:2] = 0.0
        mujoco.mj_step(m, d)

        pos = d.body("ball").xpos
        apogeu = max(apogeu, float(pos[2]))
        tv = d.time - t0
        if not traj or tv - traj[-1][0] >= 0.002:
            traj.append([tv, float(pos[0]), float(pos[1]), float(pos[2])])

        tocou = any({d.contact[i].geom1, d.contact[i].geom2} == {g_ball, g_floor}
                    for i in range(d.ncon))
        if tocou and tv > 0.1:
            carry, t_voo = float(pos[0]), tv
            descida = float(np.degrees(np.arctan2(-v_ant[2],
                                                  np.hypot(v_ant[0], v_ant[1]))))
            break
        v_ant = d.cvel[b_ball][3:].copy()

    if np.isnan(carry):
        raise SystemExit("A bola não aterrou em 20 s — alguma coisa está mal.")

    os.makedirs(REPORT, exist_ok=True)
    caminho = os.path.join(REPORT, f"trajetoria_{tag}.csv")
    with open(caminho, "w", newline="") as f:
        wr = csv.writer(f)
        wr.writerow(["t_s", "x_m", "y_m", "z_m"])
        wr.writerows(np.round(traj, 4))
    print(f"tacada perfeita ({p['fonte']}, {len(traj)} pontos) -> {caminho}")
    print(f"  previsto pelo modelo : carry {carry:.1f} m, apogeu {apogeu:.1f} m, "
          f"descida {descida:.1f} deg")
    print(f"  publicado            : carry {p['carry']:.1f} m, "
          f"apogeu {p['apogeu']:.1f} m, descida {p['descida']:.1f} deg")

    return dict(tag=tag, v_cabeca="", smash="",
                v_bola=p["v_bola"], lancamento=p["lancamento"], spin=p["spin"],
                carry=round(carry, 2), apogeu=round(apogeu, 2),
                t_voo=round(t_voo, 3), descida=round(descida, 2), land_y=0.0)


def corre(tag, modelo=MODEL, n=10, junta="wrist", std=10.0):
    """Corre n swings com ruído, grava cada trajetória e devolve as métricas.

    O ruído é o mesmo do perturbacoes.py: gaussiano no binário comandado da
    junta escolhida, mantido constante durante 1/freq segundos (10 Hz por
    omissão, a banda do tremor fisiológico). std=0 corre o swing nominal.
    """
    m, d = load(modelo)
    ruido = {f"{junta}_torque_std": std} if std > 0 else None
    os.makedirs(REPORT, exist_ok=True)

    linhas = []
    for i in range(n):
        etiqueta = f"{tag}_{i+1:02d}" if n > 1 else tag
        r = run_swing(m, d, {}, noise=ruido, seed=i, guardar_traj=True)
        if not r["hit"] or np.isnan(r["carry"]):
            print(f"  {etiqueta}: FALHOU (não acertou na bola) — ignorada")
            continue

        caminho = os.path.join(REPORT, f"trajetoria_{etiqueta}.csv")
        with open(caminho, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "x_m", "y_m", "z_m"])
            w.writerows(np.round(r["traj"], 4))
        print(f"  {etiqueta}: carry {r['carry']:6.1f} m  "
              f"lateral {r['land_y']:+5.2f} m  -> {caminho}")

        linhas.append(dict(tag=etiqueta,
                           v_cabeca=round(r["v_head"], 2),
                           v_bola=round(r["v_ball"], 2),
                           smash=round(r["v_ball"] / r["v_head"], 3),
                           lancamento=round(r["launch"], 2),
                           spin=round(r["spin_rpm"], 0),
                           carry=round(r["carry"], 2),
                           apogeu=round(r["apogeu"], 2),
                           t_voo=round(r["t_voo"], 3),
                           descida=round(r["descida"], 2),
                           land_y=round(r["land_y"], 2)))

    if not linhas:
        raise SystemExit("Nenhum dos swings acertou na bola — nada para gravar.")
    return linhas


def guarda_metricas(linha):
    """Acrescenta (ou substitui) a linha desta etiqueta em metricas_voo.csv."""
    caminho = os.path.join(REPORT, "metricas_voo.csv")
    linhas = {}
    if os.path.exists(caminho):
        with open(caminho) as f:
            for r in csv.DictReader(f):
                linhas[r["tag"]] = r
    linhas[linha["tag"]] = {k: linha[k] for k in CAMPOS}
    with open(caminho, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CAMPOS)
        w.writeheader()
        for r in linhas.values():
            w.writerow(r)
    return caminho


def le_metricas():
    caminho = os.path.join(REPORT, "metricas_voo.csv")
    if not os.path.exists(caminho):
        return []
    with open(caminho) as f:
        return list(csv.DictReader(f))


def por_familia(metricas):
    """Agrupa as linhas do metricas_voo.csv por família, em ordem alfabética."""
    grupos = {}
    for m in metricas:
        grupos.setdefault(familia(m["tag"]), []).append(m)
    return {k: grupos[k] for k in sorted(grupos)}


def tabela(metricas):
    """Imprime média ± desvio padrão de cada família ao lado da referência."""
    nomes = {"v_bola": "v da bola [m/s]", "lancamento": "lançamento [deg]",
             "spin": "spin [rpm]", "carry": "carry [m]", "apogeu": "apogeu [m]",
             "t_voo": "tempo de voo [s]", "descida": "descida [deg]"}
    grupos = por_familia(metricas)

    cab = "".join(f"{f'{k} (n={len(v)})':>18}" for k, v in grupos.items())
    print(f"\n{'':20}{cab}{'taco 7 típico':>18}")
    for campo, nome in nomes.items():
        lo, hi = REF_FERRO7[campo]
        linha = f"{nome:20}"
        for membros in grupos.values():
            v = np.array([float(m[campo]) for m in membros])
            marca = " " if lo <= v.mean() <= hi else "*"
            celula = f"{v.mean():.1f}" if v.size == 1 else f"{v.mean():.1f}±{v.std():.1f}"
            linha += f"{celula:>17}{marca}"
        print(linha + f"{f'{lo}-{hi}':>18}")
    print("\n(* = média fora da gama típica; ± é 1 desvio padrão do lote)")


def grafico(metricas):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(9.5, 4.5), layout="constrained")

    # faixa de referência: onde um ferro 7 típico aterra e que altura atinge
    ax.axvspan(*REF_FERRO7["carry"], color="0.85", zorder=0)
    ax.axhspan(*REF_FERRO7["apogeu"], color="0.92", zorder=0)
    ax.text(np.mean(REF_FERRO7["carry"]), 1.0, "carry típico", ha="center",
            fontsize=8, color="0.35", zorder=1)
    ax.text(2, np.mean(REF_FERRO7["apogeu"]), "apogeu típico", va="center",
            fontsize=8, color="0.35", zorder=1)

    # Uma cor por família, uma entrada na legenda por família: com 10 curvas
    # por lote, uma cor por curva seria ilegível. As curvas de um lote vão
    # translúcidas e finas, para se ver onde o feixe é denso.
    for cor, (nome, membros) in zip(CORES, por_familia(metricas).items()):
        primeira = True
        for m in membros:
            caminho = os.path.join(REPORT, f"trajetoria_{m['tag']}.csv")
            if not os.path.exists(caminho):
                continue
            # ndmin=2: um swing que mal toca na bola gera um CSV com uma linha
            # só, e sem isto o loadtxt devolvia um vetor em vez de uma matriz
            dados = np.loadtxt(caminho, delimiter=",", skiprows=1, ndmin=2)
            if dados.shape[0] < 2:
                continue
            if primeira:
                carry = np.array([float(x["carry"]) for x in membros])
                rot = (f"{nome} — carry {carry.mean():.0f} m" if carry.size == 1
                       else f"{nome} (n={carry.size}) — carry "
                            f"{carry.mean():.0f} ± {carry.std():.0f} m")
            ax.plot(dados[:, 1], dados[:, 3], zorder=3, color=cor,
                    lw=2.0 if len(membros) == 1 else 1.2,
                    alpha=1.0 if len(membros) == 1 else 0.45,
                    label=rot if primeira else None)
            primeira = False

    ax.axhline(0, color="0.4", lw=0.8, zorder=2)
    ax.set_xlabel("distância [m]")
    ax.set_ylabel("altura [m]")
    ax.set_title("Trajetória da bola: os nossos swings vs. a tacada perfeita", loc="left")
    ax.legend(frameon=False)
    ax.grid(alpha=0.2, zorder=0)
    ax.spines[["top", "right"]].set_visible(False)
    caminho = os.path.join(REPORT, "trajetoria.png")
    fig.savefig(caminho, dpi=150)
    print(f"gráfico -> {caminho}")


def main():
    ap = argparse.ArgumentParser(description="Trajetória da bola e comparação")
    ap.add_argument("--tag", default="sem_ar",
                    help="etiqueta desta corrida (ex.: sem_ar, com_ar)")
    ap.add_argument("--modelo", default=MODEL, help="XML a usar")
    ap.add_argument("--n", type=int, default=10,
                    help="número de swings do lote (1 = sem numeração na etiqueta)")
    ap.add_argument("--junta", default="wrist", choices=["wrist", "shoulder"],
                    help="junta onde entra o ruído")
    ap.add_argument("--std", type=float, default=10.0,
                    help="desvio padrão do ruído no binário [N.m]; 0 = sem ruído")
    ap.add_argument("--perfeita", action="store_true",
                    help="lança a bola no modelo com as condições de uma tacada perfeita")
    ap.add_argument("--perfil", default="lpga", choices=list(PERFIS),
                    help="perfil publicado usado como tacada perfeita")
    ap.add_argument("--so-grafico", action="store_true",
                    help="não corre o swing; só redesenha a partir dos CSV")
    args = ap.parse_args()

    if args.perfeita:
        print(f"métricas -> "
              f"{guarda_metricas(tacada_perfeita(modelo=args.modelo, perfil=args.perfil))}")
    elif not args.so_grafico:
        print(f"{args.n} swing(s) com ruído de sigma = {args.std} N.m "
              f"no binário de '{args.junta}'")
        for linha in corre(args.tag, args.modelo, args.n, args.junta, args.std):
            caminho = guarda_metricas(linha)
        print(f"métricas -> {caminho}")

    metricas = le_metricas()
    if not metricas:
        raise SystemExit("Ainda não há métricas gravadas.")
    tabela(metricas)
    grafico(metricas)


if __name__ == "__main__":
    main()
